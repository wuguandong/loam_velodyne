#include <pcl/filters/voxel_grid.h>

#include "loam_velodyne/BasicScanRegistration.h"
#include "math_utils.h"

//临时
#include <cmath>

namespace loam
{

//构造函数
//只干了一件事：将参数保存到成员变量
RegistrationParams::RegistrationParams(const float& scanPeriod_,
                                       const int& imuHistorySize_,
                                       const int& nFeatureRegions_,
                                       const int& curvatureRegion_,
                                       const int& maxCornerSharp_,
                                       const int& maxSurfaceFlat_,
                                       const float& lessFlatFilterSize_,
                                       const float& surfaceCurvatureThreshold_)
    : scanPeriod(scanPeriod_),
      imuHistorySize(imuHistorySize_),
      nFeatureRegions(nFeatureRegions_),
      curvatureRegion(curvatureRegion_),
      maxCornerSharp(maxCornerSharp_),
      maxCornerLessSharp(10 * maxCornerSharp_),  //次角点的数量默认为角点数量的10倍
      maxSurfaceFlat(maxSurfaceFlat_),
      lessFlatFilterSize(lessFlatFilterSize_),
      surfaceCurvatureThreshold(surfaceCurvatureThreshold_)
{};

//处理扫描线
//scanTime：该帧点云数据msg的时间戳 即该帧点云数据扫描结束时的时刻
//laserCloudScans：按环分好的点云数组
void BasicScanRegistration::processScanlines(const Time& scanTime, std::vector<pcl::PointCloud<pcl::PointXYZI>> const& laserCloudScans)
{
  // reset internal buffers and set IMU start state based on current scan time
  //重置（更新或清空了一些成员变量的值）
  reset(scanTime);

  // construct sorted full resolution cloud
  size_t cloudSize = 0;  //用于下面for循环里累计点云数量
  //对每一个环进行遍历
  //把所有点放入_laserCloud  并用range记录每个环的下标范围
  for (int i = 0; i < laserCloudScans.size(); i++) {
    _laserCloud += laserCloudScans[i];  //PointCloud重载的+运算符

    IndexRange range(cloudSize, 0);  //std::pair<size_t, size_t>
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

  //抽取特征
  extractFeatures();

  //更新IMU变换
  updateIMUTransform();
}

bool BasicScanRegistration::configure(const RegistrationParams& config)
{
  //保存 配准参数对象 到成员变量中
  _config = config;

  _imuHistory.ensureCapacity(_config.imuHistorySize);
  return true;
}

//重置（在处理新的一帧点云之前，需要调一下该函数，重置一些成员变量）
//1. 更新_scanTime、_sweepStart为当前点云msg的时间戳
//2. 将_imuIdx重置为0
//3. 更新_imuStart为当前点云msg时间戳的插值状态
//4. 清空一些点云buffer
void BasicScanRegistration::reset(const Time& scanTime)
{
  _scanTime = scanTime;

  // re-initialize IMU start index and state
  _imuIdx = 0;
  if (hasIMUData()) {
    //_imuStart中保存本次已经处理完的扫描的开始时刻的IMU状态
    interpolateIMUStateFor(0, _imuStart);
  }

  // clear internal cloud buffers at the beginning of a sweep
  //清空一些点云buffer
  if (true/*newSweep*/) {
    //重置_sweepStart
    _sweepStart = scanTime;

    // clear cloud buffers
    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();

    // clear scan indices vector
    _scanIndices.clear();
  }
}

//更新IMU数据
// 1. IMU数据中补充 位置 和 速度 信息（通过累计）
// 2. 将当前IMU数据存入循环队列
void BasicScanRegistration::updateIMUData(Vector3& acc, IMUState& newState)
{
  //如果_imuHistory中存有上一时刻的IMU状态
  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time

    //将机器人坐标系下的加速度acc 转换为 世界坐标系下的加速度（仅仅在方向上）
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);

    //取出上一时刻的IMU状态
    const IMUState& prevState = _imuHistory.last();

    //计算时间差
    float timeDiff = toSec(newState.stamp - prevState.stamp);

    //计算当前状态的位置 使用的中学物理公式
    newState.position = prevState.position
                        + (prevState.velocity * timeDiff)
                        + (0.5 * acc * timeDiff * timeDiff);

    //计算当前状态的速度 使用的中学物理公式
    newState.velocity = prevState.velocity
                        + acc * timeDiff;
  }

  //如果_imuHistory为空 直接将IMU状态添加到_imuHistory
  _imuHistory.push(newState);
}


void BasicScanRegistration::projectPointToStartOfSweep(pcl::PointXYZI& point, float relTime)
{
  // project point to the start of the sweep using corresponding IMU data
  if (hasIMUData())
  {
    //对IMU状态进行插值
    setIMUTransformFor(relTime);


    transformToStartIMU(point);
  }
  else {
    std::cout<<"没有IMU历史数据"<<std::endl;
  }
}


void BasicScanRegistration::setIMUTransformFor(const float& relTime)
{
  //提示：_scanTime为上一帧点云结束的时刻，用来作为该帧点云开始的时刻

  //获取relTime时刻的插值得到的IMU状态 将结果保存到_imuCur中
  interpolateIMUStateFor(relTime, _imuCur);

  //_scanTime和_sweepStart永远相等（经过测试）
  //因此 relSweepTime = relTime
  float relSweepTime = toSec(_scanTime - _sweepStart) + relTime;

  //计算IMU位置的漂移
  //IMU状态的位置是根据速度、加速度进行计算累计的
  //如果插值的起始和终止IMU状态之间是匀速的，_imuPositionShift=0；如果是加速的_imuPositionShift>0
  //【TODO】不理解
  _imuPositionShift = _imuCur.position - _imuStart.position - _imuStart.velocity * relSweepTime;
}



void BasicScanRegistration::transformToStartIMU(pcl::PointXYZI& point)
{
  // rotate point to global IMU system
  //point的坐标目前是相对于_imuCur
  //对坐标轴进行旋转，顺时针
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);

  // add global IMU position shift
  point.x += _imuPositionShift.x();
  point.y += _imuPositionShift.y();
  point.z += _imuPositionShift.z();

  // rotate point back to local IMU system relative to the start IMU state
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);
}



void BasicScanRegistration::interpolateIMUStateFor(const float &relTime, IMUState &outputState)
{
  //思想：
  //由于点云数据处理比较慢，IMU状态一直在更新，所以本次正在处理的点云对应的IMU状态需要从历史IMU状态中依次遍历寻找
  //该帧点云数据的第一个点执行到该函数时，会执行while循环，找到合适的_imuIdx
  double timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  while (_imuIdx < _imuHistory.size() - 1 && timeDiff > 0) {
    _imuIdx++;
    timeDiff = toSec(_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  }

  //如果没有找到使得timeDiff<0的状态 则不进行差值 直接返回_imuIdx对应的IMU状态
  if (_imuIdx == 0 || timeDiff > 0) {
    outputState = _imuHistory[_imuIdx];
  }
  //满足插值的条件
  else {
    //计算该点扫描时刻在两个IMU状态时间之间的比例
    float ratio = -timeDiff / toSec(_imuHistory[_imuIdx].stamp - _imuHistory[_imuIdx - 1].stamp);
    //插值
    IMUState::interpolate(_imuHistory[_imuIdx], _imuHistory[_imuIdx - 1], ratio, outputState);
  }
}

//抽取特征
//参数beginIdx：可选参数，如果不传默认为0
void BasicScanRegistration::extractFeatures(const uint16_t& beginIdx)
{
  // extract features from individual scans
  //_scanIndices：存放每个环在点云中的起始和终止下标
  size_t nScans = _scanIndices.size();  //环的数量

  //对每个环进行遍历
  for(size_t i = beginIdx; i < nScans; i++) {
    //用于存放次平面点的点云（隔了很远才用到这个变量）
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);


    size_t scanStartIdx = _scanIndices[i].first;  //该环起点的数组下标
    size_t scanEndIdx = _scanIndices[i].second;  //该环终点的数组下标

    // skip empty scans
    //如果这个环中的点较少，则直接conintue
    //环中的点数至少为 2*curvatureRegion+1 才行
    if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion) {
      continue;
    }

    // Quick&Dirty fix for relative point time calculation without IMU data
    /*float scanSize = scanEndIdx - scanStartIdx + 1;
    for (int j = scanStartIdx; j <= scanEndIdx; j++) {
      _laserCloud[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize;
    }*/

    // reset scan buffers
    //排除掉论文中的两类不可选点（这个函数还是比较复杂的）
    setScanBuffersFor(scanStartIdx, scanEndIdx);

    // extract features from equally sized scan regions
    //j：当前是第几个区域，从0开始
    for(int j = 0; j < _config.nFeatureRegions; j++) {
      //sp和ep是相对于该sweep的下标，分别代表该区域的起始、终止下标
      //经过实测，分成了6个区域，前5个区域都为148个点，最后一个区域有149个点，[5, 152][153, 300]...[745, 893]
      size_t sp = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - j)
                   + (scanEndIdx - _config.curvatureRegion) * j) / _config.nFeatureRegions;
      size_t ep = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - 1 - j)
                   + (scanEndIdx - _config.curvatureRegion) * (j + 1)) / _config.nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;  //该区域的点数量

      // reset region buffers
      //计算该区域所有点的曲率，并排序
      setRegionBuffersFor(sp, ep);

      //吐槽：下标用的好乱啊，看得我头晕眼花！
      // extract corner features
      int largestPickedNum = 0;  //记录最大曲率（角点+次角点）已选择的个数

      //从右向左（曲率从大到小）对_regionSortIndices中存放的点的下标进行遍历
      for(size_t k = regionSize; k > 0 && largestPickedNum < _config.maxCornerLessSharp;) {
        size_t idx = _regionSortIndices[--k];  //该点在sweep中的下标
        size_t scanIdx = idx - scanStartIdx;  //该点在scan中的下标
        size_t regionIdx = idx - sp;  //该点在region中的下标

        //如果该点不是不可选点 并且 该点的曲率大于角点的阈值
        if (_scanNeighborPicked[scanIdx] == 0 && _regionCurvature[regionIdx] > _config.surfaceCurvatureThreshold) {
          largestPickedNum++;
          //如果角点还没选够
          if (largestPickedNum <= _config.maxCornerSharp) {
            //将该点标记为 角点
            _regionLabel[regionIdx] = CORNER_SHARP;

            //将该点放入_cornerPointsSharp中
            _cornerPointsSharp.push_back(_laserCloud[idx]);
          }
          //否则，即角点已经选够了
          else {
            //将该点标记为 次角点
            _regionLabel[regionIdx] = CORNER_LESS_SHARP;
          }

          //角点 和 次角点 都放入_cornerPointsLessSharp
          _cornerPointsLessSharp.push_back(_laserCloud[idx]);

          //标记该点 和该点左右几个点 为 不可选点
          //防止两个特征点之间的距离过小
          markAsPicked(idx, scanIdx);
        }
      }

      // extract flat surface features
      int smallestPickedNum = 0;  //记录最小曲率（平面点）已选择的个数

      //从左向右（曲率从小到大）对_regionSortIndices中存放的点的下标进行遍历
      for (int k = 0; k < regionSize && smallestPickedNum < _config.maxSurfaceFlat; k++) {
        size_t idx = _regionSortIndices[k];  //该点在sweep中的下标
        size_t scanIdx = idx - scanStartIdx;  //该点在scan中的下标
        size_t regionIdx = idx - sp;  //该点在region中的下标

        //如果该点不是不可选点 并且 该点的曲率小于平面点的阈值
        if (_scanNeighborPicked[scanIdx] == 0 && _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold) {

          smallestPickedNum++;

          //将该点标记为平面点
          _regionLabel[regionIdx] = SURFACE_FLAT;

          //将该点放入_surfacePointsFlat中
          _surfacePointsFlat.push_back(_laserCloud[idx]);

          //同样，标记该点 和该点左右几个点 为 不可选点
          //同样，防止两个特征点之间的距离过小
          markAsPicked(idx, scanIdx);
        }
      }

      // extract less flat surface features
      for (int k = 0; k < regionSize; k++) {
        //如果该点的标记是“次平面点”
        if (_regionLabel[k] <= SURFACE_LESS_FLAT) {  //其实这里直接用“=”效果相同
          //将该点放入surfPointsLessFlatScan（它在该函数的开头定义的）中
          surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
        }
      }

    }  //对region进行遍历结束

    // down size less flat surface point cloud of current scan
    pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;  //用于存放下采样之后的 次平面点

    //使用体素栅格对点云进行下采样
    //注意：下采样后的点，并不是原来点云的子集
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(_config.lessFlatFilterSize, _config.lessFlatFilterSize, _config.lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    _surfacePointsLessFlat += surfPointsLessFlatScanDS;
  }
}


//更新IMU变换
// 1. _imuTrans[0]存放该帧点云开始时刻的RPY角
// 2. _imuTrans[1]存放该帧点云结束时刻的RPY角
// 3. _imuTrans[2]存放该帧点云开始时刻到该帧点云结束时刻产生的漂移
// 4. _imuTrans[3]存放从该帧点云开始时刻到点云结束时刻的速度变换量
void BasicScanRegistration::updateIMUTransform()
{
  //_imuStart：该帧点云开始时刻
  _imuTrans[0].x = _imuStart.pitch.rad();
  _imuTrans[0].y = _imuStart.yaw.rad();
  _imuTrans[0].z = _imuStart.roll.rad();

  //_imuCur：该帧点云结束时刻
  _imuTrans[1].x = _imuCur.pitch.rad();
  _imuTrans[1].y = _imuCur.yaw.rad();
  _imuTrans[1].z = _imuCur.roll.rad();

  //_imuPositionShift：该帧点云开始时刻到该帧点云结束时刻产生的漂移（具体漂移是什么含义？）
  Vector3 imuShiftFromStart = _imuPositionShift;
  //将漂移向量转换到_imuStart坐标系下
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[2].x = imuShiftFromStart.x();
  _imuTrans[2].y = imuShiftFromStart.y();
  _imuTrans[2].z = imuShiftFromStart.z();

  //计算从该帧点云开始时刻到点云结束时刻的速度变换量
  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity;
  //将速度变化量转换到_imuStart坐标系下
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[3].x = imuVelocityFromStart.x();
  _imuTrans[3].y = imuVelocityFromStart.y();
  _imuTrans[3].z = imuVelocityFromStart.z();
}

//1. 计算区域中所有点的曲率，保存至_regionCurvature
//2. 按照曲率从小到大排序下标，保存至_regionSortIndices
//3. 重置_regionLabel
void BasicScanRegistration::setRegionBuffersFor(const size_t& startIdx, const size_t& endIdx)
{
  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize); //用来存放整个区域所有点的曲率的vector
  _regionSortIndices.resize(regionSize);  //用来存放整个区域所有点按照曲率从小到大排序后的下标
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT); //用来存放整个区域所有点的类型，默认所有点都为“次平面点”

  // calculate point curvatures and reset sort indices
  float pointWeight = -2 * _config.curvatureRegion;
  //对区域中的所有点进行遍历
  //i：该点在整个sweep中的下标
  //startIdx：该点在区域中的下标
  for(size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {

    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;

    //对其左右各curvatureRegion个点进行遍历，并累计到diff中
    for(int j = 1; j <= _config.curvatureRegion; j++) {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    //将该该点的曲率保存至_regionCurvature
    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;

    //将该点在整个sweep中的下标存入_regionSortIndices
    //_regionSortIndices里存放的都是sweep中的下标，而顺序接下来会按照曲率排序
    _regionSortIndices[regionIdx] = i;
  }

  // sort point curvatures
  //插入排序算法（类似扑克牌排序），目标：曲率由小到大
  for(size_t i = 1; i < regionSize; i++){
    for(size_t j = i; j >= 1; j--){
      //如果_regionSortIndices中第j个的曲率 小于 第j-1个的曲率 就交换
      if(_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
        //说明：std::swap() 既可以交换两个容器，也可以交换一个容器里的两个元素
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
      }
    }
  }
}

//找出两类不可选点，并标记好
void BasicScanRegistration::setScanBuffersFor(const size_t& startIdx, const size_t& endIdx)
{
  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;  //该环中的点数

  //_scanNeighborPicked：用于标记哪些点是不可选点（论文列举的两种点）1-被标记为不可选点
  //说明：std::vector的assign()方法：重新分配一块空间，里面存放scanSize个0，即初始假设所有点都是可选点
  _scanNeighborPicked.assign(scanSize, 0);

  // mark unreliable points as picked
  //对该环中的每个点进行遍历（最左边和最右边curvatureRegion个点不遍历）
  for(size_t i = startIdx + _config.curvatureRegion; i < endIdx - _config.curvatureRegion; i++){
    const pcl::PointXYZI& previousPoint = (_laserCloud[i - 1]);  //上一个点
    const pcl::PointXYZI& point = (_laserCloud[i]);  //当前遍历的点
    const pcl::PointXYZI& nextPoint = (_laserCloud[i + 1]);  //下一个点

    //当前遍历的点和下一个点的距离的平方
    float diffNext = calcSquaredDiff(nextPoint, point);

    //当前遍历的点和下一个点的距离大于1cm
    //意思是当前点和下一个点的距离比较大，有可能是遮挡区类型的不可选点
    if(diffNext > 0.1){
      float depth1 = calcPointDistance(point);  //当前遍历的点到原点的距离
      float depth2 = calcPointDistance(nextPoint);  //下一个点到原点的距离

      //当前遍历的点到原点的距离 大于 下一个点到原点的距离，说明右侧有物体遮挡背景墙
      //继续要判断是否当前遍历的点和下一个点之间的夹角比较小（因为有可能雷达传回来的数据中间丢了几个点，也有可能不同雷达夹角不同）
      if(depth1 > depth2) {
        //把当前遍历的点拉近到和下一个点到原点的距离相同，然后计算两点之间的距离的平方，再除以下一个点到原点的距离
        //近似于point和nextPoint之间的夹角
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        //如果夹角比较小 这个点被正式认为是不可选点
        if(weighted_distance < 0.1){
          //用_scanNeighborPicked记录一下这些点被认为是不可选点
          //范围是当前遍历的点开始一共 curvatureRegion + 1 个点
          std::fill_n(&_scanNeighborPicked[i - startIdx - _config.curvatureRegion], _config.curvatureRegion + 1, 1);

          continue;  //疑问：为什么这里有continue，而else里面没有？
        }
      }
      //当前遍历的点到原点的距离 小于或等于 下一个点到原点的距离，说明左侧有物体遮挡背景墙
      else{
        //把下一个点拉近到和当前遍历的点到原点的距离相同，然后计算两点之间的距离的平方，再除以下一个点到原点的距离
        //近似于point和nextPoint之间的夹角
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        //如果夹角比较小 这个点被认为是不可选点
        if(weighted_distance < 0.1){
          //用_scanNeighborPicked记录一下这些点被选中了
          //范围是当前遍历的点开始一共 curvatureRegion + 1 个点
          std::fill_n(&_scanNeighborPicked[i - startIdx + 1], _config.curvatureRegion + 1, 1);
        }
      }
    }

    //当前遍历的点和上一个点距离的平方
    float diffPrevious = calcSquaredDiff(point, previousPoint);

    //当前遍历的点到原点距离的平方
    float dis = calcSquaredPointDistance(point);

    //当前遍历的点到下一个点的距离大于1.4cm 并且 当前遍历的点到上一个点的距离大于1.4cm
    //其实就说明这个点属于几乎平行于扫描线类型的不可选点
    //例如一个沿着走廊行走的小车，其中打在远处侧墙的点就是这种不可选点
    if(diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
      _scanNeighborPicked[i - startIdx] = 1;
    }
  }
}


//1. 将该点标记为不可选点（其实是已经被选过了）
//2. 将该点左右的点标记为不可选点
void BasicScanRegistration::markAsPicked(const size_t& cloudIdx, const size_t& scanIdx)
{
  //将该点标记为 不可选点，即该点已经被选过了
  _scanNeighborPicked[scanIdx] = 1;

  //向右逐个遍历
  for(int i = 1; i <= _config.curvatureRegion; i++){
    //如果点与点的距离突然大于22厘米 则停止标记
    if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
      break;
    }

    //将点标记为不可选点
    _scanNeighborPicked[scanIdx + i] = 1;
  }

  //向左逐个遍历
  for (int i = 1; i <= _config.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx - i] = 1;
  }
}


}
