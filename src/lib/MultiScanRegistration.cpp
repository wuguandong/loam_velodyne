#include "loam_velodyne/MultiScanRegistration.h"
#include "math_utils.h"
#include <pcl_conversions/pcl_conversions.h>

namespace loam {

//构造函数
MultiScanMapper::MultiScanMapper(const float& lowerBound, const float& upperBound, const uint16_t& nScanRings)
  : _lowerBound(lowerBound),
    _upperBound(upperBound),
    _nScanRings(nScanRings),
    _factor((nScanRings - 1) / (upperBound - lowerBound)){}

//Set函数（常规操作）
void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}


//给出俯仰角 返回其所属的环数
int MultiScanMapper::getRingForAngle(const float& angle) {
  //乘以180除以π：弧度制转换为角度制
  //减最低的俯仰角：得到相对仰角（相对于最低仰角的角）
  //乘以facor(每度有多少根线)：得到浮点类型的所属环数
  //+0.5再强转为int：相当于四舍五入取整
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}


//构造函数
//只干了一件事：把MultiScanMapper参数保存到成员变量
MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper) : _scanMapper(scanMapper){};

//设置
//读取参数服务器 订阅注册话题
bool MultiScanRegistration::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
  //创建 配准参数 对象
  //该类用成员变量保存一些参数 没有成员函数
  RegistrationParams config;

  //调用该类的setupROS函数
  if (!setupROS(node, privateNode, config))
    return false;

  //调用祖父类（BasicScanRegistration）的configure函数
  //用于保存 参数对象
  configure(config);

  return true;
}

//设置ROS
//参数：两个NodeHandle  一个RegistrationParams
bool MultiScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out)
{
  //调用父类的该函数
  //解析参数服务器  订阅IMU话题 注册话题
  if (!ScanRegistration::setupROS(node, privateNode, config_out))
    return false;

  // fetch scan mapping params
  std::string lidarName;

  //读取lidar参数 生成_scanMapper对象
  if (privateNode.getParam("lidar", lidarName)) {
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str());
      return false;
    }

    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());

    //解析Period参数 默认值0.1s
    if (!privateNode.hasParam("scanPeriod")) {
      config_out.scanPeriod = 0.1;
      ROS_INFO("Set scanPeriod: %f", config_out.scanPeriod);
    }
  }
  else {  //不在以上三种型号范围内时 手动设置雷达相关参数
    float vAngleMin, vAngleMax;  //最小、最大角度范围
    int nScanRings;  //雷达线数

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }
  }

  // subscribe to input cloud topic
  //订阅点云话题
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

  return true;
}

//点云订阅callback
void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  //系统开始延迟20个msg 前20个msg不处理
  if (_systemDelay > 0)
  {
    _systemDelay--;
    return;
  }

  // fetch new input cloud
  //将sensor_msgs::PointCloud2转换为pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  //处理点云数据
  //参数scanTime：点云msg的时间戳
  process(laserCloudIn, fromROSTime(laserCloudMsg->header.stamp));
}

//处理点云数据
//scanTime：时间戳
//注意：该时间戳为该帧点云结束时的时间，因此用它作为下一帧点云开始的时间，这是一个很巧妙的地方
void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();

  // determine scan start and end orientations
  //这里还是有点问题的：如果第一个点或最后一个点为无效点，这里计算出的startOri和endOri就会有误
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y, laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  }
  else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  //疑问：这个什么作用？
  //猜测：可能需要ori限制在一个范围内，方便用于计算relTime(相对时间)
  bool halfPassed = false;

  //设置_laserCloudScans为激光雷达的线数
  _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());

  // clear all scanline points
  //说明：pcl::PointCloud的clear()方法：清除所有点，并且设宽度高度为0
  std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto &&v){v.clear();});

  // extract valid points from input cloud
  //对laserCloudIn中的所有点进行遍历
  pcl::PointXYZI point;
  for (int i = 0; i < cloudSize; i++) {
    //获取第i个点 同时交换坐标轴 使得Z轴向前、Z轴向左、Y轴向上
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points
    //如果是无穷大的无效点 直接continue
    //pcl_isfinite()是PCL库自带的函数，用于判断一个浮点数是否为无穷大
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    //如果是接近0的无效点 直接continue
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
    //计算垂直角和所在环
    //疑问：为什么需要手动根据俯仰角计算该点是第几线?
    //回答：理想的情况下，16个线的数据是依次排列的，实际上点云msg中的点的个数每次都会少几十个，所以导致不能直接16个一组地取
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));  //俯仰角
    int scanID = _scanMapper.getRingForAngle(angle); //根据俯仰角获取该点所在的环数 scanID从下到上0~15
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){  //如果环数不在合理范围内，则直接continue
      continue;
    }

    // calculate horizontal point angle
    //计算水平角
    float ori = -std::atan2(point.x, point.z);  //Z轴为0° 逆时针为正 加了负号 顺时针为正
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      }
      else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    }
    else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      }
      else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    //根据点的水平朝向 计算该点扫描瞬间的相对时间
    //其中 config().scanPeriod是扫描一圈需要的时间
    float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);

    //给强度值赋值（不明白这样规定强度有什么作用）
    point.intensity = scanID + relTime;

    //使用IMU补偿加速度（如果没有IMU，这个函数会直接返回）
    projectPointToStartOfSweep(point, relTime);

    //将点存入所在的环
    _laserCloudScans[scanID].push_back(point);
  }

  //处理扫描线
  //BasicScanRegistration类的方法
  //scanTime来自该函数的参数
  processScanlines(scanTime, _laserCloudScans);

  //发布话题消息
  publishResult();
}

} // end namespace loam
