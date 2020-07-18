#include "loam_velodyne/MultiScanRegistration.h"
#include "math_utils.h"
#include <pcl_conversions/pcl_conversions.h>

namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
  : _lowerBound(lowerBound),
    _upperBound(upperBound),
    _nScanRings(nScanRings),
    _factor((nScanRings - 1) / (upperBound - lowerBound))
{

}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}



int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}


//构造函数
MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper) : _scanMapper(scanMapper){};

//设置
//读取参数服务器 订阅注册话题
bool MultiScanRegistration::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
  //创建 配准参数 对象
  RegistrationParams config;

  //调用该类的setupROS函数
  if (!setupROS(node, privateNode, config))
    return false;

  //调用祖父类（BasicScanRegistration）的configure函数
  //用于保存 参数对象
  configure(config);

  return true;
}

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
  process(laserCloudIn, fromROSTime(laserCloudMsg->header.stamp));
}

//处理点云数据
void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();

  // determine scan start and end orientations
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y, laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  pcl::PointXYZI point;

  //设置_laserCloudScans为激光雷达的线数
  _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());

  // clear all scanline points
  //说明：pcl::PointCloud的clear()方法：清除所有点，并且设宽度高度为0
  std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto &&v){v.clear();});

  // extract valid points from input cloud
  //对laserCloudIn中的所有点进行遍历
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper.getRingForAngle(angle);
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
      continue;
    }

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + relTime;

    projectPointToStartOfSweep(point, relTime);

    _laserCloudScans[scanID].push_back(point);
  }

  processScanlines(scanTime, _laserCloudScans);
  publishResult();
}

} // end namespace loam
