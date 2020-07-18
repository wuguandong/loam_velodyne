#include "loam_velodyne/ScanRegistration.h"
#include "math_utils.h"
#include <tf/transform_datatypes.h>

namespace loam {

//解析参数
//读取参数服务器中的参数，并保存到config_out中
bool ScanRegistration::parseParams(const ros::NodeHandle& nh, RegistrationParams& config_out) 
{
  bool success = true;
  int iParam = 0;
  float fParam = 0;

  if (nh.getParam("scanPeriod", fParam)) {  //scanPeriod参数实际为：/multiScanRegistration/scanPeriod
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      success = false;
    } else {
        config_out.scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (nh.getParam("imuHistorySize", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid imuHistorySize parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.imuHistorySize = iParam;
      ROS_INFO("Set imuHistorySize: %d", iParam);
    }
  }

  if (nh.getParam("featureRegions", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid featureRegions parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.nFeatureRegions = iParam;
      ROS_INFO("Set nFeatureRegions: %d", iParam);
    }
  }

  if (nh.getParam("curvatureRegion", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid curvatureRegion parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.curvatureRegion = iParam;
      ROS_INFO("Set curvatureRegion: +/- %d", iParam);
    }
  }

  if (nh.getParam("maxCornerSharp", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxCornerSharp parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.maxCornerSharp = iParam;
        config_out.maxCornerLessSharp = 10 * iParam;
      ROS_INFO("Set maxCornerSharp / less sharp: %d / %d", iParam, config_out.maxCornerLessSharp);
    }
  }

  if (nh.getParam("maxCornerLessSharp", iParam)) {
    if (iParam < config_out.maxCornerSharp) {
      ROS_ERROR("Invalid maxCornerLessSharp parameter: %d (expected >= %d)", iParam, config_out.maxCornerSharp);
      success = false;
    } else {
        config_out.maxCornerLessSharp = iParam;
      ROS_INFO("Set maxCornerLessSharp: %d", iParam);
    }
  }

  if (nh.getParam("maxSurfaceFlat", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxSurfaceFlat parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.maxSurfaceFlat = iParam;
      ROS_INFO("Set maxSurfaceFlat: %d", iParam);
    }
  }

  if (nh.getParam("surfaceCurvatureThreshold", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid surfaceCurvatureThreshold parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
        config_out.surfaceCurvatureThreshold = fParam;
      ROS_INFO("Set surfaceCurvatureThreshold: %g", fParam);
    }
  }

  if (nh.getParam("lessFlatFilterSize", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid lessFlatFilterSize parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
        config_out.lessFlatFilterSize = fParam;
      ROS_INFO("Set lessFlatFilterSize: %g", fParam);
    }
  }

  return success;
}

//设置ROS
//解析参数服务器  订阅话题 注册话题
bool ScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out)
{
  //解析参数
  if (!parseParams(privateNode, config_out))
    return false;

  // subscribe to IMU topic
  //订阅IMU的话题  包含朝向、线速度、角速度、线加速度、角加速度
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);

  // advertise scan registration topics
  //注册一些需要发布的话题
  _pubLaserCloud            = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp     = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat        = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat    = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans              = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  return true;
}


//IMU订阅者的callback函数
void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  //获取朝向角四元数
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);  //将geometry_msgs::Quaternion转化为tf::Quaternion

  //朝向角转换为欧拉角
  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  //加速度消除重力影响 博客：https://blog.csdn.net/l1323/article/details/106028032
  //交换坐标轴 使得Z轴朝前、X轴朝左、Y轴朝上
  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch)             * 9.81);

  //IMU状态结构体
  IMUState newState;
  newState.stamp = fromROSTime( imuIn->header.stamp);
  newState.roll = roll;  //朝向角
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;  //加速度

  //更新IMU数据  1. IMU数据中添加位置和速度信息 2. 将当前IMU存入循环队列
  updateIMUData(acc, newState);
}


void ScanRegistration::publishResult()
{
  auto sweepStartTime = toROSTime(sweepStart());
  // publish full resolution and feature point clouds
  publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(), sweepStartTime, "/camera");

  // publish corresponding IMU transformation information
  publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime, "/camera");
}

} // end namespace loam
