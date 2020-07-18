#include <ros/ros.h>
#include "loam_velodyne/MultiScanRegistration.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::MultiScanRegistration multiScan;

  //setup方法主要用来读取参数服务器、订阅注册话题
  if (multiScan.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
