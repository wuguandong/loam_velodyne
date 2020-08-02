#include <ros/ros.h>
#include "loam_velodyne/MultiScanRegistration.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");

  //节点句柄
  //命名空间为：launch文件指定的ns
  ros::NodeHandle node;

  //私有节点句柄
  //命名空间为：launch文件指定的ns/节点名称
  ros::NodeHandle privateNode("~");

  loam::MultiScanRegistration multiScan;

  //setup方法主要用来读取参数服务器、订阅注册话题
  //需要连个NodeHandle是因为一个用来订阅消息、一个用来解析参数，两者名称空间不同
  if (multiScan.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
