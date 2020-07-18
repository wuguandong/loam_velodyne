#ifndef LOAM_MULTISCANREGISTRATION_H
#define LOAM_MULTISCANREGISTRATION_H

#include "loam_velodyne/ScanRegistration.h"
#include <sensor_msgs/PointCloud2.h>

namespace loam {

/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring.
 *
 */
class MultiScanMapper {
public:
  /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  MultiScanMapper(const float& lowerBound = -15,
                  const float& upperBound = 15,
                  const uint16_t& nScanRings = 16);

  const float& getLowerBound() { return _lowerBound; }
  const float& getUpperBound() { return _upperBound; }
  const uint16_t& getNumberOfScanRings() { return _nScanRings; }

  /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  void set(const float& lowerBound,
           const float& upperBound,
           const uint16_t& nScanRings);

  /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
  int getRingForAngle(const float& angle);

  /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
  static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

  /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };


private:
  float _lowerBound;      ///< the vertical angle of the first scan ring
  float _upperBound;      ///< the vertical angle of the last scan ring
  uint16_t _nScanRings;   ///< number of scan rings
  float _factor;          ///< linear interpolation factor
};



/** \brief Class for registering point clouds received from multi-laser lidars.
 * 【多线扫描配准类】
 * 用于多线激光雷达点云配准的类
 */
class MultiScanRegistration : virtual public ScanRegistration {
public:
  //构造函数
  MultiScanRegistration(const MultiScanMapper& scanMapper = MultiScanMapper());

  //设置
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /** \brief Handler method for input cloud messages.
   *
   * @param laserCloudMsg the new input cloud message to process
   */
  void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

private:
  /** \brief Setup component in active mode.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  bool setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out) override;

  /** \brief Process a new input cloud.
   *
   * @param laserCloudIn the new input cloud to process
   * @param scanTime the scan (message) timestamp
   */
  void process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const Time& scanTime);

private:
  int _systemDelay = 20;             ///< system startup delay counter
  MultiScanMapper _scanMapper;  ///< mapper for mapping vertical point angles to scan ring IDs
  std::vector<pcl::PointCloud<pcl::PointXYZI> > _laserCloudScans;
  ros::Subscriber _subLaserCloud;   ///< input cloud message subscriber

};

} // end namespace loam


#endif //LOAM_MULTISCANREGISTRATION_H
