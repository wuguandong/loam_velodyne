#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H


#include "Twist.h"
#include "nanoflann_pcl.h"

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "BasicLaserOdometry.h"

namespace loam
{

  /** \brief Implementation of the LOAM laser odometry component.
   *
   */
  class LaserOdometry : public BasicLaserOdometry
  {
  public:
    explicit LaserOdometry(float scanPeriod = 0.1, uint16_t ioRatio = 2, size_t maxIterations = 25);

    /** \brief Setup component.
     *
     * @param node the ROS node handle
     * @param privateNode the private ROS node handle
     */
    virtual bool setup(ros::NodeHandle& node,
      ros::NodeHandle& privateNode);

    /** \brief Handler method for a new sharp corner cloud.
     *
     * @param cornerPointsSharpMsg the new sharp corner cloud message
     */
    void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg);

    /** \brief Handler method for a new less sharp corner cloud.
     *
     * @param cornerPointsLessSharpMsg the new less sharp corner cloud message
     */
    void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg);

    /** \brief Handler method for a new flat surface cloud.
     *
     * @param surfPointsFlatMsg the new flat surface cloud message
     */
    void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg);

    /** \brief Handler method for a new less flat surface cloud.
     *
     * @param surfPointsLessFlatMsg the new less flat surface cloud message
     */
    void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg);

    /** \brief Handler method for a new full resolution cloud.
     *
     * @param laserCloudFullResMsg the new full resolution cloud message
     */
    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

    /** \brief Handler method for a new IMU transformation information.
     *
     * @param laserCloudFullResMsg the new IMU transformation information message
     */
    void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg);


    /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
    void spin();

    /** \brief Try to process buffered data. */
    void process();

  protected:
    /** \brief Reset flags, etc. */
    void reset();

    /** \brief Check if all required information for a new processing step is available. */
    bool hasNewData();

    /** \brief Publish the current result via the respective topics. */
    void publishResult();

  private:
    uint16_t _ioRatio;       ///< ratio of input to output frames

    ros::Time _timeCornerPointsSharp;      ///< time of current sharp corner cloud
    ros::Time _timeCornerPointsLessSharp;  ///< time of current less sharp corner cloud
    ros::Time _timeSurfPointsFlat;         ///< time of current flat surface cloud
    ros::Time _timeSurfPointsLessFlat;     ///< time of current less flat surface cloud
    ros::Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
    ros::Time _timeImuTrans;               ///< time of current IMU transformation information

    bool _newCornerPointsSharp;       ///< flag if a new sharp corner cloud has been received
    bool _newCornerPointsLessSharp;   ///< flag if a new less sharp corner cloud has been received
    bool _newSurfPointsFlat;          ///< flag if a new flat surface cloud has been received
    bool _newSurfPointsLessFlat;      ///< flag if a new less flat surface cloud has been received
    bool _newLaserCloudFullRes;       ///< flag if a new full resolution cloud has been received
    bool _newImuTrans;                ///< flag if a new IMU transformation information cloud has been received

    nav_msgs::Odometry _laserOdometryMsg;       ///< laser odometry message
    tf::StampedTransform _laserOdometryTrans;   ///< laser odometry transformation

    ros::Publisher _pubLaserCloudCornerLast;  ///< last corner cloud message publisher
    ros::Publisher _pubLaserCloudSurfLast;    ///< last surface cloud message publisher
    ros::Publisher _pubLaserCloudFullRes;     ///< full resolution cloud message publisher
    ros::Publisher _pubLaserOdometry;         ///< laser odometry publisher
    tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster

    ros::Subscriber _subCornerPointsSharp;      ///< sharp corner cloud message subscriber
    ros::Subscriber _subCornerPointsLessSharp;  ///< less sharp corner cloud message subscriber
    ros::Subscriber _subSurfPointsFlat;         ///< flat surface cloud message subscriber
    ros::Subscriber _subSurfPointsLessFlat;     ///< less flat surface cloud message subscriber
    ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud message subscriber
    ros::Subscriber _subImuTrans;               ///< IMU transformation information message subscriber
  };

} // end namespace loam

#endif //LOAM_LASERODOMETRY_H
