#pragma once 

#include "Twist.h"
#include "CircularBuffer.h"
#include "time_utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace loam
{
/** IMU state data. */
typedef struct IMUState2
{
   /** The time of the measurement leading to this state (in seconds). */
   Time stamp;

   /** The current roll angle. */
   Angle roll;

   /** The current pitch angle. */
   Angle pitch;

   /** \brief Interpolate between two IMU states.
    *
    * @param start the first IMU state
    * @param end the second IMU state
    * @param ratio the interpolation ratio
    * @param result the target IMU state for storing the interpolation result
    */
   static void interpolate(const IMUState2& start,
                           const IMUState2& end,
                           const float& ratio,
                           IMUState2& result)
   {
      float invRatio = 1 - ratio;

      result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
      result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
   };
} IMUState2;

/**
 * @brief 基本雷达扫描
 */
class BasicLaserMapping
{
public:
   explicit BasicLaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief Try to process buffered data. */
   bool process(Time const& laserOdometryTime);
   void updateIMU(IMUState2 const& newState);
   void updateOdometry(double pitch, double yaw, double roll, double x, double y, double z);
   void updateOdometry(Twist const& twist);

   auto& laserCloud() { return *_laserCloudFullRes; }
   auto& laserCloudCornerLast() { return *_laserCloudCornerLast; }
   auto& laserCloudSurfLast() { return *_laserCloudSurfLast; }

   void setScanPeriod(float val) { _scanPeriod = val; }
   void setMaxIterations(size_t val) { _maxIterations = val; }
   void setDeltaTAbort(float val) { _deltaTAbort = val; }
   void setDeltaRAbort(float val) { _deltaRAbort = val; }

   auto& downSizeFilterCorner() { return _downSizeFilterCorner; }
   auto& downSizeFilterSurf() { return _downSizeFilterSurf; }
   auto& downSizeFilterMap() { return _downSizeFilterMap; }

   auto frameCount()    const { return _frameCount; }
   auto scanPeriod()    const { return _scanPeriod; }
   auto maxIterations() const { return _maxIterations; }
   auto deltaTAbort()   const { return _deltaTAbort; }
   auto deltaRAbort()   const { return _deltaRAbort; }

   auto const& transformAftMapped()   const { return _transformAftMapped; }
   auto const& transformBefMapped()   const { return _transformBefMapped; }
   auto const& laserCloudSurroundDS() const { return *_laserCloudSurroundDS; }

   bool hasFreshMap() const { return _downsizedMapCreated; }

private:
   /** Run an optimization. */
   void optimizeTransformTobeMapped();

   void transformAssociateToMap();
   void transformUpdate();
   void pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
   void pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
   void transformFullResToMap();

   bool createDownsizedMap();

   // private:
   size_t toIndex(int i, int j, int k) const
   { return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k; }

private:
   Time _laserOdometryTime;

   float _scanPeriod;          ///< time per scan
   const int _stackFrameNum;
   const int _mapFrameNum;
   long _frameCount;
   long _mapFrameCount;

   size_t _maxIterations;  ///< maximum number of iterations
   float _deltaTAbort;     ///< optimization abort threshold for deltaT
   float _deltaRAbort;     ///< optimization abort threshold for deltaR

   int _laserCloudCenWidth;
   int _laserCloudCenHeight;
   int _laserCloudCenDepth;
   const size_t _laserCloudWidth;
   const size_t _laserCloudHeight;
   const size_t _laserCloudDepth;
   const size_t _laserCloudNum;

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerLast;   ///< last corner points cloud
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfLast;     ///< last surface points cloud
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStack;
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStack;
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStackDS;  ///< down sampled
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStackDS;    ///< down sampled

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurround;
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;     ///< down sampled
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerFromMap;
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfFromMap;

   pcl::PointCloud<pcl::PointXYZI> _laserCloudOri;
   pcl::PointCloud<pcl::PointXYZI> _coeffSel;

   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfArray;
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerDSArray;  ///< down sampled
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfDSArray;    ///< down sampled

   std::vector<size_t> _laserCloudValidInd;
   std::vector<size_t> _laserCloudSurroundInd;

   Twist _transformSum, _transformIncre, _transformTobeMapped, _transformBefMapped, _transformAftMapped;

   CircularBuffer<IMUState2> _imuHistory;    ///< history of IMU states

   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner;   ///< voxel filter for down sizing corner clouds
   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf;     ///< voxel filter for down sizing surface clouds
   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterMap;      ///< voxel filter for down sizing accumulated map

   bool _downsizedMapCreated = false;
};

} // end namespace loam





