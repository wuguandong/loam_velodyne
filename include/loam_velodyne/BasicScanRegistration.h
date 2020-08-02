#pragma once

#include <utility>
#include <vector>

#include <pcl/point_cloud.h>

#include "Angle.h"
#include "Vector3.h"
#include "CircularBuffer.h"
#include "time_utils.h"

namespace loam
{



  /** \brief A pair describing the start end end index of a range. */
  typedef std::pair<size_t, size_t> IndexRange;



  /** Point label options. */
  //点的标签
  enum PointLabel
  {
    CORNER_SHARP = 2,       ///< sharp corner point 角点
    CORNER_LESS_SHARP = 1,  ///< less sharp corner point 次角点
    SURFACE_LESS_FLAT = 0,  ///< less flat surface point 平面点
    SURFACE_FLAT = -1       ///< flat surface point 次平面点
  };


  /** Scan Registration configuration parameters. */
  /**
   * 【配准参数类】
   */
  class RegistrationParams
  {
  public:
    //构造函数
    RegistrationParams(const float& scanPeriod_ = 0.1,  //扫描周期
      const int& imuHistorySize_ = 200,  //保存历史IMU数据的循环队列大小
      const int& nFeatureRegions_ = 6,  //划分区域的数量
      const int& curvatureRegion_ = 5,  //论文中S的大小
      const int& maxCornerSharp_ = 2,  //角点特征的数量
      const int& maxSurfaceFlat_ = 4,  //面点特征的数量
      const float& lessFlatFilterSize_ = 0.2,  //次面点体素滤波器的大小
      const float& surfaceCurvatureThreshold_ = 0.1);  //曲率阈值 高于则视为角点 低于则视为平面点

    /** The time per scan. */
    //扫描（scan）周期
    float scanPeriod;

    /** The size of the IMU history state buffer. */
    //保存历史IMU数据的循环队列大小
    int imuHistorySize;

    /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
    //特征区域数量：用于使得抽取的特征分布到各个区域的区域数量
    int nFeatureRegions;

    /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
    //曲率区域：一个点的周围点的数量（左右数量都是curvatureRegion），用于计算一个点的曲率
    int curvatureRegion;

    /** The maximum number of sharp corner points per feature region. */
    //尖锐角点的最大数量
    int maxCornerSharp;

    /** The maximum number of less sharp corner points per feature region. */
    //非尖锐角点的最大数量（为尖锐角点最大数量的10倍）
    int maxCornerLessSharp;

    /** The maximum number of flat surface points per feature region. */
    //平面点的最大数量
    int maxSurfaceFlat;

    /** The voxel size used for down sizing the remaining less flat surface points. */
    //不是很平的平面点的体素滤波器的大小
    float lessFlatFilterSize;

    /** The curvature threshold below / above a point is considered a flat / corner point. */
    float surfaceCurvatureThreshold;
  };



  /** IMU state data. */
  //IMU状态结构体
  typedef struct IMUState
  {
    /** The time of the measurement leading to this state (in seconds). */
    Time stamp;  //时间戳

    /** The current roll angle. */
    Angle roll;  //朝向角 R

    /** The current pitch angle. */
    Angle pitch;  //朝向角 P

    /** The current yaw angle. */
    Angle yaw;  //朝向角 Y

    /** The accumulated global IMU position in 3D space. */
    Vector3 position;  //位置

    /** The accumulated global IMU velocity in 3D space. */
    Vector3 velocity;  //速度

    /** The current (local) IMU acceleration in 3D space. */
    Vector3 acceleration;  //加速度

    /** \brief Interpolate between two IMU states.
     * 插值
    * 在两个IMU状态之间进行插值
    * @param start the first IMUState 第一个IMU状态
    * @param end the second IMUState 第二个IMU状态
    * @param ratio the interpolation ratio 插值时间在两个IMU状态之间的比例
    * @param result the target IMUState for storing the interpolation result 插值的结果 引用参数相当于返回值
    */
    static void interpolate(const IMUState& start, const IMUState& end, const float& ratio, IMUState& result)
    {
      //反转比例：距离第二状态的时间比例
      float invRatio = 1 - ratio;

      //线性插值
      //简述：
      //有两个状态，分别为start和end。要计算距离start为3/4（即距离end为1/4）时刻的状态result。
      //公式：result = start * 1/4 + end * 3/4
      //含义就是：距离哪个状态近，乘以它的比例就大。距离哪个状态远，乘以它的比例就小。

      //对朝向角进行插值
      result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
      result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
      if (start.yaw.rad() - end.yaw.rad() > M_PI)
      {
        result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() + 2 * M_PI) * ratio;
      }
      else if (start.yaw.rad() - end.yaw.rad() < -M_PI)
      {
        result.yaw = start.yaw.rad() * invRatio + (end.yaw.rad() - 2 * M_PI) * ratio;
      }
      else
      {
        result.yaw = start.yaw.rad() * invRatio + end.yaw.rad() * ratio;
      }

      //对速度进行插值
      result.velocity = start.velocity * invRatio + end.velocity * ratio;

      //对位置进行插值
      result.position = start.position * invRatio + end.position * ratio;
    };
  } IMUState;

  /**
   * 【基本扫描配准类】
   */
  class BasicScanRegistration
  {
  public:
    /** \brief Process a new cloud as a set of scanlines.
    *
    * @param relTime the time relative to the scan time
    */
    void processScanlines(const Time& scanTime, std::vector<pcl::PointCloud<pcl::PointXYZI>> const& laserCloudScans);

    bool configure(const RegistrationParams& config = RegistrationParams()); 

    /** \brief Update new IMU state. NOTE: MUTATES ARGS! */
    void updateIMUData(Vector3& acc, IMUState& newState);

    /** \brief Project a point to the start of the sweep using corresponding IMU data
    *
    * @param point The point to modify
    * @param relTime The time to project by
    */
    void projectPointToStartOfSweep(pcl::PointXYZI& point, float relTime);

    auto const& imuTransform          () { return _imuTrans             ; }
    auto const& sweepStart            () { return _sweepStart           ; }
    auto const& laserCloud            () { return _laserCloud           ; }
    auto const& cornerPointsSharp     () { return _cornerPointsSharp    ; }
    auto const& cornerPointsLessSharp () { return _cornerPointsLessSharp; }
    auto const& surfacePointsFlat     () { return _surfacePointsFlat    ; }
    auto const& surfacePointsLessFlat () { return _surfacePointsLessFlat; }
    auto const& config                () { return _config               ; }

  private:

    /** \brief Check is IMU data is available. */
    inline bool hasIMUData() { return _imuHistory.size() > 0; };

    /** \brief Set up the current IMU transformation for the specified relative time.
     *
     * @param relTime the time relative to the scan time
     */
    void setIMUTransformFor(const float& relTime);

    /** \brief Project the given point to the start of the sweep, using the current IMU state and position shift.
     *
     * @param point the point to project
     */
    void transformToStartIMU(pcl::PointXYZI& point);

    /** \brief Prepare for next scan / sweep.
     *
     * @param scanTime the current scan time
     * @param newSweep indicator if a new sweep has started
     */
    void reset(const Time& scanTime);

    /** \brief Extract features from current laser cloud.
     *
     * @param beginIdx the index of the first scan to extract features from
     */
    void extractFeatures(const uint16_t& beginIdx = 0);

    /** \brief Set up region buffers for the specified point range.
     *
     * @param startIdx the region start index
     * @param endIdx the region end index
     */
    void setRegionBuffersFor(const size_t& startIdx,
      const size_t& endIdx);

    /** \brief Set up scan buffers for the specified point range.
     *
     * @param startIdx the scan start index
     * @param endIdx the scan start index
     */
    void setScanBuffersFor(const size_t& startIdx,
      const size_t& endIdx);

    /** \brief Mark a point and its neighbors as picked.
     *
     * This method will mark neighboring points within the curvature region as picked,
     * as long as they remain within close distance to each other.
     *
     * @param cloudIdx the index of the picked point in the full resolution cloud
     * @param scanIdx the index of the picked point relative to the current scan
     */
    void markAsPicked(const size_t& cloudIdx,
      const size_t& scanIdx);

    /** \brief Try to interpolate the IMU state for the given time.
     *
     * @param relTime the time relative to the scan time
     * @param outputState the output state instance
     */
    void interpolateIMUStateFor(const float& relTime, IMUState& outputState);

    void updateIMUTransform();

  private:
    RegistrationParams _config;  ///< registration parameter

    pcl::PointCloud<pcl::PointXYZI> _laserCloud;   ///< full resolution input cloud
    std::vector<IndexRange> _scanIndices;          ///< start and end indices of the individual scans withing the full resolution cloud

    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;      ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp;  ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat;      ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat;  ///< less flat surface points cloud

    Time _sweepStart;            ///< time stamp of beginning of current sweep
    Time _scanTime;              ///< time stamp of most recent scan
    IMUState _imuStart;                     ///< the interpolated IMU state corresponding to the start time of the currently processed laser scan
    IMUState _imuCur;                       ///< the interpolated IMU state corresponding to the time of the currently processed laser scan point
    Vector3 _imuPositionShift;              ///< IMU位置的漂移 position shift between accumulated IMU position and interpolated IMU position
    size_t _imuIdx = 0;                         ///< the current index in the IMU history

    //历史IMU状态  CircularBuffer是自实现的循环队列
    CircularBuffer<IMUState> _imuHistory;   ///< history of IMU states for cloud registration

    pcl::PointCloud<pcl::PointXYZ> _imuTrans = { 4,1 };  ///< IMU的平移（类型可以看成三维向量的数组，长度为4） IMU transformation information

    std::vector<float> _regionCurvature;      ///< point curvature buffer
    std::vector<PointLabel> _regionLabel;     ///< point label buffer
    std::vector<size_t> _regionSortIndices;   ///< sorted region indices based on point curvature
    //用于记录该环中的哪些点是论文中的两类不可选点 1-被标记为不可选点
    std::vector<int> _scanNeighborPicked;     ///< flag if neighboring point was already picked
  };

}

