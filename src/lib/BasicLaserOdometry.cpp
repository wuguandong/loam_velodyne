#include "loam_velodyne/BasicLaserOdometry.h"

#include "math_utils.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace loam
{

using std::sin;
using std::cos;
using std::asin;
using std::atan2;
using std::sqrt;
using std::fabs;
using std::pow;


BasicLaserOdometry::BasicLaserOdometry(float scanPeriod, size_t maxIterations) :
   _scanPeriod(scanPeriod),
   _systemInited(false),
   _frameCount(0),
   _maxIterations(maxIterations),
   _deltaTAbort(0.1),
   _deltaRAbort(0.1),
   _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
   _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
   _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
   _surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
   _coeffSel(new pcl::PointCloud<pcl::PointXYZI>())
{}



void BasicLaserOdometry::transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   //计算该点在该scan过程中的时刻比例
   //pi.intensity - int(pi.intensity)：去除强度的小数部分，即该点的relTime
   //不如写成：float s = (pi.intensity - int(pi.intensity)) / _scanPeriod 更直观
   float s = (1.f / _scanPeriod) * (pi.intensity - int(pi.intensity));

   po.x = pi.x - s * _transform.pos.x();
   po.y = pi.y - s * _transform.pos.y();
   po.z = pi.z - s * _transform.pos.z();
   po.intensity = pi.intensity;

   Angle rx = -s * _transform.rot_x.rad();
   Angle ry = -s * _transform.rot_y.rad();
   Angle rz = -s * _transform.rot_z.rad();
   rotateZXY(po, rz, rx, ry);
}



size_t BasicLaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
   size_t cloudSize = cloud->points.size();

   for (size_t i = 0; i < cloudSize; i++)
   {
      pcl::PointXYZI& point = cloud->points[i];

      float s = (1.f / _scanPeriod) * (point.intensity - int(point.intensity));

      point.x -= s * _transform.pos.x();
      point.y -= s * _transform.pos.y();
      point.z -= s * _transform.pos.z();
      point.intensity = int(point.intensity);

      Angle rx = -s * _transform.rot_x.rad();
      Angle ry = -s * _transform.rot_y.rad();
      Angle rz = -s * _transform.rot_z.rad();
      rotateZXY(point, rz, rx, ry);
      rotateYXZ(point, _transform.rot_y, _transform.rot_x, _transform.rot_z);

      point.x += _transform.pos.x() - _imuShiftFromStart.x();
      point.y += _transform.pos.y() - _imuShiftFromStart.y();
      point.z += _transform.pos.z() - _imuShiftFromStart.z();

      rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart);
      rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);
   }

   return cloudSize;
}



void BasicLaserOdometry::pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                                           const Angle& blx, const Angle& bly, const Angle& blz,
                                           const Angle& alx, const Angle& aly, const Angle& alz,
                                           Angle &acx, Angle &acy, Angle &acz)
{
   float sbcx = bcx.sin();
   float cbcx = bcx.cos();
   float sbcy = bcy.sin();
   float cbcy = bcy.cos();
   float sbcz = bcz.sin();
   float cbcz = bcz.cos();

   float sblx = blx.sin();
   float cblx = blx.cos();
   float sbly = bly.sin();
   float cbly = bly.cos();
   float sblz = blz.sin();
   float cblz = blz.cos();

   float salx = alx.sin();
   float calx = alx.cos();
   float saly = aly.sin();
   float caly = aly.cos();
   float salz = alz.sin();
   float calz = alz.cos();

   float srx = -sbcx * (salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly)
      - cbcx * cbcz*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                     - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - cbcx * sbcz*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                     - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz);
   acx = -asin(srx);

   float srycrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                                  - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      - (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                        - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      + cbcx * sbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   float crycrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
                                                  - calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
      - (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
                                        - calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
      + cbcx * cbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
   acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

   float srzcrx = sbcx * (cblx*cbly*(calz*saly - caly * salx*salz) - cblx * sbly*(caly*calz + salx * saly*salz) + calx * salz*sblx)
      - cbcx * cbcz*((caly*calz + salx * saly*salz)*(cbly*sblz - cblz * sblx*sbly)
                     + (calz*saly - caly * salx*salz)*(sbly*sblz + cbly * cblz*sblx)
                     - calx * cblx*cblz*salz)
      + cbcx * sbcz*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
                     + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz)
                     + calx * cblx*salz*sblz);
   float crzcrx = sbcx * (cblx*sbly*(caly*salz - calz * salx*saly) - cblx * cbly*(saly*salz + caly * calz*salx) + calx * calz*sblx)
      + cbcx * cbcz*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
                     + (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly)
                     + calx * calz*cblx*cblz)
      - cbcx * sbcz*((saly*salz + caly * calz*salx)*(cblz*sbly - cbly * sblx*sblz)
                     + (caly*salz - calz * salx*saly)*(cbly*cblz + sblx * sbly*sblz)
                     - calx * calz*cblx*sblz);
   acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
}



void BasicLaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,
                                            Angle lx, Angle ly, Angle lz,
                                            Angle &ox, Angle &oy, Angle &oz)
{
   float srx = lx.cos()*cx.cos()*ly.sin()*cz.sin()
      - cx.cos()*cz.cos()*lx.sin()
      - lx.cos()*ly.cos()*cx.sin();
   ox = -asin(srx);

   float srycrx = lx.sin()*(cy.cos()*cz.sin() - cz.cos()*cx.sin()*cy.sin())
      + lx.cos()*ly.sin()*(cy.cos()*cz.cos() + cx.sin()*cy.sin()*cz.sin())
      + lx.cos()*ly.cos()*cx.cos()*cy.sin();
   float crycrx = lx.cos()*ly.cos()*cx.cos()*cy.cos()
      - lx.cos()*ly.sin()*(cz.cos()*cy.sin() - cy.cos()*cx.sin()*cz.sin())
      - lx.sin()*(cy.sin()*cz.sin() + cy.cos()*cz.cos()*cx.sin());
   oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

   float srzcrx = cx.sin()*(lz.cos()*ly.sin() - ly.cos()*lx.sin()*lz.sin())
      + cx.cos()*cz.sin()*(ly.cos()*lz.cos() + lx.sin()*ly.sin()*lz.sin())
      + lx.cos()*cx.cos()*cz.cos()*lz.sin();
   float crzcrx = lx.cos()*lz.cos()*cx.cos()*cz.cos()
      - cx.cos()*cz.sin()*(ly.cos()*lz.sin() - lz.cos()*lx.sin()*ly.sin())
      - cx.sin()*(ly.sin()*lz.sin() + ly.cos()*lz.cos()*lx.sin());
   oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}

void BasicLaserOdometry::updateIMU(pcl::PointCloud<pcl::PointXYZ> const& imuTrans)
{
   assert(4 == imuTrans.size());
   _imuPitchStart = imuTrans.points[0].x;
   _imuYawStart = imuTrans.points[0].y;
   _imuRollStart = imuTrans.points[0].z;

   _imuPitchEnd = imuTrans.points[1].x;
   _imuYawEnd = imuTrans.points[1].y;
   _imuRollEnd = imuTrans.points[1].z;

   _imuShiftFromStart = imuTrans.points[2];
   _imuVeloFromStart = imuTrans.points[3];
}

void BasicLaserOdometry::process()
{
  //如果还没有系统初始化，即第一次执行该函数，则进行初始化
  if (!_systemInited)
  {
    //将本次的 次角点 和 次平面点 保存到 上一次的角点 和 上一次的平面点
    //说明：PointCloud::swap() 交换两个点云
    _cornerPointsLessSharp.swap(_lastCornerCloud);
    _surfPointsLessFlat.swap(_lastSurfaceCloud);

    //初始化上一次角点、平面点的KD-tree
    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);

    _transformSum.rot_x += _imuPitchStart;
    _transformSum.rot_z += _imuRollStart;

    _systemInited = true;
    return;
  }

  pcl::PointXYZI coeff;
  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;

  _frameCount++;
  _transform.pos -= _imuVeloFromStart * _scanPeriod;


  size_t lastCornerCloudSize = _lastCornerCloud->points.size();  //上一次的次角点数量
  size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();  //上一次的次平面点的数量

  //如果上一次的角点数量>10且上一次的平面点数量>100
  if(lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100){
    std::vector<int> pointSearchInd(1);  //用于存放kd-tree搜索k最近邻后的结果下标
    std::vector<float> pointSearchSqDis(1);  //用于存放kd-tree搜索k最近邻后的结果距离
    std::vector<int> indices;  //removeNaNFromPointCloud的引用参数，没卵用

    pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);
    size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();  //本次角点的数量
    size_t surfPointsFlatNum = _surfPointsFlat->points.size();  //本次平面点的数量

    _pointSearchCornerInd1.resize(cornerPointsSharpNum);
    _pointSearchCornerInd2.resize(cornerPointsSharpNum);
    _pointSearchSurfInd1.resize(surfPointsFlatNum);
    _pointSearchSurfInd2.resize(surfPointsFlatNum);
    _pointSearchSurfInd3.resize(surfPointsFlatNum);

    //迭代
    for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++)
    {
      //pointSel：
      pcl::PointXYZI pointSel, pointProj, tripod1, tripod2, tripod3;
      _laserCloudOri->clear();
      _coeffSel->clear();

      //对每个角点进行遍历
      for (int i = 0; i < cornerPointsSharpNum; i++)
      {
        transformToStart(_cornerPointsSharp->points[i], pointSel);

        //每5次迭代执行一次
        if (iterCount % 5 == 0)
        {
          //去除上一次角点的NaN点（疑问：不是在话题回调函数里就去掉了吗？）
          pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices);

          //从上一次的角点中查找距离pointSel最近的点
          //查找结果的下标保存到pointSearchInd，查找结果的距离保存到pointSearchSqDis
          _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

          int closestPointInd = -1, minPointInd2 = -1;
          //如果距离最近的点的距离小于25m
          if (pointSearchSqDis[0] < 25)
          {
            closestPointInd = pointSearchInd[0];  //最近点的下标
            int closestPointScan = int(_lastCornerCloud->points[closestPointInd].intensity);  //最近点的scanID

            float pointSqDis, minPointSqDis2 = 25;

            //从 上一次的角点[最近点下标+1] 开始向右查找，...
            //疑问：cornerPointsSharpNum我觉得应该换成lastCornerCloudSize？？
            for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
            {
              //如果查找到了相隔两线以上的点，则break，终止查找
              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan + 2.5)
              {
                break;
              }

              //查找中的点和最近点的距离的平方
              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              //如果查找中的点的scanID大于最近点的scanID，主要是第二个点不能和第一个最近点在同一个scan上
              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan)
              {
                if (pointSqDis < minPointSqDis2)
                {
                  minPointSqDis2 = pointSqDis;  //更新minPointSqDis2为更小的距离
                  minPointInd2 = j;  //将当前查到的较好点的下标保存到minPointInd2
                }
              }
            }

            //从 上一次的角点[最近点下标-1] 开始向左查找，...
            for (int j = closestPointInd - 1; j >= 0; j--)
            {
              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan - 2.5)
              {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan)
              {
                if (pointSqDis < minPointSqDis2)
                {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }

          }  //if (pointSearchSqDis[0] < 25) end

          _pointSearchCornerInd1[i] = closestPointInd;  //论文中的点j的下标
          _pointSearchCornerInd2[i] = minPointInd2;  //论文中的点l的下标
        }

        //minPointInd2的初始值为-1，因此_pointSearchCornerInd2[i]<0表示没找到l点，即该角点找不到一致性
        if (_pointSearchCornerInd2[i] >= 0)
        {
          //单词tripod：三脚架
          tripod1 = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
          tripod2 = _lastCornerCloud->points[_pointSearchCornerInd2[i]];

          float x0 = pointSel.x;  //暂时称为点0
          float y0 = pointSel.y;
          float z0 = pointSel.z;
          float x1 = tripod1.x;  //暂时称为点1
          float y1 = tripod1.y;
          float z1 = tripod1.z;
          float x2 = tripod2.x;  //暂时称为点2
          float y2 = tripod2.y;
          float z2 = tripod2.z;

          //公式2的分子
          float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                            * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

          //点1和点2之间的距离，公式2的分母
          float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

          //la lb lc 这三个暂时用不到
          float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                      + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

          float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                       - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

          float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                       + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

          //点到直线的距离
          float ld2 = a012 / l12; // Eq. (2)

          // TODO: Why writing to a variable that's never read?
          pointProj = pointSel;
          pointProj.x -= la * ld2;
          pointProj.y -= lb * ld2;
          pointProj.z -= lc * ld2;

          float s = 1;
          if (iterCount >= 5)
          {
            s = 1 - 1.8f * fabs(ld2);
          }

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          if(s > 0.1 && ld2 != 0)
          {
            _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }  //对角点遍历结束

      //对每个平面点进行遍历
      for (int i = 0; i < surfPointsFlatNum; i++)
      {
        transformToStart(_surfPointsFlat->points[i], pointSel);

        if (iterCount % 5 == 0)
        {
          _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < 25)
          {
            closestPointInd = pointSearchInd[0];
            int closestPointScan = int(_lastSurfaceCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
            for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
            {
              if (int(_lastSurfaceCloud->points[j].intensity) > closestPointScan + 2.5)
              {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) <= closestPointScan)
              {
                if (pointSqDis < minPointSqDis2)
                {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
              else
              {
                if (pointSqDis < minPointSqDis3)
                {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--)
            {
              if (int(_lastSurfaceCloud->points[j].intensity) < closestPointScan - 2.5)
              {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) >= closestPointScan)
              {
                if (pointSqDis < minPointSqDis2)
                {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
              else
              {
                if (pointSqDis < minPointSqDis3)
                {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
          }

          _pointSearchSurfInd1[i] = closestPointInd;
          _pointSearchSurfInd2[i] = minPointInd2;
          _pointSearchSurfInd3[i] = minPointInd3;
        }

        if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0)
        {
          tripod1 = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
          tripod2 = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
          tripod3 = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

          float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
              - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
          float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
              - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
          float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
              - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
          float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

          float ps = sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd; //Eq. (3)??

          // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
          pointProj = pointSel;
          pointProj.x -= pa * pd2;
          pointProj.y -= pb * pd2;
          pointProj.z -= pc * pd2;

          float s = 1;
          if (iterCount >= 5)
          {
            s = 1 - 1.8f * fabs(pd2) / sqrt(calcPointDistance(pointSel));
          }

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          if (s > 0.1 && pd2 != 0)
          {
            _laserCloudOri->push_back(_surfPointsFlat->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }  //对平面点遍历结束

      int pointSelNum = _laserCloudOri->points.size();
      if (pointSelNum < 10)
      {
        continue;
      }

      //LM（具体过程没有看）
      Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);
      Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum);
      Eigen::Matrix<float, 6, 6> matAtA;
      Eigen::VectorXf matB(pointSelNum);
      Eigen::Matrix<float, 6, 1> matAtB;
      Eigen::Matrix<float, 6, 1> matX;

      for (int i = 0; i < pointSelNum; i++)
      {
        const pcl::PointXYZI& pointOri = _laserCloudOri->points[i];
        coeff = _coeffSel->points[i];

        float s = 1;

        float srx = sin(s * _transform.rot_x.rad());
        float crx = cos(s * _transform.rot_x.rad());
        float sry = sin(s * _transform.rot_y.rad());
        float cry = cos(s * _transform.rot_y.rad());
        float srz = sin(s * _transform.rot_z.rad());
        float crz = cos(s * _transform.rot_z.rad());
        float tx = s * _transform.pos.x();
        float ty = s * _transform.pos.y();
        float tz = s * _transform.pos.z();

        float arx = (-s * crx*sry*srz*pointOri.x + s * crx*crz*sry*pointOri.y + s * srx*sry*pointOri.z
                     + s * tx*crx*sry*srz - s * ty*crx*crz*sry - s * tz*srx*sry) * coeff.x
            + (s*srx*srz*pointOri.x - s * crz*srx*pointOri.y + s * crx*pointOri.z
               + s * ty*crz*srx - s * tz*crx - s * tx*srx*srz) * coeff.y
            + (s*crx*cry*srz*pointOri.x - s * crx*cry*crz*pointOri.y - s * cry*srx*pointOri.z
               + s * tz*cry*srx + s * ty*crx*cry*crz - s * tx*crx*cry*srz) * coeff.z;

        float ary = ((-s * crz*sry - s * cry*srx*srz)*pointOri.x
                     + (s*cry*crz*srx - s * sry*srz)*pointOri.y - s * crx*cry*pointOri.z
                     + tx * (s*crz*sry + s * cry*srx*srz) + ty * (s*sry*srz - s * cry*crz*srx)
                     + s * tz*crx*cry) * coeff.x
            + ((s*cry*crz - s * srx*sry*srz)*pointOri.x
               + (s*cry*srz + s * crz*srx*sry)*pointOri.y - s * crx*sry*pointOri.z
               + s * tz*crx*sry - ty * (s*cry*srz + s * crz*srx*sry)
               - tx * (s*cry*crz - s * srx*sry*srz)) * coeff.z;

        float arz = ((-s * cry*srz - s * crz*srx*sry)*pointOri.x + (s*cry*crz - s * srx*sry*srz)*pointOri.y
                     + tx * (s*cry*srz + s * crz*srx*sry) - ty * (s*cry*crz - s * srx*sry*srz)) * coeff.x
            + (-s * crx*crz*pointOri.x - s * crx*srz*pointOri.y
               + s * ty*crx*srz + s * tx*crx*crz) * coeff.y
            + ((s*cry*crz*srx - s * sry*srz)*pointOri.x + (s*crz*sry + s * cry*srx*srz)*pointOri.y
               + tx * (s*sry*srz - s * cry*crz*srx) - ty * (s*crz*sry + s * cry*srx*srz)) * coeff.z;

        float atx = -s * (cry*crz - srx * sry*srz) * coeff.x + s * crx*srz * coeff.y
            - s * (crz*sry + cry * srx*srz) * coeff.z;

        float aty = -s * (cry*srz + crz * srx*sry) * coeff.x - s * crx*crz * coeff.y
            - s * (sry*srz - cry * crz*srx) * coeff.z;

        float atz = s * crx*sry * coeff.x - s * srx * coeff.y - s * crx*cry * coeff.z;

        float d2 = coeff.intensity;

        matA(i, 0) = arx;
        matA(i, 1) = ary;
        matA(i, 2) = arz;
        matA(i, 3) = atx;
        matA(i, 4) = aty;
        matA(i, 5) = atz;
        matB(i, 0) = -0.05 * d2;
      }
      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;

      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      if (iterCount == 0)
      {
        Eigen::Matrix<float, 1, 6> matE;
        Eigen::Matrix<float, 6, 6> matV;
        Eigen::Matrix<float, 6, 6> matV2;

        Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
        matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();

        matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = { 10, 10, 10, 10, 10, 10 };
        for (int i = 0; i < 6; i++)
        {
          if (matE(0, i) < eignThre[i])
          {
            for (int j = 0; j < 6; j++)
            {
              matV2(i, j) = 0;
            }
            isDegenerate = true;
          }
          else
          {
            break;
          }
        }
        matP = matV.inverse() * matV2;
      }

      if (isDegenerate)
      {
        Eigen::Matrix<float, 6, 1> matX2(matX);
        matX = matP * matX2;
      }

      _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
      _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
      _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
      _transform.pos.x() += matX(3, 0);
      _transform.pos.y() += matX(4, 0);
      _transform.pos.z() += matX(5, 0);

      if (!pcl_isfinite(_transform.rot_x.rad())) _transform.rot_x = Angle();
      if (!pcl_isfinite(_transform.rot_y.rad())) _transform.rot_y = Angle();
      if (!pcl_isfinite(_transform.rot_z.rad())) _transform.rot_z = Angle();

      if (!pcl_isfinite(_transform.pos.x())) _transform.pos.x() = 0.0;
      if (!pcl_isfinite(_transform.pos.y())) _transform.pos.y() = 0.0;
      if (!pcl_isfinite(_transform.pos.z())) _transform.pos.z() = 0.0;

      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
        break;
    } //迭代for结束
  }  //上一次的 角点>10 且 平面点>100 if结束

  Angle rx, ry, rz;
  accumulateRotation(_transformSum.rot_x,
                     _transformSum.rot_y,
                     _transformSum.rot_z,
                     -_transform.rot_x,
                     -_transform.rot_y.rad() * 1.05,
                     -_transform.rot_z,
                     rx, ry, rz);

  Vector3 v(_transform.pos.x() - _imuShiftFromStart.x(),
            _transform.pos.y() - _imuShiftFromStart.y(),
            _transform.pos.z() * 1.05 - _imuShiftFromStart.z());
  rotateZXY(v, rz, rx, ry);
  Vector3 trans = _transformSum.pos - v;

  pluginIMURotation(rx, ry, rz,
                    _imuPitchStart, _imuYawStart, _imuRollStart,
                    _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                    rx, ry, rz);

  _transformSum.rot_x = rx;
  _transformSum.rot_y = ry;
  _transformSum.rot_z = rz;
  _transformSum.pos = trans;

  transformToEnd(_cornerPointsLessSharp);
  transformToEnd(_surfPointsLessFlat);

  _cornerPointsLessSharp.swap(_lastCornerCloud);
  _surfPointsLessFlat.swap(_lastSurfaceCloud);

  lastCornerCloudSize = _lastCornerCloud->points.size();
  lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100)
  {
    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
  }

}



} // end namespace loam
