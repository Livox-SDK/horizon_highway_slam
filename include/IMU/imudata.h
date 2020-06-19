#ifndef IMUDATA_H
#define IMUDATA_H

#include <Eigen/Dense>
#include "IMU/configparam.h"

using namespace Eigen;

class IMUData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static Matrix3d _gyrMeasCov;
    static Matrix3d _accMeasCov;
    static Matrix3d getGyrMeasCov(void) {return _gyrMeasCov;}
    static Matrix3d getAccMeasCov(void) {return _accMeasCov;}

    static Matrix3d _gyrBiasRWCov;
    static Matrix3d _accBiasRWCov;
    static Matrix3d getGyrBiasRWCov(void) {return _gyrBiasRWCov;}
    static Matrix3d getAccBiasRWCov(void) {return _accBiasRWCov;}

    static double _gyrBiasRw2;
    static double _accBiasRw2;
    static double getGyrBiasRW2(void) {return _gyrBiasRw2;}
    static double getAccBiasRW2(void) {return _accBiasRw2;}


    IMUData(const double& gx, const double& gy, const double& gz,
            const double& ax, const double& ay, const double& az,
            const double& t);
    ~IMUData() = default;
    static void ResetCovarianceMatrix() {
      _gyrBiasRw2 = ConfigParam::gyr_w * ConfigParam::gyr_w;
      _accBiasRw2 = ConfigParam::acc_w * ConfigParam::acc_w;
      _gyrBiasRWCov = Matrix3d::Identity() * _gyrBiasRw2;
      _accBiasRWCov = Matrix3d::Identity() * _accBiasRw2;
      _gyrMeasCov = Matrix3d::Identity() * ConfigParam::gyr_n * ConfigParam::gyr_n;
      _accMeasCov = Matrix3d::Identity() * ConfigParam::acc_n * ConfigParam::acc_n;
    }

    Vector3d _g;
    Vector3d _a;
    double _t;
};

#endif // IMUDATA_H
