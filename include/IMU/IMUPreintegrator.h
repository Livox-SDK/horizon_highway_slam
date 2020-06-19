#ifndef TVISLAM_IMUPREINTEGRATOR_H
#define TVISLAM_IMUPREINTEGRATOR_H

#include <Eigen/Dense>

#include "IMU/imudata.h"
#include "so3.h"

using namespace Eigen;
using namespace Sophus;

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

class IMUPreintegrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUPreintegrator();
    IMUPreintegrator(const IMUPreintegrator& pre);

    void reset();

    void update(const Vector3d& omega, const Vector3d& acc, double& dt);

    inline Eigen::Vector3d getDeltaP() const
    {
        return _delta_P;
    }
        inline Eigen::Vector3d getDeltaV() const
    {
        return _delta_V;
    }
        inline Eigen::Matrix3d getDeltaR() const
    {
        return _delta_R;
    }

    inline Eigen::Matrix3d getJPBiasg() const
    {
        return _J_P_Biasg;
    }
    inline Eigen::Matrix3d getJPBiasa() const
    {
        return _J_P_Biasa;
    }
    inline Eigen::Matrix3d getJVBiasg() const
    {
        return _J_V_Biasg;
    }
    inline Eigen::Matrix3d getJVBiasa() const
    {
        return _J_V_Biasa;
    }
    inline Eigen::Matrix3d getJRBiasg() const
    {
        return _J_R_Biasg;
    }

    inline Matrix9d getCovPVPhi() const 
    {
        return _cov_P_V_Phi;
    }

    inline double getDeltaTime() const {
        return _delta_time;
    }

    static Matrix3d skew(const Vector3d& v)
    {
        return SO3::hat( v );
    }

    static Matrix3d Expmap(const Vector3d& v)
    {
        return SO3::exp(v).matrix();
    }

    static Matrix3d JacobianR(const Vector3d& w)
    {
        Matrix3d Jr = Matrix3d::Identity();
        double theta = w.norm();
        if(theta<0.00001)
        {
            return Jr;
        }
        else
        {
            Vector3d k = w.normalized();
            Matrix3d K = skew(k);
            Jr =   Matrix3d::Identity()
                    - (1-cos(theta))/theta*K
                    + (1-sin(theta)/theta)*K*K;
        }
        return Jr;
    }
    static Matrix3d JacobianRInv(const Vector3d& w)
    {
        Matrix3d Jrinv = Matrix3d::Identity();
        double theta = w.norm();

        // very small angle
        if(theta < 0.00001)
        {
            return Jrinv;
        }
        else
        {
            Vector3d k = w.normalized();
            Matrix3d K = SO3::hat(k);
            Jrinv = Matrix3d::Identity()
                    + 0.5*SO3::hat(w)
                    + ( 1.0 - (1.0+cos(theta))*theta / (2.0*sin(theta)) ) *K*K;
        }

        return Jrinv;
    }

    static Matrix3d JacobianL(const Vector3d& w)
    {
        return JacobianR(-w);
    }

    static Matrix3d JacobianLInv(const Vector3d& w)
    {
        return JacobianRInv(-w);
    }


    inline Quaterniond normalizeRotationQ(const Quaterniond& r)
    {
        Quaterniond _r(r);
        if (_r.w()<0)
        {
            _r.coeffs() *= -1;
        }
        return _r.normalized();
    }

    inline Matrix3d normalizeRotationM (const Matrix3d& R)
    {
        Quaterniond qr(R);
        return normalizeRotationQ(qr).toRotationMatrix();
    }
private:
    Eigen::Vector3d _delta_P;
    Eigen::Vector3d _delta_V;
    Eigen::Matrix3d _delta_R;

    Eigen::Matrix3d _J_P_Biasg;
    Eigen::Matrix3d _J_P_Biasa;
    Eigen::Matrix3d _J_V_Biasg;
    Eigen::Matrix3d _J_V_Biasa;
    Eigen::Matrix3d _J_R_Biasg;

    Matrix9d _cov_P_V_Phi;

    double _delta_time;

};

#endif // TVISLAM_IMUPREINTEGRATOR_H
