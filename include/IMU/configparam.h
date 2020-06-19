#ifndef CONFIGPARAM_H
#define CONFIGPARAM_H

#include <Eigen/Dense>
#include <ctime>
#include <ros/ros.h>

class ConfigParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ConfigParam(ros::NodeHandle& nodeHandler);
    explicit ConfigParam(const int& _ifIMU=0, const int& _processMap=1,
                         const int& _initWidth=100, const int& _undistortion=1,
                         const Eigen::Matrix4d& _ExtrinsicT_lb = Eigen::Matrix4d::Identity());
    ~ConfigParam() = default;
    static Eigen::Matrix4d GetEigTbl();
    static Eigen::Matrix4d GetEigT_lb();

    static double GetG(){return _g;}
    static void SetG(double new_g){ _g = new_g;}

    static void SetEigT_lb(const Eigen::Matrix4d& T_lb);
    static void SetEigT_lb(const Eigen::Matrix3d& R_lb);
    static void SetEigTbl(const Eigen::Matrix4d& T_bl);
    static void RequestSystemShutDown(){SYSTEM_SHUT_DOWN = true;}
    static bool GetSystemStatus(){return SYSTEM_SHUT_DOWN;}

    static double acc_n;
    static double gyr_n;
    static double acc_w;
    static double gyr_w;
    static bool _Multi9p8;
    static bool _exRblCalibration;
    static bool _exPblCalibration;
    static Eigen::Vector3d _GravityVec;
    static Eigen::Matrix3d _RwiInit;
    static int ifIMU;
    static int MapDownSize;
    static int SmoothWindowSize;
    static std::string SaveMapPath;
    static int LidarToMapping;
    static int AccumulateCount;
    static int LocalMapWidth;
    static int GlobalMapWidth;
    static int SlideWindowWidth;
    static int OpenMapping;
    static float TimeOffset;
    static int _doMap;
    static bool GravityInOpt;
    static float TrustLidar;
    static Eigen::Matrix3d GravityMargCovInv;
    static Eigen::Vector3d gw_init;
    static double TheFirstFrame;
    static bool CallForRestart;
    static int undistortion;

    template<typename T> static void toEulerAngle(Eigen::Quaternion<T>& q, T& roll, T& pitch, T& yaw){
      // roll (x-axis rotation)
      T sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
      T cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
      roll = atan2(sinr_cosp, cosr_cosp);

      // pitch (y-axis rotation)
      T sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
      if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
        pitch = asin(sinp);

      // yaw (z-axis rotation)
      T siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
      T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
      yaw = atan2(siny_cosp, cosy_cosp);
    }

    static void Delay_N_ms(uint time)
    {
      time *= 1000;
      clock_t now = clock();
      while( clock() - now < time );
    }
private:
    static Eigen::Matrix4d _EigTbl;
    static Eigen::Matrix4d _EigTlb;
    static double _g;
    static bool SYSTEM_SHUT_DOWN;
};

#endif // CONFIGPARAM_H
