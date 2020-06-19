#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <IMU/imudata.h>
#include <IMU/NavState.h>
#include <IMU/IMUPreintegrator.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <horizon/CircularBuffer.h>
#include "IMU/math_utils.h"
#include <queue>
#include <thread>
#include <memory>
#include <condition_variable>
#include <chrono>
using namespace std;

class Frame
{
    typedef pcl::PointXYZINormal PointType;
    typedef pcl::PointCloud<PointType> PointCloudT;

public:
    typedef boost::shared_ptr<Frame> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame(PointCloudT::Ptr  cloud, std::vector<int>&  sharp,
          std::vector<int>&  lessSharp, std::vector<int>&  flat, std::vector<int>&  lessFlat,
          const double& timeStamp, std::chrono::steady_clock::time_point& receiveTime,
          Frame::Ptr  lastptr = nullptr);
    Frame();
    ~Frame()= default;
    template<typename T> void toEulerAngle(Quaternion<T>& q, T& roll, T& pitch, T& yaw){
      T sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
      T cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
      roll = atan2(sinr_cosp, cosr_cosp);

      T sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
      if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
      else
        pitch = asin(sinp);

      T siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
      T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
      yaw = atan2(siny_cosp, cosy_cosp);
    }
    void ComputeIMUPreIntSinceLastFrame();
    void UpdatePoseFromNS(const Eigen::Matrix4d &Tbl);
    void SetInitialNavStateAndBias(const NavState& ns);
    void UpdateNavState(const IMUPreintegrator& imupreint, const Vector3d& gw);

    void SetNavState(const NavState& ns);
    void SetNavStateBiasGyr(const Vector3d &bg);
    void SetNavStateBiasAcc(const Vector3d &ba);
    void SetNavStateVel(const Vector3d &vel);
    void SetNavStatePos(const Vector3d &pos);
    void SetNavStateRot(const Matrix3d &rot);
    void SetNavStateRot(const Sophus::SO3 &rot);
    void SetNavStateDeltaBg(const Vector3d &dbg);
    void SetNavStateDeltaBa(const Vector3d &dba);
    void SetPose(Eigen::Matrix4d& Tlw);
    void SetPoseInverse(Eigen::Matrix4d& Twl);
    void SetPoseInverseBeforeMapped(Eigen::Matrix4d& Twl);
    void SetPoseInverse(Eigen::Matrix3d& Rwl, Eigen::Vector3d& twl);
    void UpdatePoseMatrices();
    inline Eigen::Vector3d GetPositionInverse(){ return mOw; }
    inline Eigen::Matrix3d GetRotationInverse(){ return mRwl; }
    inline Eigen::Vector3d GetPosition(){ return mtlw; }
    inline Eigen::Matrix3d GetRotation(){ return mRlw; }

    const Eigen::Matrix4d& GetPose(){ return mTlw; }
    const Eigen::Matrix4d& GetPoseInverse(){ return mTwl; }
    const NavState& GetNavState() const;
    const Eigen::Matrix4d& GetPoseInverseBeforeMapped() const { return mTwl_BeforeMapped; };
protected:
    std::mutex mMutexNavState;
    NavState mNavState;
public:
    const PointCloudT::Ptr _cloud;
    const std::vector<int> _sharp;
    const std::vector<int> _lessSharp;
    const std::vector<int> _flat;
    const std::vector<int> _lessFlat;
    const double _timeStamp;
    Frame::Ptr _lastFrame;
    std::vector<IMUData> mvIMUDataSinceLastFrame;
    IMUPreintegrator mIMUPreInt;
    NavState mNavStateForDistortion;
    Matrix<double,15,15> mMargCovInv;
    NavState mNavStatePrior;
    bool InResetBuffer = false;
    bool IsReseted = false;
    std::chrono::steady_clock::time_point _theReceiveTime;
private:
    constexpr static const float ot = 0.1;
    std::mutex mMutexPose;
    std::mutex mMutexIMUData;
    Eigen::Matrix4d mTlw;
    Eigen::Matrix4d mTwl;
    Eigen::Matrix3d mRlw;
    Eigen::Vector3d mtlw;
    Eigen::Matrix3d mRwl;
    Eigen::Vector3d mOw;
    Eigen::Matrix4d mTwl_BeforeMapped = Eigen::Matrix4d::Identity();
};


#endif // FRAME_H
