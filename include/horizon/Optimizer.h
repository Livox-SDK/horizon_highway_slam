#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "horizon/Frame.h"
#include "IMU/g2otypes.h"
#include "g2o/g2o/types/types_seven_dof_expmap.h"
#include "g2o/g2o/core/sparse_optimizer.h"
#include "g2o/g2o/core/block_solver.h"
#include "g2o/g2o/solvers/linear_solver_eigen.h"
#include "g2o/g2o/solvers/linear_solver_cholmod.h"
#include "g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/g2o/core/robust_kernel_impl.h"
#include "IMU/configparam.h"
#define TEST_REINIT false
using namespace std;
class Optimizer
{
    typedef pcl::PointXYZINormal PointType;

public:
    static uint32_t __float_as_int(float f){
      union{uint32_t i; float f;} conv{};
      conv.f = f;
      return conv.i;
    }

    static float __int_as_float(uint32_t i){
      union{float f; uint32_t i;} conv{};
      conv.i = i;
      return conv.f;
    }
    static bool Initialization(CircularBuffer<Frame::Ptr> &all_laser_transforms,
                               Vector3d &g,
                               Matrix4d &transform_lb,
                               Matrix3d &R_WI);
    static void EstimateGyroBias(CircularBuffer<Frame::Ptr> &all_laser_transforms);
    static bool ApproximateGravity(CircularBuffer<Frame::Ptr> &all_laser_transforms, Vector3d &g, Matrix4d &transform_lb);
    static void RefineGravityAccBias(CircularBuffer<Frame::Ptr> &all_laser_transforms,
                                     Vector3d &g_approx,
                                     Matrix4d &transform_lb,
                                     Matrix3d &R_WI);
    static MatrixXd TangentBasis(Vector3d &g0);
    static void InitGBANavStatePVR(CircularBuffer<Frame::Ptr> &FramePtrBuffer, const Eigen::Vector3d &gw);
    static void RefreshGravityVec(CircularBuffer<Frame::Ptr> &FramePtrBuffer, const Eigen::Vector3d &gw,
                                      int nIterations, bool bRobust, bool bExtrinsic);

    static bool TrackLocalMap(LocalMaper& localmaper, CircularBuffer<Frame::Ptr> &FramePtrBuffer,
                              CircularBuffer<Frame::Ptr> &ResetPtrBuffer, std::vector<IMUData>& vimuDataBuffer,
                              const Eigen::Vector3d &gw, int nIterations, bool bRobust, bool bExtrinsic);

    template<typename PointT , typename Scalar > static void
    TransformPointCloud(const Frame::Ptr& frame,
                        const std::vector<int>& cloudptr_in,
                        pcl::PointCloud<PointT>& cloudptr_out,
                        const Eigen::Matrix<Scalar,3,3>& _R, const Eigen::Matrix<Scalar,3,1>& _t,
                        int _intensity = -1){
      size_t num = cloudptr_in.size();
      cloudptr_out.points.resize(num);
      Eigen::Matrix<Scalar,3,1> eigP;
      for(size_t i=0; i<num; ++i){
        eigP.x() = frame->_cloud->points[cloudptr_in[i]].x;
        eigP.y() = frame->_cloud->points[cloudptr_in[i]].y;
        eigP.z() = frame->_cloud->points[cloudptr_in[i]].z;
        eigP = (_R * eigP +_t).eval();
        cloudptr_out.points[i].x = eigP.x();
        cloudptr_out.points[i].y = eigP.y();
        cloudptr_out.points[i].z = eigP.z();
        cloudptr_out.points[i].normal_x = frame->_cloud->points[cloudptr_in[i]].normal_x;
        cloudptr_out.points[i].normal_y = frame->_cloud->points[cloudptr_in[i]].normal_y;
        cloudptr_out.points[i].normal_z = frame->_cloud->points[cloudptr_in[i]].normal_z;
        if(_intensity >= 0){
          cloudptr_out.points[i].intensity = _intensity;
        }else{
          cloudptr_out.points[i].intensity = frame->_cloud->points[cloudptr_in[i]].intensity;
        }
      }
    }

    static void AddSharpEdgeWithLastFrame(const Frame::Ptr& frameptr,
                                          std::vector<g2o::EdgeIMUNavStatePVRtoLine*>& edges,
                                          uint start_idx = 0);

    static void AddFlatEdgeWithLastFrame(const Frame::Ptr& frameptr,
                                         std::vector<g2o::EdgeIMUNavStatePVRtoPlan*>& edges,
                                         uint start_idx = 0);

};


#endif // OPTIMIZER_H
