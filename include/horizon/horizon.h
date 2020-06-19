#ifndef HORIZON_HORIZON_H
#define HORIZON_HORIZON_H
#include <cmath>
#include <vector>
#include <list>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <chrono>
#include <memory>
#include "IMU/g2otypes.h"
#include "g2o/g2o/core/sparse_optimizer.h"
#include "g2o/g2o/core/block_solver.h"
#include "g2o/g2o/solvers/linear_solver_eigen.h"
#include "g2o/g2o/solvers/linear_solver_cholmod.h"
#include "g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/g2o/core/robust_kernel_impl.h"
#include <horizon/Frame.h>
#include "IMU/MsgSynchronizer.h"
#include <IMU/configparam.h>
#include <horizon/Optimizer.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <horizon/lidarFrame.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <newmsg/X1Msg.h>
#include <newmsg/X1Point.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <future>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "IMU/math_utils.h"
#include <utility>

using std::sin;
using std::cos;
using std::atan2;
typedef pcl::PointXYZINormal PointType;

inline uint32_t __float_as_int(float f){
  union{uint32_t i; float f;} conv{};
  conv.f = f;
  return conv.i;
}

inline float __int_as_float(uint32_t i){
  union{float f; uint32_t i;} conv{};
  conv.i = i;
  return conv.f;
}
struct PointWithCurv{
    PointType *point_ptr{};
    int idx;
    float curvature;
    float angle;
    bool deprecated;
    float depth{};
    float depth_last{};
    float depth_next{};
    explicit PointWithCurv(float a=0, bool b=false, float c=0)
            :curvature(a),deprecated(b),angle(c),idx(0){}
    ~PointWithCurv()= default;
};

class ODOMETRY{
public:
    explicit ODOMETRY(ros::NodeHandle& _nh);
    ~ODOMETRY()= default;
    void inputFeatures(const Frame::Ptr& ptr, const livox_ros_driver::CustomMsgConstPtr& msg);
    void inputFeatures(const Frame::Ptr& ptr, const newmsg::X1MsgConstPtr& msg);
    void TransformToStart(PointType const * pi, PointType * po);
    void TransformCloudToEnd(pcl::PointCloud<PointType>::Ptr& cloud);
    void estimatOdom();
    static bool findLineMatch(pcl::KdTreeFLANN<PointType>::Ptr& cornerLast, pcl::PointCloud<PointType>::Ptr& cloudCornerLast,
                       PointType& sel, int& closestPointInd, int& minPointInd2);
    static bool findFlatMatch(pcl::KdTreeFLANN<PointType>::Ptr& surfLast, pcl::PointCloud<PointType>::Ptr& cloudSurfLast,
                       PointType& sel, int& closestPointInd, int& minPointInd2, int& minPointInd3);
    void setExtrinsicStatus(bool status){ ExtrinsicEstimated = status;}
    bool getExtrinsicStatus() const{ return ExtrinsicEstimated;}
    void setWaitTryInit(bool status){ WaitTryInit = status;}
    void transformUpdate(const std_msgs::Float32MultiArrayConstPtr& msg);
public:
    ros::NodeHandle& nh_odom;
    ros::Publisher pubUndistortCloud;
    ros::Publisher pubLidarFrame;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserOdometryPath;
    ros::Publisher pubEgoMotion;
    ros::Publisher pubLaserCloud;
    nav_msgs::Odometry laserOdometry;
    nav_msgs::Path laserOdoPath;
    tf::StampedTransform laserOdometryTrans;
    tf::TransformBroadcaster tfBroadcaster;
    LocalMaper localmapper{(size_t)ConfigParam::LocalMapWidth};
    ImuMsgFetcher* msgFetcher{};
    CircularBuffer<Frame::Ptr> FramePtrBuffer{(size_t)ConfigParam::GlobalMapWidth};
    CircularBuffer<Frame::Ptr> ResetPtrBuffer{15};
private:
    geometry_msgs::Twist twi;
    int optimizeIterations = 0;
    bool TryRunInitialization();
    std::list<float> odomWindow[6];
    float lastSum[6] = {0};
    void highFrameRateOdom(Frame::Ptr& odomframeptr);
    std::mutex _mutexTransform;
    void transformAssociateToMap(Eigen::Matrix4d& _transformSum, Eigen::Matrix4d& _transformTobeMapped);
    Eigen::Matrix4d transformBefMapped;
    Eigen::Matrix4d transformAftMapped;
    bool allowUpdateTrans = true;
    bool volatile ExtrinsicEstimated;
    bool volatile WaitTryInit;
    std::mutex _frameQueue;
    std::condition_variable _queue_not_empty;
    std::queue<livox_ros_driver::CustomMsgConstPtr> originMsg_q;
    std::queue<newmsg::X1MsgConstPtr> originX1Msg_q;
    livox_ros_driver::CustomMsgConstPtr originMsg;
    newmsg::X1MsgConstPtr originX1Msg;
    std::queue<Frame::Ptr> frames_q;
    uint volatile newFramesInfo = 0;
    void addPointToLineEdge(std::vector<g2o::EdgeNavStatePVRtoLine*>& edges, int cornerPointsSharpNum);
    void addPointToPlanEdge(std::vector<g2o::EdgeNavStatePVRtoPlan*>& edges, int cornerPointsSharpNum, int surfPointsFlatNum);
    float transform_2d[3] = {0};
    bool volatile newOdometry = false;
    pcl::PointCloud<PointType> newLessSurfCloud;
    double newTimeLessSurf{};
    const int skipFrameNum = 1;
    bool systemInited = false;
    double timeCornerPointsSharp = 0;
    double timeCornerPointsLessSharp = 0;
    double timeSurfPointsFlat = 0;
    double timeSurfPointsLessFlat = 0;
    double timeLaserCloudFullRes = 0;
    double timeImuTrans = 0;
    bool newCornerPointsSharp = false;
    bool newCornerPointsLessSharp = false;
    bool newSurfPointsFlat = false;
    bool newSurfPointsLessFlat = false;
    bool newLaserCloudFullRes = false;
    bool newImuTrans = false;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;
    int laserCloudCornerLastNum{};
    int laserCloudSurfLastNum{};
    int pointSelCornerInd[40000]{};
    int pointSearchCornerInd1[40000]{};
    int pointSearchCornerInd2[40000]{};
    int pointSelSurfInd[40000]{};
    int pointSearchSurfInd1[40000]{};
    int pointSearchSurfInd2[40000]{};
    int pointSearchSurfInd3[40000]{};
    float transform[6] = {0};
    float transform_tmp[6] = {0};
    float transformSum[6] = {0};
    float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
    float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
    float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
    float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    PointType pointSel, coeff;
    bool isDegenerate = false;
    bool need2DICP = false;
    int frameCount;
    NavState NavStateLast;
    NavState NavStateCurr;
    Eigen::Matrix3d delta_Rcl;
    Eigen::Vector3d delta_tcl;
};

class MAPPING_SWIFT{
public:
    explicit MAPPING_SWIFT(ros::NodeHandle& _nh);
    ~MAPPING_SWIFT()= default;
    void transformAssociateToMap(Eigen::Matrix4d& _transformSum, Eigen::Matrix4d& _transformTobeMapped);
    void transformUpdate(const Eigen::Matrix4d& _transformSum, const Eigen::Matrix4d& _transformTobeMapped);
    static void pointAssociateToMap(PointType const * pi, PointType * po, Eigen::Matrix4d& _transformTobeMapped);
    void estimateMap(const horizon_slam::lidarFrame::ConstPtr& msg);
    size_t ToIndex(int i, int j, int k) const {
      return i + laserCloudDepth * j + laserCloudDepth * laserCloudWidth * k;
    }

private:
    static pcl::RGB getRGB(float& intensity);
    static void cloudTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Matrix4d& surroundCloudPose);
    void displaySurroundMap(pcl::PointCloud<PointType>::Ptr surroundCloud, Eigen::Matrix4d surroundCloudPose, double surroundCloudTime);
    std::list<std::thread*> thread_pool;
    ros::NodeHandle& nh_mapping;
    std::mutex _mutexPublish;
    ros::Publisher pubTransUpdate;
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserCloudFarAway;
    ros::Subscriber subLidarFrame;
    bool volatile AllowUpdateTransform = true;
    pcl::PointCloud<PointType>::Ptr A_BufferCloud;
    pcl::PointCloud<PointType>::Ptr pointOrisLine;
    pcl::PointCloud<PointType>::Ptr coeffsLine;
    pcl::PointCloud<PointType>::Ptr pointOrisPlan;
    pcl::PointCloud<PointType>::Ptr coeffsPlan;
    bool volatile updateTransform;
    std::mutex mtx;
    void processPointToLine(pcl::PointCloud<PointType>::Ptr& pointOris, pcl::PointCloud<PointType>::Ptr& coeffs, Eigen::Matrix4d& _transformTobeMapped);
    void processPointToPlan(pcl::PointCloud<PointType>::Ptr& pointOris, pcl::PointCloud<PointType>::Ptr& coeffs, Eigen::Matrix4d& _transformTobeMapped);
    uint volatile newOdomInfo = 0;
    NavState NavStateCurrToMap;
    NavState NavStateOrigin;
    std::vector<g2o::EdgeNavStatePVRtoLine*> vEdgePVRtoLine;
    std::vector<g2o::EdgeNavStatePVRtoPlan*> vEdgePVRtoPlan;
    float transform_2d[3] = {0};
    bool need2DICP = false;
    const double scanPeriod = 0.1;
    const int stackFrameNum = 2;
    const int mapFrameNum = 5;
    double timeLaserCloudCornerLast = 0;
    double timeLaserCloudSurfLast = 0;
    double timeLaserCloudFullRes = 0;
    bool newLaserCloudCornerLast = false;
    bool newLaserCloudSurfLast = false;
    bool newLaserCloudFullRes = false;
    bool newLaserOdometry = false;
    int laserCloudCenWidth = 10;
    int laserCloudCenHeight = 5;
    int laserCloudCenDepth = 10;
    static const int laserCloudWidth = 21;
    static const int laserCloudHeight = 11;
    static const int laserCloudDepth = 21;
    static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//4851
    int laserCloudValidInd[125]{};
    int laserCloudSurroundInd[125]{};
    pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;
    pcl::PointCloud<PointType>::Ptr laserCloudSurround;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
    pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
    float transformIncre[6] = {0};

    Eigen::Matrix4d transformBefMapped;
    Eigen::Matrix4d transformAftMapped;

    int imuPointerFront = 0;
    int imuPointerLast = -1;
    static const int imuQueLength = 200;

    double imuTime[imuQueLength] = {0};
    float imuRoll[imuQueLength] = {0};
    float imuPitch[imuQueLength] = {0};
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    PointType pointOri, pointSel, pointProj, coeff;
    Eigen::Matrix<double,5,3> matA0;
    Eigen::Matrix<double,5,1> matB0;
    Eigen::Matrix<double,3,1> matX0;
    Eigen::Matrix<double,3,3> matA1;
    Eigen::Matrix<double,1,3> matD1;
    Eigen::Matrix<double,3,3> matV1;
    bool isDegenerate = false;
    Eigen::Matrix<double,6,6> matP;
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    int frameCount = stackFrameNum - 1;   //0
    int mapFrameCount = mapFrameNum - 1;  //4
};

class ROSINPUT{
public:
    explicit ROSINPUT(ros::NodeHandle& _nh);
    ~ROSINPUT()= default;
    void detectFeaturePoint(const livox_ros_driver::CustomMsgConstPtr& msg, pcl::PointCloud<PointType>::Ptr& cloud,
                            std::vector<int>& pointsSharp, std::vector<int>& pointsLessSharp,
                            std::vector<int>& pointsFlat, std::vector<int>& pointsLessFlat, int& countMsg);
    void detectFeaturePoint(const newmsg::X1MsgConstPtr & msg, pcl::PointCloud<PointType>::Ptr& cloud,
                            std::vector<int>& pointsSharp, std::vector<int>& pointsLessSharp,
                            std::vector<int>& pointsFlat, std::vector<int>& pointsLessFlat, int& countMsg);
    void customHandler();
    void customHandlerX1();
    void imuCallBack(const sensor_msgs::ImuConstPtr& msg);
    void lidarCallBack(const livox_ros_driver::CustomMsgConstPtr &msg);
    void lidarCallBack(const newmsg::X1MsgConstPtr &msg);
    void processThisLine(int s, std::vector<std::vector<PointWithCurv>>& laserCloudScans,
                         std::vector< std::vector<int> >& CurvIdx, std::vector<int>& countLine_tmp,
                         std::vector<std::vector<int>>& valid_countLine_tmp,
                         std::vector<int>& pointsSharp, std::vector<int>& pointsLessSharp,
                         std::vector<int>& pointsFlat, std::vector<int>& pointsLessFlat);
    static void shutdown();
    void saveMapCloud(){}
private:
    ODOMETRY* OdometryPtr;
    std::thread* thr0_customhandler;
    std::thread* thr1_odometry;
    pcl::PointCloud<PointType>::Ptr cornerPointsSharpForPub_c;
    pcl::PointCloud<PointType>::Ptr surfPointsFlatForPub_c;
    bool SuccessAllocate;
    ros::NodeHandle& nh;
    uint32_t TimeBase_F = 0;
    uint32_t TimeEnd_E = 0;
    ros::Subscriber customCloud;
    ros::Subscriber IMUsuber;
    ros::Publisher pubB_BOXs;
    ros::Subscriber subTransUpdate;
    ImuMsgFetcher msgFetcher;

    pcl::PointCloud<PointType>::Ptr laserCloud;
    std::vector<int> cornerPointsSharp;
    std::vector<int> cornerPointsLessSharp;
    std::vector<int> surfPointsFlat;
    std::vector<int> surfPointsLessFlat;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeFilter;
    std::vector<int> countLine;
    const uint8_t CONTROL_CURVATURE = 3;
    const int EQUAL_PARTS_NUM = 100;
    const int systemDelay = 0;
    const int accumNum = 2;
    const float ANGLE_RESOLUTION = 0.007;
    const int N_SCANS = 6;
    const float LessFlatDist = 0.2;
    int systemInitCount = 0;
    bool systemInited = false;
    visualization_msgs::Marker line_list;
    const int whichLidar;
    uint32_t cloudIdx = 0;
};

#endif
