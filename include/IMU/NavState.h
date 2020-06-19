#ifndef NAVSTATE_H
#define NAVSTATE_H

#include "Eigen/Geometry"
#include "so3.h"
#include "IMU/IMUPreintegrator.h"

using namespace Eigen;
typedef Matrix<double, 15, 1> Vector15d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;

class NavState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    NavState();
    NavState(const NavState& _ns);
    ~NavState() = default;

    Sophus::SO3 Get_R() const{return _R;}
    Matrix3d Get_RotMatrix() const{return _R.matrix();}
    Vector3d Get_P() const{return _P;}
    Vector3d Get_V() const{return _V;}
    void Set_Pos(const Vector3d &pos){_P = pos;}
    void Set_Vel(const Vector3d &vel){_V = vel;}
    void Set_Rot(const Matrix3d &rot){_R = Sophus::SO3(rot);}
    void Set_Rot(const Sophus::SO3 &rot){_R = rot;}

    Vector3d Get_BiasGyr() const{return _BiasGyr;}
    Vector3d Get_BiasAcc() const{return _BiasAcc;}
    void Set_BiasGyr(const Vector3d &bg){_BiasGyr = bg;}
    void Set_BiasAcc(const Vector3d &ba){_BiasAcc = ba;}

    Vector3d Get_dBias_Gyr() const{return _dBias_g;}
    Vector3d Get_dBias_Acc() const{return _dBias_a;}
    void Set_DeltaBiasGyr(const Vector3d &dbg){_dBias_g = dbg;}
    void Set_DeltaBiasAcc(const Vector3d &dba){_dBias_a = dba;}

    void IncSmall(Vector15d delta);
    void IncSmallPVR(Vector9d dPVR);
    void IncSmallPR(Vector6d dPR);
    void IncSmallV(Vector3d dV);
    void IncSmallBias(Vector6d dBias);

    static void updateNS(NavState& ns, const IMUPreintegrator& imupreint, const Vector3d& gw);

private:
    Vector3d _P;
    Vector3d _V;
    Sophus::SO3 _R;

    Vector3d _BiasGyr;
    Vector3d _BiasAcc;


    Vector3d _dBias_g;
    Vector3d _dBias_a;
};

#endif // NAVSTATE_H
