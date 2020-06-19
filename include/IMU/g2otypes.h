#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "g2o/g2o/core/base_vertex.h"
#include "g2o/g2o/core/base_unary_edge.h"
#include "so3.h"
#include "NavState.h"
#include "IMUPreintegrator.h"
#include "g2o/g2o/core/base_multi_edge.h"
#include "g2o/g2o/core/base_binary_edge.h"
#include "g2o/g2o/types/types_six_dof_expmap.h"

namespace g2o
{
class VertexGravityVec : public BaseVertex<3, Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexGravityVec() : BaseVertex<3, Vector3d>() {}

    bool read(std::istream& is) override {return true;}

    bool write(std::ostream& os) const override {return true;}

    void setToOriginImpl() override {
      _estimate.setZero();
    }

    void oplusImpl(const double* update_) override  {
      Eigen::Map<const Vector3d> update(update_);
      _estimate += update;
    }
};

class VertexNavStatePVR : public BaseVertex<9, NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexNavStatePVR() : BaseVertex<9, NavState>(){}
    ~VertexNavStatePVR() override= default;

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl() {
      _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector9d> update(update_);
        _estimate.IncSmallPVR(update);
    }

};

class VertexNavStateBias : public BaseVertex<6, NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexNavStateBias() : BaseVertex<6, NavState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl() {
      _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector6d> update(update_);
        _estimate.IncSmallBias(update);
    }

};

class EdgeNavStatePVRtoLine : public BaseBinaryEdge<1, double, VertexNavStatePVR, VertexNavStatePVR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeNavStatePVRtoLine(const Vector3d& _p, const Vector3d& _vtx1, const Vector3d& _vtx2, double _s = 1):
            point(_p), vtx1(_vtx1), vtx2(_vtx2), s(_s){
      l12 = std::sqrt((vtx1(0) - vtx2(0))*(vtx1(0) - vtx2(0)) + (vtx1(1) - vtx2(1))*(vtx1(1) - vtx2(1)) + (vtx1(2) - vtx2(2))*(vtx1(2) - vtx2(2)));
    }
    ~EdgeNavStatePVRtoLine() override = default;

    bool read(std::istream& is) override{return true;}

    bool write(std::ostream& os) const override{return true;}

    void computeError() override;

    double computeError(const NavState& nslast, const NavState& nscurr);

    void linearizeOplus() override;

    void SetParams(const Vector3d& _p, const Vector3d& _vtx1, const Vector3d& _vtx2, double _s = 1){
      point = _p;
      vtx1 = _vtx1;
      vtx2 = _vtx2;
      s = _s;
      l12 = std::sqrt((vtx1(0) - vtx2(0))*(vtx1(0) - vtx2(0)) + (vtx1(1) - vtx2(1))*(vtx1(1) - vtx2(1)) + (vtx1(2) - vtx2(2))*(vtx1(2) - vtx2(2)));
    }

protected:
    double s;
    Vector3d point;
    Vector3d vtx1;
    Vector3d vtx2;
    double l12;
    double a012 = 0;
    Vector3d P_to_start;
    Vector3d Pcl;
    Quaterniond qlc;
    Quaterniond delta_qlc;
    float _weight = 1;
};

class EdgeIMUNavStatePVRtoLine : public BaseBinaryEdge<1, double, VertexNavStatePVR, VertexNavStatePVR>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeIMUNavStatePVRtoLine(Vector3d  _p, Vector3d  _vtx1, Vector3d  _vtx2, double _s = 1):
          point(std::move(_p)), vtx1(std::move(_vtx1)), vtx2(std::move(_vtx2)), s(_s){
    l12 = std::sqrt((vtx1(0) - vtx2(0))*(vtx1(0) - vtx2(0)) + (vtx1(1) - vtx2(1))*(vtx1(1) - vtx2(1)) + (vtx1(2) - vtx2(2))*(vtx1(2) - vtx2(2)));
  }

  bool read(std::istream& is) override{return true;}

  bool write(std::ostream& os) const override{return true;}

  void computeError() override;

  double computeError(const NavState& nslast, const NavState& nscurr);

  void linearizeOplus() override;

  void SetParams(const Matrix3d& _Rbl, const Vector3d& _Pbl){
    Rbl = _Rbl;
    Pbl = _Pbl;
    Rlb = _Rbl.transpose();
    Plb = -Rlb * _Pbl;
    qbl = Quaterniond(Rbl).normalized();
    qlb = Quaterniond(Rlb).normalized();
  }
  void SetParams(const Matrix4d& _Tbl){
    Rbl = _Tbl.topLeftCorner(3,3);
    Pbl = _Tbl.topRightCorner(3,1);
    Rlb = Rbl.transpose();
    Plb = -Rlb * Pbl;
    qbl = Quaterniond(Rbl).normalized();
    qlb = Quaterniond(Rlb).normalized();
  }

  const float& get_Weight(){return _weight;}

protected:
  double s;
  Vector3d point;
  Vector3d vtx1;
  Vector3d vtx2;
  double l12;
  Matrix3d Rbl, Rlb;
  Quaterniond qbl, qlb;
  Vector3d Pbl, Plb;
  float _weight = 1;

  Quaterniond q_half;
  Quaterniond qlc;
  Vector3d Pcl;
  Vector3d P_to_start;
  double a012 = 0;

};

class EdgeNavStatePVRtoPlan : public BaseBinaryEdge<1, double, VertexNavStatePVR, VertexNavStatePVR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeNavStatePVRtoPlan(const Vector3d& _p, const Vector3d& _vtx1, const Vector3d& _vtx2, const Vector3d& _vtx3, double _s = 1):
            point(_p), vtx1(_vtx1), vtx2(_vtx2), vtx3(_vtx3), s(_s){
      pa = (vtx2(1) - vtx1(1)) * (vtx3(2) - vtx1(2))
         - (vtx3(1) - vtx1(1)) * (vtx2(2) - vtx1(2));
      pb = (vtx2(2) - vtx1(2)) * (vtx3(0) - vtx1(0))
         - (vtx3(2) - vtx1(2)) * (vtx2(0) - vtx1(0));
      pc = (vtx2(0) - vtx1(0)) * (vtx3(1) - vtx1(1))
         - (vtx3(0) - vtx1(0)) * (vtx2(1) - vtx1(1));
      pd = -(pa * vtx1(0) + pb * vtx1(1) + pc * vtx1(2));
      ps = std::sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;
    }
    explicit EdgeNavStatePVRtoPlan(const Vector3d& _p, double _pa, double _pb, double _pc, double _pd, double _s = 1):
            point(_p), pa(_pa), pb(_pb), pc(_pc), pd(_pd), ps(1), s(_s){};
    ~EdgeNavStatePVRtoPlan() override = default;

    bool read(std::istream& is) override{return true;}

    bool write(std::ostream& os) const override{return true;}

    void computeError() override;

    double computeError(const NavState& nslast, const NavState& nscurr);

    void linearizeOplus() override;

    void SetParams(Vector3d& _p, const Vector3d& _vtx1, const Vector3d& _vtx2, const Vector3d& _vtx3, const double _s = 1) {
      point = _p;
      s = _s;
      vtx1 = _vtx1;
      vtx2 = _vtx2;
      vtx3 = _vtx3;
    }
    void SetParams(const Vector3d& _p, double _pa, double _pb, double _pc, double _pd, double _s = 1){
      point = _p;
      pa = _pa;
      pb = _pb;
      pc = _pc;
      pd = _pd;
      ps = 1;
      s = _s;
    };

protected:
    double s;
    Vector3d point;
    Vector3d vtx1;
    Vector3d vtx2;
    Vector3d vtx3;
    double pa, pb, pc, pd, ps;
    Vector3d P_to_start;
    Vector3d Pcl;
    Quaterniond qlc;
    Quaterniond delta_qlc;
    float _weight = 1;
};

class EdgeIMUNavStatePVRtoPlan : public BaseBinaryEdge<1, double, VertexNavStatePVR, VertexNavStatePVR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeIMUNavStatePVRtoPlan(const Vector3d& _p, const Vector3d& _vtx1, const Vector3d& _vtx2, const Vector3d& _vtx3, double _s = 1):
            point(_p), vtx1(_vtx1), vtx2(_vtx2), vtx3(_vtx3), s(_s){
      pa = (vtx2(1) - vtx1(1)) * (vtx3(2) - vtx1(2))
           - (vtx3(1) - vtx1(1)) * (vtx2(2) - vtx1(2));
      pb = (vtx2(2) - vtx1(2)) * (vtx3(0) - vtx1(0))
           - (vtx3(2) - vtx1(2)) * (vtx2(0) - vtx1(0));
      pc = (vtx2(0) - vtx1(0)) * (vtx3(1) - vtx1(1))
           - (vtx3(0) - vtx1(0)) * (vtx2(1) - vtx1(1));
      pd = -(pa * vtx1(0) + pb * vtx1(1) + pc * vtx1(2));
      ps = std::sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;
    }
    explicit EdgeIMUNavStatePVRtoPlan(const Vector3d& _p, double _pa, double _pb, double _pc, double _pd, double _s = 1):
            point(_p), pa(_pa), pb(_pb), pc(_pc), pd(_pd), ps(1), s(_s){};
//    ~EdgeNavStatePVRtoPlan() override = default;

    bool read(std::istream& is) override{return true;}

    bool write(std::ostream& os) const override{return true;}

    void computeError() override;

    double computeError(const NavState& nslast, const NavState& nscurr);

    void linearizeOplus() override;

    void SetParams(const Matrix3d& _Rbl, const Vector3d& _Pbl){
      Rbl = _Rbl;
      Pbl = _Pbl;
      Rlb = _Rbl.transpose();
      Plb = -Rlb * _Pbl;
      qbl = Quaterniond(Rbl).normalized();
      qlb = Quaterniond(Rlb).normalized();
    }
    void SetParams(const Matrix4d& _Tbl){
      Rbl = _Tbl.topLeftCorner(3,3);
      Pbl = _Tbl.topRightCorner(3,1);
      Rlb = Rbl.transpose();
      Plb = -Rlb * Pbl;
      qbl = Quaterniond(Rbl).normalized();
      qlb = Quaterniond(Rlb).normalized();
    }

    const float& get_Weight(){return _weight;}

protected:
    double s;
    Vector3d point;
    Vector3d vtx1;
    Vector3d vtx2;
    Vector3d vtx3;
    double pa, pb, pc, pd, ps;
    Matrix3d Rbl, Rlb;
    Quaterniond qbl, qlb;
    Vector3d Pbl, Plb;
    float _weight = 1;

    Quaterniond q_half;
    Quaterniond qlc;
    Vector3d Pcl;
    Vector3d P_to_start;
};

class EdgeNavStatePVR : public BaseMultiEdge<9, IMUPreintegrator>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePVR() : BaseMultiEdge<9, IMUPreintegrator>() {
        resize(3);
    }

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    void computeError();

    virtual void linearizeOplus();

    void SetParams(const Vector3d& gw) {
        GravityVec = gw;
    }

protected:
    // Gravity vector in 'world' frame
    Vector3d GravityVec;
};

class EdgeNavStatePVRGravity : public BaseMultiEdge<9, IMUPreintegrator>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePVRGravity() : BaseMultiEdge<9, IMUPreintegrator>() {
      resize(4);
    }

    bool read(std::istream& is) override {return true;}

    bool write(std::ostream& os) const override {return true;}

    void computeError() override;

    void linearizeOplus() override;

protected:

};

class EdgeGravityNorm : public BaseUnaryEdge<1, double, VertexGravityVec>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGravityNorm() : BaseUnaryEdge<1, double, VertexGravityVec>() {}

    bool read(std::istream& is) override {return true;}

    bool write(std::ostream& os) const override {return true;}

    void computeError() override;

    void linearizeOplus() override;

protected:

};

class EdgeNavStateBias : public BaseBinaryEdge<6, Vector6d, VertexNavStateBias, VertexNavStateBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStateBias() : BaseBinaryEdge<6, Vector6d, VertexNavStateBias, VertexNavStateBias>() {}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    void computeError();

    virtual void linearizeOplus();

};

class EdgeNavStatePriorPVRBias : public BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePriorPVRBias() : BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>() {}

    bool read(std::istream &is){return true;}

    bool write(std::ostream &os) const{return true;}

    void computeError();

    virtual void linearizeOplus();

};
}

#endif // G2OTYPES_H
