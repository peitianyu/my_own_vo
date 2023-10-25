#ifndef __PT2POLAR_FACTOR_H__
#define __PT2POLAR_FACTOR_H__

#include "factor_graph/factor.h"
#include "factor_graph/variable.h"

#include <Eigen/Geometry>

class Pt2PolarVariable : public Variable
{
public:
    Pt2PolarVariable(const Eigen::Vector4d& x) : x_(x) {}

    Eigen::Matrix3d ext_R() const {
        Eigen::Quaterniond q(1, x_(0), x_(1), x_(2));
        return q.normalized().toRotationMatrix();
    }

    Eigen::Vector4d x() const { return x_; }

    double scale() const { return x_(3); }

    virtual size_t Dim() const { return 4; }

    virtual void Plus(const Eigen::VectorXd &delta) { x_ += delta; }

    virtual void Print() const { std::cout << "x: " << x_.transpose() << std::endl; }
private:
    Eigen::Vector4d x_;
};

class Pt2PolarFactor : public Factor
{
public:
    Pt2PolarFactor(Pt2PolarVariable *v_a, const Eigen::Matrix4d& T, const double& baseline, const Eigen::Matrix3d& K,
        const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const uint& mode = 0) 
        : T_(T), baseline_(baseline), K_inv_(K.inverse()), p0_(p0), p1_(p1), mode_(mode) { AddVariable(v_a);}

    virtual int Dim() const override { return 1;}

    virtual Eigen::VectorXd Error() const override
    {
        GRAPH_ASSERT(this->NumVariables() == 1);
        const Pt2PolarVariable *v_a = static_cast<Pt2PolarVariable *>(this->VariableAt(0));
        
        double bl = 0;
        if(mode_ == 0) bl = baseline_;
        else if(mode_ == 1) bl = -baseline_;
        else if(mode_ == 2) bl = 0;
          

        Eigen::Matrix4d T01 = Eigen::Matrix4d::Identity();
        T01 << T_(0,0), T_(0,1), T_(0,2), T_(0,3) * v_a->scale(),
               T_(1,0), T_(1,1), T_(1,2), T_(1,3) * v_a->scale(),
               T_(2,0), T_(2,1), T_(2,2), T_(2,3) * v_a->scale(),
               0, 0, 0, 1;
        
        Eigen::Matrix4d ext_T = Eigen::Matrix4d::Identity();
        ext_T.block<3, 3>(0, 0) = v_a->ext_R();
        ext_T(0, 3) = bl;
        
        T01 = T01 * ext_T;

        Eigen::Matrix3d E = hat(T01.block<3, 1>(0, 3)) * T01.block<3, 3>(0, 0);

        Eigen::Vector3d x0 = K_inv_ * p0_;
        Eigen::Vector3d x1 = K_inv_ * p1_;

        Eigen::VectorXd r(1);
        r <<  x0.transpose() * E * x1;
        return r;
    }

    Eigen::Matrix3d hat(const Eigen::Vector3d& x) const
    {
        Eigen::Matrix3d x_hat;
        x_hat << 0, -x(2), x(1),
                 x(2), 0, -x(0),
                 -x(1), x(0), 0;
        return x_hat;
    }
private:
    Eigen::Matrix4d T_;
    double baseline_;
    Eigen::Matrix3d K_inv_;
    Eigen::Vector3d p0_;
    Eigen::Vector3d p1_;
    uint mode_;
};








#endif // __PT2POLAR_FACTOR_H__