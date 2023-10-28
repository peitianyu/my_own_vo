#ifndef __BA_OPTIMIZE_H__
#define __BA_OPTIMIZE_H__

#include "factor_graph/linear_regression_optimize.h"
#include <Eigen/Geometry>
/***************************************
 * 优化窗口为: 4
 * 优化变量为: T12, T23, T34, s23, s34
 * 约束为:  窗口内的所有特征点
 **************************************/
class BaFactor : public LinearRegressionFactor
{
public:
    BaFactor(LinearRegressionVariable *v_a, const Eigen::VectorXd& measurement, const Eigen::Matrix3d& K, const uint& mode = 0) 
        : LinearRegressionFactor(v_a, measurement), K_inv_(K.inverse()), mode_(mode) {}

    virtual Eigen::VectorXd Error() const override
    {
        GRAPH_ASSERT(v_a_->Dim() == 20);
        const LinearRegressionVariable *v_a = static_cast<LinearRegressionVariable *>(this->VariableAt(0));

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        if(mode_ == 0){
            Eigen::Matrix4d T12 = toT(v_a->x().segment<6>(0), 1.0);
            Eigen::Matrix4d T23 = toT(v_a->x().segment<6>(6), v_a->x()(18));
            T = T12 * T23;
        }else if(mode_ == 1){
            Eigen::Matrix4d T23 = toT(v_a->x().segment<6>(0), v_a->x()(18));
            Eigen::Matrix4d T34 = toT(v_a->x().segment<6>(6), v_a->x()(19));
            T = T23 * T34;
        } else if(mode_ == 2){
            Eigen::Matrix4d T12 = toT(v_a->x().segment<6>(0), 1.0);
            Eigen::Matrix4d T23 = toT(v_a->x().segment<6>(6), v_a->x()(18));
            Eigen::Matrix4d T34 = toT(v_a->x().segment<6>(6), v_a->x()(19));
            T = T12 * T23 * T34;
        }

        Eigen::Matrix3d E = hat(T.block<3, 1>(0, 3)) * T.block<3, 3>(0, 0);

        Eigen::Vector2d p0 = measurement_.segment<2>(0);
        Eigen::Vector2d p1 = measurement_.segment<2>(2);
        Eigen::Vector3d x0 = K_inv_ * p0; // curr
        Eigen::Vector3d x1 = K_inv_ * p1; // prev

        Eigen::Vector3d l0 = E * x0;
        Eigen::Vector3d l1 = E.transpose() * x1;

        double dis = x0.transpose() * E * x1;

        double d0 = dis / l0.head<2>().norm();
        double d1 = dis / l1.head<2>().norm();        

        Eigen::VectorXd r(1);
        r <<  d0*d0 + d1*d1;
        return r;
    }
private:
    Eigen::Matrix3d hat(const Eigen::Vector3d& x) const
    {
        Eigen::Matrix3d x_hat;
        x_hat << 0, -x(2), x(1),
                 x(2), 0, -x(0),
                 -x(1), x(0), 0;
        return x_hat;
    }
    Eigen::Matrix4d toT(const Eigen::VectorXd& x, const double& s) const {
        GRAPH_ASSERT(x.size() == 6);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R(x.segment<3>(0));
        T.block<3, 1>(0, 3) = x.segment<3>(3) * s;
        return T;
    }

    Eigen::Matrix3d R(const Eigen::Vector3d& x) const {
        return Eigen::Quaterniond(1, x(0), x(1), x(2)).normalized().toRotationMatrix();
    }
private:
    Eigen::Matrix3d K_inv_;

    uint mode_; // 0: 13, 1: 24, 2: 14
};



#endif // __BA_OPTIMIZE_H__