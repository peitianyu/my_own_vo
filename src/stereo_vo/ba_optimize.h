#ifndef __BA_OPTIMIZE_H__
#define __BA_OPTIMIZE_H__

#include "factor_graph/factor.h"
#include "factor_graph/variable.h"

#include <Eigen/Geometry>

class BaVariable : public Variable
{
public:
    BaVariable(const Eigen::Vector4d& x) : x_(x) {}

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

class BaFactor : public Factor
{
public:
private:
};



#endif // __BA_OPTIMIZE_H__