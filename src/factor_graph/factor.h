#ifndef __FACTOR_H__
#define __FACTOR_H__

#include<vector>
#include<memory>
#include<Eigen/Core>
#include"variable.h"
#include "utils.h"
#include <iostream>

class Factor
{
public:
    static constexpr int kMaxVariables = 2;

    virtual ~Factor() {}

    int NumVariables() const;

    void AddVariable(Variable *v);

    Variable *VariableAt(int idx) const;

    // Dimensionality of the error.
    virtual int Dim() const = 0;

    virtual Eigen::VectorXd Error() const = 0;

    // Returns e1 - e2.
    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2) const { return e1 - e2; }

    // Jacobian wrt to the variable at idx. Defaults
    // to computing the jacobian numerically.
    virtual Eigen::MatrixXd Jacobian(int idx) const;
protected:
    Eigen::MatrixXd ComputeNumericalJacobian(Variable * v) const;
    std::array<Variable *, kMaxVariables> m_variables;
    int m_num_variables = 0;
};

#endif // __FACTOR_H__