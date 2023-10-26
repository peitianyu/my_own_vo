#ifndef __LINEAR_REGRESSION_OPTIMIZE_H__
#define __LINEAR_REGRESSION_OPTIMIZE_H__

#include <memory>
#include "factor_graph/graph_optimize.h"

class LinearRegressionVariable : public Variable
{
public:
    LinearRegressionVariable(const Eigen::VectorXd& x) : x_(x) {}

    Eigen::VectorXd x() const { return x_; }

    virtual size_t Dim() const { return x_.size(); }

    virtual void Plus(const Eigen::VectorXd &delta) { x_ += delta; }

    virtual void Print() const { std::cout << "x: " << x_.transpose() << std::endl; }
private:
    Eigen::VectorXd x_;
};

class LinearRegressionFactor : public Factor
{
public:
    LinearRegressionFactor(LinearRegressionVariable *v_a, const Eigen::MatrixXd& measurement) : measurement_(measurement)
    {
        AddVariable(v_a);
    }

    virtual int Dim() const override { return 1;}

    virtual Eigen::VectorXd Error() const override // 重写
    {
        GRAPH_ASSERT(this->NumVariables() == 1);
        return Eigen::VectorXd::Zero(1);
    }
protected:
    Eigen::MatrixXd measurement_;
};

template<typename LinearRegressionFactorType>
class LinearRegressionOptimize 
{
public:
    LinearRegressionOptimize() {}

    void Optimize(const Eigen::VectorXd& X, const Eigen::MatrixXd& measurement)
    {
        FactorGraph factor_graph;
        LinearRegressionVariable *v_x = new LinearRegressionVariable(X);
        factor_graph.AddVariable(v_x);
        for(int i = 0; i < measurement.rows(); i++)
        {
            LinearRegressionFactorType *factor = new LinearRegressionFactorType(v_x, measurement.row(i));
            factor_graph.AddFactor(factor);
        }

        GraphOptimize graph_optimize = GraphOptimize(GraphOptimize::Option());
        graph_optimize.OptimizeGN(&factor_graph);

        X_ = v_x->x();
    }

    Eigen::VectorXd X() const { return X_; }

    void Print() const { std::cout << "X: " << X_.transpose() << std::endl; }
private:
    Eigen::VectorXd X_;
};


#endif // __LINEAR_REGRESSION_OPTIMIZE_H__