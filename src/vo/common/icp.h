#ifndef __ICP_H__
#define __ICP_H__

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>



class ICP
{
public:
    struct Option
    {
        uint max_iters;
        double eps;

        Option(const uint& _max_iters = 5, const double& _eps = 1e-6)
            : max_iters(_max_iters), eps(_eps) {}
    };

    ICP(const Option& option = Option()) : p_(option){}
    
    void optimize(const std::vector<Eigen::Vector3d>& ref_pts, const std::vector<Eigen::Vector3d>& cur_pts, Eigen::Matrix4d& pose)
    {
        double err = 0, err_prev = 999999999.9;
        for(uint iters = 0; iters < p_.max_iters; iters++)
        {
            Eigen::VectorXd delta_x = optimize_func(err, ref_pts, cur_pts, pose);

            if(std::isnan(delta_x[0])) {
                pose = Eigen::Matrix4d::Identity();
                break;
            }

            if(converged(delta_x, err_prev, err)) break;

            err_prev = err;
        }
    }
private:
    Eigen::VectorXd optimize_func(double& err, const std::vector<Eigen::Vector3d>& ref_pts, 
                                    const std::vector<Eigen::Vector3d>& cur_pts, Eigen::Matrix4d& pose)
    {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6);

        for (uint i = 0; i < ref_pts.size(); i++) 
        {
            Eigen::Vector3d world_pt = pose.block(0, 0, 3, 3) * ref_pts[i] + pose.block(0, 3, 3, 1);
            Eigen::Vector3d error = world_pt - cur_pts[i] ;

            Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
            J.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
            J.block(0, 3, 3, 3) = -Eigen::Matrix3d::Identity() * skew(ref_pts[i]);

            err += error.transpose() * error;

            H += J.transpose() * J;
            b += -J.transpose() * error;
        }

        Eigen::VectorXd delta_x = H.ldlt().solve(b);

        update_pose(delta_x, pose);
        return delta_x;
    }

    void update_pose(const Eigen::VectorXd& dx, Eigen::Matrix4d& pose)
    {
        Eigen::Quaterniond d_q(1, dx[3] / 2, dx[4] / 2, dx[5] / 2);
        d_q.normalized();
        Eigen::Vector3d d_t = dx.head<3>();

        pose.block(0, 0, 3, 3) = d_q.toRotationMatrix() * pose.block(0, 0, 3, 3);
        pose.block(0, 3, 3, 1) += d_t;
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d& vec)
    {
        return (Eigen::Matrix3d() << 0, -vec[2], vec[1],
                                     vec[2], 0, -vec[0],
                                     -vec[1], vec[0], 0).finished();
    }

    bool converged(const Eigen::VectorXd& delta_x, const double& err_prev, const double& err)
    {
        if(delta_x.norm() < p_.eps)         return true;
        if(fabs(err_prev - err) < p_.eps)   return true;
        if(err < p_.eps)                    return true;
        return false;
    }
private:
    Option p_;
};

#endif // __ICP_H__