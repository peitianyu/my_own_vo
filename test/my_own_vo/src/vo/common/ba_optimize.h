#ifndef __BA_OPTIMIZE_H__
#define __BA_OPTIMIZE_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

class BA_Optimize
{
public:
    struct Option
    {
        uint max_iters;
        double eps;
        Eigen::Matrix3d camera_info;

        // fx, fy, cx, cy
        double fx() { return camera_info(0, 0); }
        double fy() { return camera_info(1, 1); }
        double cx() { return camera_info(0, 2); }
        double cy() { return camera_info(1, 2); }

        Option(const uint& _max_iters = 100, const double& _eps = 1e-6, const Eigen::Matrix3d& _camera_info = Eigen::Matrix3d::Identity())
            : max_iters(_max_iters), eps(_eps), camera_info(_camera_info) {}
    };

    BA_Optimize(const Option& option = Option())
        : p_(option){}


    void optimize(const std::vector<Eigen::Vector2d>& p2ds, const std::vector<Eigen::Vector3d>& p3ds, Eigen::Matrix4d& pose)
    {
        double err = 0, err_prev = 999999999.9;
        for(uint iters = 0; iters < p_.max_iters; iters++)
        {
            Eigen::VectorXd delta_x = optimize_func(err, p2ds, p3ds, pose);

            if(std::isnan(delta_x[0])) {
                pose = Eigen::Matrix4d::Identity();
                break;
            }

            if(converged(delta_x, err_prev, err)) break;

            err_prev = err;
        }
    }
private:
    Eigen::VectorXd optimize_func(double& err, const std::vector<Eigen::Vector2d>& p2ds, const std::vector<Eigen::Vector3d>& p3ds, Eigen::Matrix4d& pose)
    {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6);

        for (uint i = 0; i < p3ds.size(); i++) 
        {
            Eigen::Vector3d pc = pose.block(0, 0, 3, 3) * p3ds[i] + pose.block(0, 3, 3, 1);
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;

            Eigen::Vector2d proj(p_.fx()*inv_z*pc[0]+p_.cx(), p_.fy()*inv_z*pc[1]+p_.cy());
            Eigen::Vector2d e = p2ds[i] - proj;

            err += e.squaredNorm();
            
            Eigen::Matrix<double, 2, 6> J = Eigen::Matrix<double, 2, 6>::Zero();
            J << (-p_.fx()*inv_z), 0, (p_.fx()*pc[0]*inv_z2), (p_.fx()*pc[0]*pc[1]*inv_z2), (-p_.fx()-p_.fx()*pc[0]*pc[0]*inv_z2), (p_.fx()*pc[1]*inv_z),
                0, (-p_.fy()*inv_z), (p_.fy()*pc[1]*inv_z2), (p_.fy()+p_.fy()*pc[1]*pc[1]*inv_z2), (-p_.fy()*pc[0]*pc[1]*inv_z2), (-p_.fy()*pc[0]*inv_z);

            for(uint j = 0; j < 6; j++){
            for(uint k = j; k < 6; k++) H(j, k) += J.col(j).transpose() * J.col(k); }
               
            b += -J.transpose() * e;
        }

        for(uint j = 0; j < 6; j++){
        for(uint k = j; k < 6; k++) H(k, j) = H(j, k); }

        err /= p3ds.size();        
        Eigen::VectorXd dx = H.ldlt().solve(b);

        update_pose(dx, pose);

        return dx;
    }
    
    void update_pose(const Eigen::VectorXd& dx, Eigen::Matrix4d& pose)
    {
        Eigen::Quaterniond d_q(1, dx[3] / 2, dx[4] / 2, dx[5] / 2);
        d_q.normalized();
        Eigen::Vector3d d_t = dx.head<3>();

        pose.block(0, 0, 3, 3) = d_q.toRotationMatrix() * pose.block(0, 0, 3, 3);
        pose.block(0, 3, 3, 1) += d_t;
    }

    bool converged(const Eigen::VectorXd& delta_x, const double& err_prev, const double& err)
    {
        if(delta_x.head(6).norm() < p_.eps)    return true;
        if( err < p_.eps)                      return true;
        if( fabs(err-err_prev) < p_.eps )      return true;

        return false;
    }
private:
    Option p_;
};


#endif // __BA_OPTIMIZE_H__