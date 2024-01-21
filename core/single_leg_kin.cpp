/* related header files */
#include "single_leg_kin.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */

bool SingleLegKin::checkJointLimits(const Eigen::Vector3d &joints)
{
    for (size_t i = 0; i < 3; i++)
    {
        if (joints[i] < joint_limits_[2 * i] || joints[i] > joint_limits_[2 * i + 1])
        {
            return false;
        }
    }
    return true;
}

bool SingleLegKin::inverseKin(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &sols)
{
    Eigen::Matrix3d mirror = Eigen::Matrix3d::Zero();
    mirror.diagonal() = mirror_offset_;
    Eigen::Vector3d target = mirror * rot_offset_.transpose() * (pos - pos_offset_) + origin_calib_;
    Eigen::Vector3d approx = mirror * rot_offset_.transpose() * (ik_approx_point_ - pos_offset_) + origin_calib_;
    vector<double> target_vec = {target[0], target[1], target[2], approx[0], approx[1], approx[2]};

    vector<vector<double>> ret = IKFast_trans3D(target_vec, true);
    if (ret.size() == 0)
    {
        return false;
    }
    sols.resize(ret.size());
    for (size_t i = 0; i < ret.size(); i++)
    {
        sols[i] = Eigen::Map<Eigen::Vector3d>(ret[i].data());
    }
    return true;
}

bool SingleLegKin::inverseKinConstraint(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &sols)
{
    inverseKin(pos, sols);
    for (size_t i = 0; i < sols.size(); i++)
    {
        if (!checkJointLimits(sols[i]))
        {
            sols.erase(sols.begin() + i);
            i--;
        }
    }
    return sols.size() > 0;
}

bool SingleLegKin::inverseKinConstraint(const Eigen::Vector3d &pos, Eigen::Vector3d &sol)
{
    vector<Eigen::Vector3d> sols;
    inverseKinConstraint(pos, sols);
    if (sols.size() == 0)
    {
        return false;
    }
    sol = sols[0];
    return true;
}

bool SingleLegKin::forwardKin(const Eigen::Vector3d &joints, Eigen::Vector3d &pos)
{
    pinocchio::forwardKinematics(model_, data_, joints);
    Eigen::Matrix3d mirror = Eigen::Matrix3d::Zero();
    mirror.diagonal() = mirror_offset_;
    pos = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId(end_effector_name_)).translation();
    pos = rot_offset_ * mirror * (pos - origin_calib_) + pos_offset_;
    return true;
}

bool SingleLegKin::forwardKinConstraint(const Eigen::Vector3d &joints, Eigen::Vector3d &pos)
{
    if (!checkJointLimits(joints))
    {
        return false;
    }
    return forwardKin(joints, pos);
}