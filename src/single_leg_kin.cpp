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
    vector<double> target = {pos[0] + origin_calib_[0], pos[1] + origin_calib_[1], pos[2] + origin_calib_[2]};
    vector<vector<double>> ret = IKFast_trans3D(target, true);
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

bool SingleLegKin::forwardKin(const Eigen::Vector3d &joints, Eigen::Vector3d &pos)
{
    pinocchio::forwardKinematics(model_, data_, joints);
    pos = data_.oMi[3].translation();
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