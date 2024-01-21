#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */

/* internal project header files */
#include "single_leg_kin.h"

class ElSpiderKin
{
private:
    std::vector<SingleLegKin> legs_;

public:
    ElSpiderKin()
    {
        legs_.resize(6, SingleLegKin(URDF_PATH));
        legs_[0].setOriginCalib(Eigen::Vector3d(0.3, -0.08, 0.011));
        legs_[1].setOriginCalib(Eigen::Vector3d(0.3, -0.08, 0.011));
        legs_[2].setOriginCalib(Eigen::Vector3d(0.3, -0.08, 0.011));
        legs_[3].setOriginCalib(Eigen::Vector3d(0.3, -0.08, 0.011));
        legs_[4].setOriginCalib(Eigen::Vector3d(0.3, -0.08, 0.011));
        legs_[5].setOriginCalib(Eigen::Vector3d(0.3, -0.08, 0.011));
        legs_[0].setMirrorOffset(Eigen::Vector3d(1, 1, 1));
        legs_[1].setMirrorOffset(Eigen::Vector3d(1, 1, 1));
        legs_[2].setMirrorOffset(Eigen::Vector3d(-1, 1, 1));
        legs_[3].setMirrorOffset(Eigen::Vector3d(1, -1, 1));
        legs_[4].setMirrorOffset(Eigen::Vector3d(1, -1, 1));
        legs_[5].setMirrorOffset(Eigen::Vector3d(-1, -1, 1));
        legs_[0].setPosOffset(Eigen::Vector3d(0.3, -0.08, 0.011));
        legs_[1].setPosOffset(Eigen::Vector3d(0, -0.14, 0.011));
        legs_[2].setPosOffset(Eigen::Vector3d(-0.3, -0.08, 0.011));
        legs_[3].setPosOffset(Eigen::Vector3d(0.3, 0.08, 0.011));
        legs_[4].setPosOffset(Eigen::Vector3d(0, 0.14, 0.011));
        legs_[5].setPosOffset(Eigen::Vector3d(-0.3, 0.08, 0.011));
        Eigen::VectorXd joint_limits;
        joint_limits.resize(6);
        joint_limits << -0.785, 0.785, -0.5233, 3.14, -0.6978, 3.925;
        for (size_t i = 0; i < 6; i++)
        {
            legs_[i].setJointLimits(joint_limits);
        }
    };
    ~ElSpiderKin(){};
    bool inverseKin(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &sols, uint leg_index)
    {
        return legs_[leg_index].inverseKin(pos, sols);
    };
    bool inverseKinConstraint(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &sols, uint leg_index)
    {
        return legs_[leg_index].inverseKinConstraint(pos, sols);
    };
    bool inverseKinConstraint(const Eigen::Vector3d &pos, Eigen::Vector3d &sol, uint leg_index)
    {
        return legs_[leg_index].inverseKinConstraint(pos, sol);
    };
    bool forwardKin(const Eigen::Vector3d &joints, Eigen::Vector3d &pos, uint leg_index)
    {
        return legs_[leg_index].forwardKin(joints, pos);
    };
    bool forwardKinConstraint(const Eigen::Vector3d &joints, Eigen::Vector3d &pos, uint leg_index)
    {
        return legs_[leg_index].forwardKinConstraint(joints, pos);
    };
};