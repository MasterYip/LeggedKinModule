#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
#include <string>
/* external project header files */
#include "Eigen/Dense"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
/* internal project header files */
#include "config.h"

using namespace ikfast_leg;

class SingleLegKin
{
private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::string end_effector_name_ = "RF_FOOT";
    // Origin Calib: move the origin of the robot to the origin of the world frame
    Eigen::Vector3d origin_calib_ = Eigen::Vector3d::Zero(); // robot origin in world frame
    // Installation Offsets (in order: mirror, rot, pos )
    Eigen::Vector3d mirror_offset_ = Eigen::Vector3d::Ones();  // Mirror operation
    Eigen::Matrix3d rot_offset_ = Eigen::Matrix3d::Identity(); // Rotation operation
    Eigen::Vector3d pos_offset_ = Eigen::Vector3d::Zero();     // Translation operation
    // Joint limits (default: [-pi, pi], order: [min, max, min, max, min, max])
    Eigen::VectorXd joint_limits_ = Eigen::VectorXd::Zero(6);
    // IKFast Approx point (usually the middle point of the joint limits)
    Eigen::Vector3d ik_approx_point_ = Eigen::Vector3d::Zero();

public:
    SingleLegKin(std::string urdf_file_path);
    ~SingleLegKin() = default;
    // Setters
    void setEndEffectorName(const std::string &end_effector_name) { end_effector_name_ = end_effector_name; };
    void setOriginCalib(const Eigen::Vector3d &origin_calib) { origin_calib_ = origin_calib; };
    void setMirrorOffset(const Eigen::Vector3d &mirror_offset) { mirror_offset_ = mirror_offset; };
    void setPosOffset(const Eigen::Vector3d &pos_offset) { pos_offset_ = pos_offset; };
    void setRotOffset(const Eigen::Matrix3d &rot_offset) { rot_offset_ = rot_offset; };
    void setJointLimits(const Eigen::VectorXd &joint_limits) { joint_limits_ = joint_limits; };
    void setIKApproxPoint(const Eigen::Vector3d &ik_approx_point) { ik_approx_point_ = ik_approx_point; };

    bool checkJointLimits(const Eigen::Vector3d &joints);
    // Kinematics
    /* IK - multiple solutions */
    bool inverseKin(const Eigen::Vector3d &pos, std::vector<Eigen::Vector3d> &sols, bool approx = true);
    /* IK with constraint - multiple solutions */
    bool inverseKinConstraint(const Eigen::Vector3d &pos, std::vector<Eigen::Vector3d> &sols, bool approx = true);
    /* IK with constraint - first solution */
    bool inverseKinConstraint(const Eigen::Vector3d &pos, Eigen::Vector3d &sol, bool approx = true);
    bool forwardKin(const Eigen::Vector3d &joints, Eigen::Vector3d &pos);
    bool forwardKinConstraint(const Eigen::Vector3d &joints, Eigen::Vector3d &pos);
};
