#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <string>
#include <vector>
/* external project header files */
#include "Eigen/Dense"
#ifndef USE_PINOCCHIO // If USE_PINOCCHIO is defined, then it assumes that the user has already included pinocchio
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#endif
/* internal project header files */
#include "config.h"

using namespace ikfast_leg;

class SingleLegKin
{
  private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::vector<std::string> joint_names_ = {"RF_HAA", "RF_HFE_Coll", "RF_KFE_Coll"};
    std::string end_effector_name_ = "RF_FOOT";
    // Origin Calib: move the origin of the leg to the origin of the base frame
    Eigen::Vector3d origin_calib_ = Eigen::Vector3d::Zero(); // leg origin in base frame
    // Installation Offsets (in order: mirror, rot, pos )
    Eigen::Vector3d mirror_offset_ = Eigen::Vector3d::Ones();  // Mirror operation
    Eigen::Matrix3d rot_offset_ = Eigen::Matrix3d::Identity(); // Rotation operation
    Eigen::Vector3d pos_offset_ = Eigen::Vector3d::Zero();     // Translation operation
    // Joint Directions (default: [1, 1, 1], decide before joint limits)
    Eigen::Vector3d joint_directions_ = Eigen::Vector3d::Ones();
    // Joint limits (default: [-pi, pi], order: [min, max, min, max, min, max])
    Eigen::VectorXd joint_limits_ = Eigen::VectorXd::Zero(6);
    // IKFast Approx point (usually the middle point of the joint limits)
    Eigen::Vector3d ik_approx_point_ = Eigen::Vector3d::Zero();

    Eigen::Matrix3d mirror_offset_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d joint_dir_mat_ = Eigen::Matrix3d::Identity();

  public:
    SingleLegKin(std::string urdf_file_path);
    ~SingleLegKin() = default;
    // Setters
    void setEndEffectorName(const std::string &end_effector_name)
    {
        end_effector_name_ = end_effector_name;
    };
    void setOriginCalib(const Eigen::Vector3d &origin_calib)
    {
        origin_calib_ = origin_calib;
    };
    void setMirrorOffset(const Eigen::Vector3d &mirror_offset)
    {
        mirror_offset_ = mirror_offset;
        mirror_offset_mat_.diagonal() = mirror_offset_;
    };
    void setPosOffset(const Eigen::Vector3d &pos_offset)
    {
        pos_offset_ = pos_offset;
    };
    void setRotOffset(const Eigen::Matrix3d &rot_offset)
    {
        rot_offset_ = rot_offset;
    };
    void setJointLimits(const Eigen::VectorXd &joint_limits)
    {
        joint_limits_ = joint_limits;
    };
    void setJointDirections(const Eigen::Vector3d &joint_directions)
    {
        joint_directions_ = joint_directions;
        joint_dir_mat_.diagonal() = joint_directions_;
    };
    void setIKApproxPoint(const Eigen::Vector3d &ik_approx_point)
    {
        ik_approx_point_ = ik_approx_point;
    };

    bool checkJointLimits(const Eigen::Vector3d &joints);
    void constraintFiltering(std::vector<Eigen::Vector3d> &sols);

    // Kinematics
    /* IK - multiple solutions */
    bool inverseKin(const Eigen::Vector3d &pos, std::vector<Eigen::Vector3d> &sols,
                    bool approx = true);
    /* IK with constraint - multiple solutions */
    bool inverseKinConstraint(const Eigen::Vector3d &pos, std::vector<Eigen::Vector3d> &sols,
                              bool approx = true, uint iter = 10);
    /* IK with constraint - first solution */
    bool inverseKinConstraint(const Eigen::Vector3d &pos, Eigen::Vector3d &sol, bool approx = true,
                              uint iter = 10);
    /* FK */
    bool forwardKin(const Eigen::Vector3d &joints, Eigen::Vector3d &pos);
    bool forwardKin(const Eigen::Vector3d &joints, int joint_idx, Eigen::Vector3d &pos);
    bool forwardKinConstraint(const Eigen::Vector3d &joints, Eigen::Vector3d &pos);

    // Jacobian

    /**
     * @brief Get the Jacobian w.r.t. the end effector
     * 
     * @param joints 
     * @param jac 
     * @return true 
     * @return false 
     */
    bool getJacobian(const Eigen::Vector3d &joints, Eigen::Matrix3Xd &jac);

    /**
     * @brief Get joint jacobian given the joint frame index
     * 
     * @param joints 
     * @param joint_idx Joint frame select (0: RF_HAA, 1: RF_HFE, 2: RF_KFE)
     * @param jac 
     * @return true 
     * @return false 
     */
    bool getJacobian(const Eigen::Vector3d &joints, int joint_idx, Eigen::Matrix3Xd &jac);

    bool getJacobianTimeVariation(const Eigen::Vector3d &joints, const Eigen::Vector3d &vel,
                                  Eigen::Matrix3Xd &jac_dot);

    bool getJacobianTimeVariation(const Eigen::Vector3d &joints, const Eigen::Vector3d &vel,
                                  int joint_idx, Eigen::Matrix3Xd &jac_dot);
};
