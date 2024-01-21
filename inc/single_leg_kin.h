#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
#include <string>
/* external project header files */
#include "Eigen/Dense"
// Pinocchio
#include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/geometry.hpp"
// #include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
/* internal project header files */
#include "ikfast_leg_lib.h"

using namespace ikfast_leg;
using namespace std;
class SingleLegKin
{
private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    // Origin Calib
    // urdf origin may not be [0,0,0]
    Eigen::Vector3d origin_calib_ = Eigen::Vector3d::Zero();
    // Offsets (in order)
    Eigen::Vector3d mirror_offset_ = Eigen::Vector3d::Ones();
    Eigen::Vector3d pos_offset_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot_offset_ = Eigen::Matrix3d::Identity();
    // Joint limits
    Eigen::VectorXd joint_limits_ = Eigen::VectorXd::Zero(6); // FIXME: hard code

public:
    SingleLegKin(string urdf_file_path)
    {
        pinocchio::urdf::buildModel(urdf_file_path, model_);
        data_ = pinocchio::Data(model_);
    };
    ~SingleLegKin() = default;
    bool checkJointLimits(const Eigen::Vector3d &joints);
    bool inverseKin(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &sols);
    bool inverseKinConstraint(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &sols);
    bool forwardKin(const Eigen::Vector3d &joints, Eigen::Vector3d &pos);
    bool forwardKinConstraint(const Eigen::Vector3d &joints, Eigen::Vector3d &pos);
};
