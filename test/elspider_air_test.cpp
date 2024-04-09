#include "elspider_air_kin.h"

int main()
{
    ElSpiderKin elair = ElSpiderKin();
    Eigen::Vector3d joints;
    std::cout << "Input joints: ";
    std::cin >> joints[0] >> joints[1] >> joints[2];
    // Eigen::Vector3d pos = {0.35350208, -0.22998902, -0.1360658};

    for (int leg_index = 0; leg_index < 6; leg_index++)
    {
        printf("Leg %d:\n", leg_index);
        // FK
        Eigen::Vector3d pos_fk;
        elair.forwardKin(joints, pos_fk, leg_index);
        printf("FK: %.4f, %.4f, %.4f\n", pos_fk[0], pos_fk[1], pos_fk[2]);
        for (int joint_idx = 0; joint_idx < 3; joint_idx++)
        {
            elair.forwardKin(joints, pos_fk, leg_index, joint_idx);
            printf("Joint%d FK: %.4f, %.4f, %.4f\n", joint_idx, pos_fk[0], pos_fk[1], pos_fk[2]);
        }

        // IK
        Eigen::Vector3d sol;
        if (elair.inverseKinConstraint(pos_fk, sol, leg_index, true))
            printf("IK: %.4f, %.4f, %.4f\n", sol[0], sol[1], sol[2]);
        else
            printf("IK: no solution\n");

        // Jacobian
        Eigen::Matrix3Xd jacobian;
        elair.getJacobian(joints, jacobian, leg_index);
        std::cout << "EE Jacobian:\n" << jacobian << std::endl;

        for (int joint_idx = 0; joint_idx < 3; joint_idx++)
        {
            elair.getJacobian(joints, jacobian, leg_index, joint_idx);
            std::cout << "Joint" << joint_idx << " Jacobian:\n" << jacobian << std::endl;
        }
    }
}
