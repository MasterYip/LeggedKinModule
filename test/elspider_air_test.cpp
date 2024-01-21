#include "elspider_air_kin.h"

int main()
{
    ElSpiderKin elair = ElSpiderKin();
    Eigen::Vector3d joints;
    cout << "Input joints: ";
    cin >> joints[0] >> joints[1] >> joints[2];
    // Eigen::Vector3d pos = {0.35350208, -0.22998902, -0.1360658};

    for (int leg_index = 0; leg_index < 6; leg_index++)
    {
        printf("FK: ");
        Eigen::Vector3d pos_fk;
        elair.forwardKinConstraint(joints, pos_fk, leg_index);
        cout << pos_fk.transpose() << endl;

        printf("IK: ");
        // vector<Eigen::Vector3d> sols;
        // elair.inverseKinConstraint(pos_fk, sols, leg_index);
        // cout << sols.size() << endl;
        // for (size_t i = 0; i < sols.size(); i++)
        // {
        //     cout << sols[i].transpose() << endl;
        // }
        Eigen::Vector3d sol;
        elair.inverseKinConstraint(pos_fk, sol, leg_index);
        cout << sol.transpose() << endl;
    }
}