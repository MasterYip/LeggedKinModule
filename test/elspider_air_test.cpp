#include "elspider_air_kin.h"

int main()
{
    ElSpiderKin elair = ElSpiderKin();
    printf("IK: ");
    Eigen::Vector3d pos = {0.35350208, -0.22998902, -0.1360658};
    vector<Eigen::Vector3d> sols;
    elair.inverseKinConstraint(pos, sols, 0);
    cout << sols.size() << endl;
    for (size_t i = 0; i < sols.size(); i++)
    {
        cout << sols[i].transpose() << endl;
    }

    printf("FK: ");
    Eigen::Vector3d joints = {0.0, 0.0, 0.0};
    Eigen::Vector3d pos_fk;
    elair.forwardKin(joints, pos_fk, 0);
    cout << pos_fk.transpose() << endl;
}