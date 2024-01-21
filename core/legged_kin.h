#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "Eigen/Dense"
/* internal project header files */
#include "single_leg_kin.h"

class LeggedKin
{
private:
    uint num_legs_;
    vector<SingleLegKin> legs_;
public:
    
};