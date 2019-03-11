#pragma once

#include <pcl/features/pfh.h>

#include "common.h"

#define ALPHA_PLANE M_PI_2
#define PHI_PLANE M_PI_2
#define THETA_PLANE M_PI_2
#define ERROR_TOLERANCE 0.349 /*20°*/

class PFHEvaluation {
public:
    PFHEvaluation(){}
    ~PFHEvaluation(){}

    static bool isValidPlane(PointNormalCloud::Ptr points, vector<int> &indices_in);

private:
    bool isInInterval(float mean, float dev, float ref);
};
