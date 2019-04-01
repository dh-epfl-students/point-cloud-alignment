#pragma once

#include <pcl/features/pfh.h>

#include "common.h"

#define PLANE_TRESHOLD 20

class PFHEvaluation {
public:
    PFHEvaluation(){}
    ~PFHEvaluation(){}

    static bool isValidPlane(PointNormalKCloud::Ptr points, vector<int> &indices_in);

private:
    bool isInInterval(float mean, float dev, float ref);
};
