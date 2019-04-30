#pragma once

#include "common.h"
#include "segmented_points_container.h"

class Registration {
public:
    void setClouds(vector<SegmentedPointsContainer::SegmentedPlane> &source, vector<SegmentedPointsContainer::SegmentedPlane> &target, bool isMesh);
    mat3 findAlignment();

private:
    bool targetIsMesh = false;
    Eigen::MatrixXf M;
    vector<SegmentedPointsContainer::SegmentedPlane> source;
    vector<SegmentedPointsContainer::SegmentedPlane> target;

    mat3 findRotation();
    mat3 findTranslation();
    void computeM();
    mat3 computeH();
    mat3 computeR(mat3 H);
    vec3 computeCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list, bool isMesh);
};
