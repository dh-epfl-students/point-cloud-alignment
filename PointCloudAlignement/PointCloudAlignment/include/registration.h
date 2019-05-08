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
    void computeMwithNormals();
    void computeMwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT);
    mat3 computeHwithNormals(vector<vec3> qs, vector<vec3> qt);
    mat3 computeHwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT);
    mat3 computeR(mat3 H);
    vec3 computeCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list, bool isMesh);
    vec3 computeCentersCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list);
    vector<vec3> computeDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid, bool isMesh);
    vector<vec3> computeCentersDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid);
};
