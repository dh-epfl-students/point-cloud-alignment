#pragma once

#include <cmath>
#include <fstream>
#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>

#include "common.h"
#include "segmented_points_container.h"

#define MIN_SURFACE 50

class Registration {
public:
    void setClouds(vector<SegmentedPointsContainer::SegmentedPlane> &source, vector<SegmentedPointsContainer::SegmentedPlane> &target, bool targetIsMesh, bool sourceIsMesh,
                   PointNormalKCloud::Ptr p_source_cloud = nullptr, PointNormalKCloud::Ptr p_target_cloud = nullptr);
    mat3 findAlignment();
    void setCallback(function<void(SegmentedPointsContainer::SegmentedPlane, SegmentedPointsContainer::SegmentedPlane, ivec3)> callable) { this->display_update_callable = callable; }

private:
    bool targetIsMesh = false;
    bool sourceIsMesh = false;
    Eigen::MatrixXf M;
    PointNormalKCloud::Ptr p_cloud;
    vector<SegmentedPointsContainer::SegmentedPlane> source;
    vector<SegmentedPointsContainer::SegmentedPlane> target;
    vector<float> source_surfaces;
    vector<float> target_surfaces;

    function<void(SegmentedPointsContainer::SegmentedPlane, SegmentedPointsContainer::SegmentedPlane, ivec3)> display_update_callable;

    void filterPlanes(int nb_planes, vector<SegmentedPointsContainer::SegmentedPlane> &planes, vector<float> &surfaces);
    mat3 findRotation();
    mat3 findTranslation();
    void computeMwithNormals();
    void computeMwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT, vector<float> &l_aS, vector<float> &l_aT, vector<float> &angles_cS, vector<float> &angles_cT);
    mat3 computeHwithNormals(vector<vec3> qs, vector<vec3> qt);
    mat3 computeHwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT);
    mat3 computeR(mat3 H);
    vec3 computeCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list, bool isMesh);
    vec3 computeCentersCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list);
    vector<vec3> computeDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid, bool isMesh);
    vector<vec3> computeCentersDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid);
    vector<float> computeAngleDifs(vector<vec3> &l_shifted_centroids, vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);
    vector<float> estimatePlanesSurface(PointNormalKCloud::Ptr p_cloud, vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);
    vector<float> computeCenterAngles(vector<vec3> &l_shifted_centroids);
    float estimatePlaneSurface(PointNormalKCloud::Ptr p_cloud, SegmentedPointsContainer::SegmentedPlane &plane);
    vector<float> computeDelaunaySurfaces(PointNormalKCloud::Ptr p_cloud, vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);
    float computeDelaunaySurface(PointNormalKCloud::Ptr p_cloud, SegmentedPointsContainer::SegmentedPlane &plane);
    vector<vec2> pointsTo2D(PointNormalKCloud::Ptr p_cloud, SegmentedPointsContainer::SegmentedPlane &plane, vec3 e1, vec3 e2);
    void computePlaneBase(SegmentedPointsContainer::SegmentedPlane &plane, vec3 &e1, vec3 &e2);
    vec2 compute2dCentroid(vector<vec2> l_points);
};
