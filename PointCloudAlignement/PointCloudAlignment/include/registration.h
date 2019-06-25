#pragma once

#include <tuple>
#include <cmath>
#include <fstream>
#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/registration/icp.h>

#include "common.h"
#include "segmented_points_container.h"
#include "pfh_evaluation.h"

typedef tuple<size_t, size_t, float> PlaneTuples;

class Registration {
public:
    void setClouds(vector<SegmentedPointsContainer::SegmentedPlane> &source, vector<SegmentedPointsContainer::SegmentedPlane> &target, bool targetIsMesh, bool sourceIsMesh,
                   PointNormalKCloud::Ptr p_source_cloud = nullptr, PointNormalKCloud::Ptr p_target_cloud = nullptr);

    void setCallback(function<void(SegmentedPointsContainer::SegmentedPlane, SegmentedPointsContainer::SegmentedPlane, ivec3)> callable) { this->display_update_callable = callable; }

    mat4 findAlignment();

    mat4 refineAlignment();

    void highlightAssociatedPlanes();

    void applyTransform(mat4 &M);

    /**
     * @brief Apply ICP to refine the initial alignement
     * @return The Transform found by ICP
     */
    mat4 finalICP();

    float getAlignmentError();

    void getCenterClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr p_source_centers, pcl::PointCloud<pcl::PointXYZ>::Ptr p_target_centers);

    vector<tuple<SegmentedPointsContainer::SegmentedPlane, SegmentedPointsContainer::SegmentedPlane> > getSelectedPlanes();
    vector<float> computeDistanceErrors();

private:
    bool targetIsMesh = false;
    bool sourceIsMesh = false;
    Eigen::MatrixXf M;
    mat4 R;
    mat4 T;
    PointNormalKCloud::Ptr p_cloud;
    vector<SegmentedPointsContainer::SegmentedPlane> source;
    vector<SegmentedPointsContainer::SegmentedPlane> target;
    vector<float> source_surfaces;
    vector<float> target_surfaces;
    vector<tuple<size_t, size_t, float>> selected_planes;
    int curr_highlighted_plane = -1;

    function<void(SegmentedPointsContainer::SegmentedPlane, SegmentedPointsContainer::SegmentedPlane, ivec3)> display_update_callable;

    void filterPlanes(int nb_planes, vector<SegmentedPointsContainer::SegmentedPlane> &planes, vector<float> &surfaces);
    mat4 findRotation(matX aggregation_matrix);
    mat4 findAndAddTranslation(mat4 &R);
    void computeMwithNormals();
    void computeMwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT, vector<float> &l_aS, vector<float> &l_aT, vector<float> &angles_cS, vector<float> &angles_cT);
    mat3 computeHwithNormals(vector<vec3> qs, vector<vec3> qt);
    mat3 computeHwithCentroids(matX aggregation_m, vector<vec3> &l_cS, vector<vec3> &l_cT);
    mat3 computeR(mat3 H);
    vec3 computeNormalsCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list, bool isMesh);
    vec3 computeCentersCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list);
    vector<vec3> computeNormalsDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid, bool isMesh);
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

    /**
     * @brief computeMWithPFHSignature
     * @return The matrix M
     */
    Eigen::MatrixXf buildM(vector<PlaneTuples> &l_tuples);
    Eigen::MatrixXf planeTuplesWithPFH(int source_nb);
    vector<PlaneTuples> planeTuplesWithFPFH();

    float computePFHError(size_t i, size_t j, PFHCloud &source, PFHCloud &target);

    vector<size_t> getSortedIndicesGiven(vector<float> &l_surfaces);
    void rotateSourceNormals();
};
