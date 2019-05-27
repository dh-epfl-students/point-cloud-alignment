#pragma once

#include "common.h"
#include "segmented_points_container.h"

#define KNN 20
#define NORMAL_ERROR 0.0872665f  // 0.0872665f = 5°
#define DISTANCE_ERROR 0.1f
#define OVERLAP_ANGLE 0.0872665f // 0.0872665f = 5°, 0.174533f = 10°

class PlaneMerging {
public:
    void init(function<void(PointNormalKCloud::Ptr, ivec3, vector<int>, bool)> callable, bool isSource);
    void start_merge(vector<SegmentedPointsContainer::SegmentedPlane> &p_list, PointNormalKCloud::Ptr p_cloud);
    void filter_small_planes(vector<SegmentedPointsContainer::SegmentedPlane> &p_list, int min_size);

    vector<SegmentedPointsContainer::SegmentedPlane> getSegmentedPlanes();
    bool isCloudMerged();

    void applyTransform(mat4 M);

    void printVectorsInFile(string filename);

private:
    vector<SegmentedPointsContainer::SegmentedPlane> plane_list;

    /**
     * @brief PointCloud containing the center of every segmented plane.
     * They must be stored in the same order as the their corresponding plane in plane_list vector,
     * to facilitate their retrieval.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_plane_cloud;
    boost::shared_ptr<vector<int>> p_plane_indices;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr p_kdtree;
    PointNormalKCloud::Ptr p_point_cloud;
    vector<int> merged_planes_indices;
    bool isMerged = false;
    bool isSource = true;

    function<void(PointNormalKCloud::Ptr, ivec3, vector<int>, bool)> display_update_callable;

    void merge();
    bool planeOverlap(SegmentedPointsContainer::SegmentedPlane &p1, SegmentedPointsContainer::SegmentedPlane &p2, float d_tolerance = 0);
    float farestPointInDir(SegmentedPointsContainer::SegmentedPlane &plane, vec3 dir);
    void callDisplayCallback(PointNormalKCloud::Ptr p_cloud, ivec3 c, vector<int> indices, bool isSource);
};
