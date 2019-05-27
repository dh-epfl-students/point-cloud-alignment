#pragma once

#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <omp.h>

#include "common.h"
#include "normal_computation.h"
#include "segmented_points_container.h"
#include "pfh_evaluation.h"

#define PHASE1_ITERATIONS 3
#define MIN_STABLE_SIZE 500
#define MIN_PLANE_SIZE 10
#define MAX_ITERATIONS 100

/**
 * @brief Max angle in radians between two normals to be
 * considered in the same plane. Examples: 0.349066f = 20°, 0.261799 = 15°
 */
#define MAX_NORMAL_ANGLE 0.174533f // = 10°


class PlaneSegmentation {
public:
    int init(string cloud_file, bool isSource = true);
    void resetSegmentation() { this->isSegmented = false; }
    void preprocessCloud();
    void setViewerUpdateCallback(function<void(PointNormalKCloud::Ptr, ivec3, vector<int>, bool)> callable);
    void setAddPlaneCallback(function<void(pcl::ModelCoefficients, float, float, float)> callable);
    void setUpdateNormalCloudCallback(function<void(void)> callable);
    float getCurvBound();

    bool isReady();
    bool isCloudSegmented() { return this->isSegmented; }

    void start_pause();

    void runMainLoop();

    void runOneStep();

    void stop();

    void filterOutCurvature(float max_curvature);

    void resampleCloud();

    PointNormalKCloud::Ptr getPointCloud() { return this->p_cloud; }
    KdTreeFlannK::Ptr getKdTree() { return this->p_kdtree; }
    PointNormalKCloud::Ptr getAvailablePointCloud();
    PointNormalKCloud::Ptr getExcludedPointCloud();
    vector<SegmentedPointsContainer::SegmentedPlane> getSegmentedPlanes() { return p_segmented_points_container->getPlanes(); }

    void setPointCloud(PointNormalKCloud::Ptr p_new_cloud) { this->p_cloud = p_new_cloud; }

private:
    /**
     * @brief The algo_RunProperties struct encapsulates
     * all variables kept during the region growing phase.
     */
    typedef struct _RunProperties
    {
        int p_index;
        ivec3 curr_color;
        PointNormalK root_p;
        boost::shared_ptr<vector<int>> p_nghbrs_indices;
        boost::shared_ptr<vector<int>> p_new_points_indices;
        Plane plane;
        int iteration;
        int plane_nb;
        size_t prev_size;
        float max_search_distance;
        float epsilon;

        _RunProperties(): p_index(0), p_nghbrs_indices(new vector<int>(0)),
                            p_new_points_indices(new vector<int>(0)),
                            iteration(0), plane_nb(0), prev_size(0),
                            max_search_distance(0), epsilon(0) {}

        void setupNextPlane(int index, PointNormalK &p, ivec3 color, int plane_id);
        void addToNeighborhood(vector<int> &new_points);
    } RunProperties;

    bool isSource = true;
    bool is_plane_initialized = false;
    bool isResampled = false;
    bool is_started = false;
    bool is_ready = false;
    bool isSegmented = false;
    bool dont_quit = true;

    float safety_distance;
    float curv_bound;

    RunProperties current_run;
    SegmentedPointsContainer::Ptr p_segmented_points_container;

    function<void(PointNormalKCloud::Ptr, ivec3, vector<int>, bool)> display_update_callable;
    function<void(pcl::ModelCoefficients, float, float, float)> add_plane_callable;
    function<void(void)> update_normal_cloud_callable;

    PointNormalKCloud::Ptr p_cloud;
    KdTreeFlannK::Ptr p_kdtree;

    boost::shared_ptr<vector<int>> p_indices;
    boost::shared_ptr<vector<int>> p_excluded_indices;

    void callDisplayCallback(PointNormalKCloud::Ptr p_cloud, ivec3 c, vector<int> indices, bool isSource);

    float getMeanOfMinDistances();
    int getRegionGrowingStartLocation();
    void getNeighborsOf(boost::shared_ptr<vector<int>> indices_in, float search_d, vector<int> &indices_out);

    void segmentPlane();
    bool initRegionGrowth();
    void performRegionGrowth();
    bool regionGrowthOneStep();
    void performOneStep();
    bool planeHasShrinked();
    void reorient_normals(PointNormalKCloud::Ptr cloud_in, vector<int> indices, vec3 pn);
    void exclude_points(vector<int> indices);
    void exclude_from_search(vector<int> &indices);
    void color_points(vector<int> indices, ivec3 color);
    void color_point(int index, ivec3 color);
    void fillSegmentedPointsContainer();
};
