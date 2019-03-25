#pragma once

#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/filter_indices.h>

#include <omp.h>

#include "common.h"
#include "normal_computation.h"
#include "segmented_points_container.h"
#include "pfh_evaluation.h"

#define PHASE1_ITERATIONS 3
#define MIN_STABLE_SIZE 100

class PlaneSegmentation {
public:
    int init(string cloud_file);

    void setViewerUpdateCallback(function<void(PointNormalKCloud::Ptr, ivec3, vector<int>)> callable);
    void setAddPlaneCallback(function<void(pcl::ModelCoefficients, float, float, float)> callable);

    bool isReady();

    void start_pause();

    void runMainLoop();

    void runOneStep();

    void stop();

    PointNormalKCloud::Ptr getPointCloud() { return this->p_cloud; }
    KdTreeFlannK::Ptr getKdTree() { return this->p_kdtree; }
    //Octree::Ptr getOctree() { return this->p_octree; }

private:
    /**
     * @brief The algo_RunProperties struct encapsulates
     * all variables kept during the region growing phase.
     */
    typedef struct _RunProperties
    {
        int p_index;
        PointNormalK root_p;
        boost::shared_ptr<vector<int>> p_nghbrs_indices;
        boost::shared_ptr<vector<int>> p_new_points_indices;
        Plane plane;
        int iteration;
        int plane_nb;
        size_t prev_size;
        float max_search_distance;
        float epsilon;
        vec3 n;

        _RunProperties(): p_index(0), p_nghbrs_indices(new vector<int>(0)),
                            p_new_points_indices(new vector<int>(0)),
                            iteration(0), plane_nb(0), prev_size(0),
                            max_search_distance(0), epsilon(0), n(0, 0, 0) {}

        void setupNextPlane(int index, PointNormalK &p);
        void addToNeighborhood(vector<int> &new_points);
    } RunProperties;

    bool is_plane_initialized = false;
    bool is_started = false;
    bool is_ready = false;
    bool dont_quit = true;

    float safety_distance;

    /**
     * @brief Max angle in radians between two normals to be
     * considered in the same plane
     */
    float max_normal_angle = 0.349066f; // = 20°

    RunProperties current_run;
    SegmentedPointsContainer segmented_points_container;

    function<void(PointNormalKCloud::Ptr, ivec3, vector<int>)> display_update_callable;
    function<void(pcl::ModelCoefficients, float, float, float)> add_plane_callable;

    PointNormalKCloud::Ptr p_cloud;
    KdTreeFlannK::Ptr p_kdtree;
    //Octree::Ptr p_octree;
    boost::shared_ptr<vector<int>> p_indices;
    boost::shared_ptr<vector<int>> p_excluded_indices;

    float getMeanOfMinDistances();
    int getRegionGrowingStartLocation();
    void getNeighborsOf(boost::shared_ptr<vector<int>> indices_in, float search_d, vector<int> &indices_out);

    void segmentPlane();
    bool initRegionGrowth();
    void performRegionGrowth();
    bool regionGrowthOneStep();
    void performOneStep();
    void stop_current_plane_segmentation();
    void exclude_points(vector<int> indices);
    void color_points(vector<int> indices, ivec3 color);
    void color_point(int index, ivec3 color);
};
