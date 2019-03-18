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
#define MIN_STABLE_SIZE 60

class PlaneSegmentation {
public:
    int init(string cloud_file);

    void setViewerUpdateCallback(function<void(PointNormalKCloud::Ptr)> callable);

    bool isReady();

    void start_pause();

    void runMainLoop();

    void stop();

    PointNormalKCloud::Ptr getPointCloud() { return this->p_cloud; }
    KdTreeFlannK::Ptr getKdTree() { return this->p_kdtree; }

private:
    /**
     * @brief The algo_RunProperties struct encapsulates
     * all variables kept during the region growing phase.
     */
    typedef struct _RunProperties
    {
        PointNormalK root_p;
        PointIndices::Ptr p_nghbrs_indices;
        PointIndices::Ptr p_new_points_indices;
        Plane plane;
        int iteration;
        int plane_nb;
        int prev_size;
        float max_search_distance;
        float epsilon;
        vec3 n;

        _RunProperties(): p_nghbrs_indices(new PointIndices),
                            p_new_points_indices(new PointIndices),
                            iteration(0), plane_nb(0), prev_size(0),
                            max_search_distance(0), epsilon(0), n(0, 0, 0) {}

        inline void setupNextPlane(PointNormalK &p)
        {
            plane = Plane();
            root_p = p;
            plane_nb++;
            iteration = 0;
            prev_size = 0;
            max_search_distance = 0;
            epsilon = 0;
            p_nghbrs_indices->indices.clear();
            p_new_points_indices->indices.clear();
            n = vec3(0, 0, 0);
        }
    } RunProperties;

    bool is_started = false;
    bool is_ready = false;
    bool dont_quit = true;

    float safety_distance;

    RunProperties current_run;
    SegmentedPointsContainer segmented_points_container;

    function<void(PointNormalKCloud::Ptr)> display_update_callable;

    PointNormalKCloud::Ptr p_cloud;
    KdTreeFlannK::Ptr p_kdtree;
    PointIndices::Ptr p_indices;
    PointIndices::Ptr p_excluded_indices;

    float getMeanOfMinDistances();
    int getRegionGrowingStartLocation();
    void getNeighborsOf(PointIndices::Ptr indices_in, float search_d, vector<int> indices_out);

    void segmentPlane();
    bool initRegionGrowth();
    void performRegionGrowth();
    void performOneStep();
    void stop_current_plane_segmentation();
};
