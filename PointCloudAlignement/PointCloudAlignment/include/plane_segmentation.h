#pragma once

#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/filter_indices.h>

#include <omp.h>

#include "common.h"
#include "normal_computation.h"

#define PHASE1_ITERATIONS 3
#define MIN_STABLE_SIZE 100

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
    bool is_started = false;
    bool is_ready = false;
    bool dont_quit = true;

    function<void(PointNormalKCloud::Ptr)> display_update_callable;

    PointNormalKCloud::Ptr p_cloud;
    KdTreeFlannK::Ptr p_kdtree;
    PointIndices::Ptr p_indices;
    PointIndices::Ptr p_excluded_indices;

    int getRegionGrowingStartLocation();

    void segmentPlane(int start_index);
    void performFirstPhase(int start_index);
    void performSndPhase();
    void performOneStep();
};
