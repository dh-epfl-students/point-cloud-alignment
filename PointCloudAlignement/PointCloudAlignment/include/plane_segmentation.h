#pragma once

#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/pfh.h>

#include "common.h"
#include "normal_computation.h"

#define PHASE1_ITERATIONS 3
#define MIN_STABLE_SIZE 100

class PlaneSegmentation {
public:
    int init(string cloud_file);

    void setViewerUpdateCallback(void* callback);

    bool isReady();

    void start_pause();

    void stop();

    PointNormalCloud::Ptr getPointCloud() { return this->p_cloud; }
    KdTreeFlann::Ptr getKdTree() { return this->p_kdtree; }

private:
    bool is_started = false;
    bool is_ready = false;

    PointNormalCloud::Ptr p_cloud;
    KdTreeFlann::Ptr p_kdtree;

    void mainloop();
    int adjustStartLocation();

    void segmentPlane();
    void performFirstPhase();
    void performSndPhase();
    void performOneStep();
};
