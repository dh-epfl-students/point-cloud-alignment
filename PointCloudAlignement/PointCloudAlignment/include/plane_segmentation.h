#pragma once

#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include "common.h"
#include "normal_computation.h"

class PlaneSegmentation {
public:
    int init(string cloud_file);

    bool isReady();

    void start_pause();

    void stop();

    PointCloud::Ptr getPointCloud();
    NormalCloud::Ptr getNormalCloud();
    KdTreeFlann::Ptr getKdTree();

private:
    bool is_started = false;
    bool is_ready = false;

    PointCloud::Ptr p_cloud;
    NormalCloud::Ptr p_normals;
    KdTreeFlann::Ptr p_kdtree;

    void mainloop();
};
