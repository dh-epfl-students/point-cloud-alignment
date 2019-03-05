#pragma once

#include "common.h"

#include <pcl/features/normal_3d_omp.h>

class NormalComputation {
public:
    void computeNormalCloud(PointCloud::Ptr cloud_in, KdTreeFLANN::Ptr kdTree_in, NormalCloud::Ptr normals_out);
};
