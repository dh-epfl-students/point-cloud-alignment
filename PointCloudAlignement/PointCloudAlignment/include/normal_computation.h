#pragma once

#include <omp.h>

#include <pcl/features/normal_3d_omp.h>

#include "common.h"


class NormalComputation {
public:
    void computeNormalCloud(PointCloud::Ptr cloud_in, KdTreeFlann::Ptr kdTree_in, NormalCloud::Ptr normals_out);

private:
    float estimateKForPoint(int p_id, PointCloud::Ptr cloud_in, KdTreeFlann::Ptr kdTree);
    float computeCurvature(PointCloud::Ptr cloud, vector<int> indices, vector<float> sqrd_distances, Point3 p);
};
