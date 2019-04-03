#pragma once

#include <omp.h>

#include <pcl/features/normal_3d_omp.h>

#include "common.h"
#include "plane.h"

#define MAX_K 50


class NormalComputation {
public:
    void computeNormalCloud(PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree_in);

private:
    int estimateKForPoint(int p_id, PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree, float &curv);
    float computeCurvature(PointNormalKCloud::Ptr cloud, boost::shared_ptr<vector<int>> indices, vector<float> sqrd_distances, PointNormalK p);
};
