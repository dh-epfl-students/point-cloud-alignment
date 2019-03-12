#pragma once

#include <omp.h>

#include <pcl/features/normal_3d_omp.h>

#include "common.h"
#include "plane.h"


class NormalComputation {
public:
    void computeNormalCloud(PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree_in);

private:
    float estimateKForPoint(int p_id, PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree);
    float computeCurvature(PointNormalKCloud::Ptr cloud, boost::shared_ptr<vector<int>> indices, vector<float> sqrd_distances, PointNormalK p);
};
