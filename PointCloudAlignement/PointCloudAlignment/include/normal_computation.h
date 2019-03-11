#pragma once

#include <omp.h>

#include <pcl/features/normal_3d_omp.h>

#include "common.h"
#include "plane.h"


class NormalComputation {
public:
    void computeNormalCloud(PointNormalCloud::Ptr cloud_in, KdTreeFlann::Ptr kdTree_in);

private:
    float estimateKForPoint(int p_id, PointNormalCloud::Ptr cloud_in, KdTreeFlann::Ptr kdTree);
    float computeCurvature(PointNormalCloud::Ptr cloud, boost::shared_ptr<vector<int>> indices, vector<float> sqrd_distances, Point3N p);
};
