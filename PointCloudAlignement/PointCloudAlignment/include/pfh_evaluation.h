#pragma once

#include <pcl/features/pfh.h>

#include "common.h"
#include "segmented_points_container.h"

#define PLANE_TRESHOLD 20
#define CENTER_KNN 20

typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;
typedef pcl::PointNormal PointNormal;
typedef pcl::KdTreeFLANN<PointNormal, flann::L2_Simple<float>> KdTreeFlann;

class PFHEvaluation {
public:
    PFHEvaluation(){}
    ~PFHEvaluation(){}

    static bool isValidPlane(PointNormalKCloud::Ptr points, vector<int> &indices_in);

    static pcl::PointCloud<pcl::PFHSignature125> computePFHSignatures(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);

    static size_t getMinTarget(size_t i, pcl::PointCloud<pcl::PFHSignature125> source_signs, pcl::PointCloud<pcl::PFHSignature125> target_signs, vector<size_t> &ignore_list);

private:
};
