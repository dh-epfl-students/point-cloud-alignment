#pragma once

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>

#include "common.h"
#include "segmented_points_container.h"

#define PLANE_TRESHOLD 20
#define CENTER_KNN 20
#define SURFACE_INTERVAL 200

typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;
typedef pcl::PointNormal PointNormal;
typedef pcl::KdTreeFLANN<PointNormal, flann::L2_Simple<float>> KdTreeFlann;

class PFHEvaluation {
public:
    PFHEvaluation(){}
    ~PFHEvaluation(){}

    static bool isValidPlane(PointNormalKCloud::Ptr points, vector<int> &indices_in);

    static PFHCloud computePFHSignatures(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);

    static FPFHCloud computeFPFHSignature(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);

    static size_t getMinTarget(size_t i, PFHCloud source_signs, PFHCloud target_signs, float &out_error);

    static int getMinTarget(size_t i, float s_surf, vector<float> &t_surfs, FPFHCloud &source_signs, FPFHCloud &target_signs, float &out_error);

    static float computeFPFHError(size_t s_id, size_t t_id, FPFHCloud &source_signs, FPFHCloud &target_signs);

private:
    static PointNormalCloud::Ptr buildPointCloud(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);
};
