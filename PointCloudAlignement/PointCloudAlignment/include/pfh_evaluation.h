#pragma once

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>

#include "common.h"
#include "segmented_points_container.h"

typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;
typedef pcl::PointNormal PointNormal;
typedef pcl::KdTreeFLANN<PointNormal, flann::L2_Simple<float>> KdTreeFlann;

template<int N> using FeatureCloud = pcl::PointCloud<pcl::Histogram<N>>;

class PFHEvaluation {
public:
    PFHEvaluation(){}
    ~PFHEvaluation(){}

    static bool isValidPlane(PointNormalKCloud::Ptr points, vector<int> &indices_in);

    static PFHCloud computePFHSignatures(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);

    static void computeFPFHSignature(PointNormalCloud::Ptr p_cloud,
                                     pcl::search::KdTree<PointNormal>::Ptr p_kdTree,
                                     vector<SegmentedPointsContainer::SegmentedPlane> &l_planes,
                                     FPFHCloud::Ptr p_output_cloud);

    static void computeFPFHSignature(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes, FPFHCloud::Ptr p_output_cloud);

    static APFHCloud computeAPFHSignature(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes, vector<float> &surfaces);

    static size_t getMinTarget(size_t i, PFHCloud source_signs, PFHCloud target_signs, float &out_error);

    template<int N>
    static int getMinTarget(size_t i, float s_surf, vector<float> &t_surfs, FeatureCloud<N> &source_signs, FeatureCloud<N> &target_signs, float &out_error);

    template<int N>
    static float computeFeatureError(size_t s_id, size_t t_id, FeatureCloud<N> &source_signs, FeatureCloud<N> &target_signs);

private:
    static PointNormalCloud::Ptr buildPointCloud(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes);

    static float computeF4(PointNormal &pi, PointNormal &pj, PointNormal &pk);

    static APFHSignature concatenateHists(pcl::FPFHSignature33 &fpfh, pcl::Histogram<NB_BINS_APFH> &f4h, pcl::Histogram<NB_BINS_APFH> &f5h);

    static void computePairAPF(PointNormal &pi, PointNormal &pj, PointNormal &pk, float &f1, float &f2, float &f3, float &f4);

    static void fillHist(float f1, float f2, float f3, float f4, APFHSignature &apf);

    static int getBinIndex(float feature, float interval_size, float interval_lb);
};

// IMPLEMENTATIONS OF TEMPLATE FUNCTIONS

template<int N>
int PFHEvaluation::getMinTarget(size_t i, float s_surf, vector<float> &t_surfs, FeatureCloud<N> &source_signs, FeatureCloud<N> &target_signs, float &out_error)
{
    int j = -1;
    float min_error = numeric_limits<float>::infinity();

    // Get subset of target planes that are in surface interval of source surface
    vector<size_t> target_indices;
    for(size_t t_id = 0; t_id < t_surfs.size(); ++t_id)
    {
        if(abs(s_surf - t_surfs[t_id]) < SURFACE_INTERVAL)
        {
            target_indices.push_back(t_id);
        }
    }

    // Choose best target plane
    for(size_t it: target_indices)
    {
        float curr_error = PFHEvaluation::computeFeatureError(i, it, source_signs, target_signs);

        if(curr_error < min_error /*&& abs(s_surf - t_surfs[j]) > abs(s_surf - t_surfs[it])*/)
        {
            min_error = curr_error;
            j = it;
        }
        else if (curr_error == min_error && (abs(s_surf - t_surfs[j]) > abs(s_surf - t_surfs[it]))) {
            // In case of same error, keep target plane that has the nearest surface
            min_error = curr_error;
            j = it;
        }
    }

    out_error = min_error;
    return j;
}

template<int N>
float PFHEvaluation::computeFeatureError(size_t s_id, size_t t_id, FeatureCloud<N> &source_signs, FeatureCloud<N> &target_signs)
{
    float error = 0;

    auto s_bin = source_signs.points[s_id];
    auto t_bin = target_signs.points[t_id];

    for(int i = 0; i < s_bin.descriptorSize(); ++i)
    {
        error += abs(s_bin.histogram[i] - t_bin.histogram[i]);
        //error = max(error, abs(s_bin.histogram[i] - t_bin.histogram[i]));
    }

    return error;
}
