#include "pfh_evaluation.h"

bool PFHEvaluation::isValidPlane(PointNormalKCloud::Ptr points, vector<int> &indices)
{
    Eigen::VectorXf pfh_histogram(125);
    pcl::PFHEstimation<PointNormalK, PointNormalK, pcl::PFHSignature125> pfh;
    pfh.computePointPFHSignature(*points, *points, indices, 5, pfh_histogram);

    // Test the histogram for plane
    /*
    cout << "PFHSignature125 dimensions: " << pfh_histogram.rows() << ", " << pfh_histogram.cols() << endl;
    for(int i = 0; i < pfh_histogram.rows(); ++i)
    {
        cout << i << ": " << pfh_histogram[i] << " ";
    }
    cout << endl;
    */

    return pfh_histogram[62] > PLANE_TRESHOLD;
}

PFHCloud PFHEvaluation::computePFHSignatures(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes)
{
    // First, build cloud of points+normals of every planes' centers and normals.
    PointNormalCloud::Ptr cloud = PFHEvaluation::buildPointCloud(l_planes);

    // Fill kdTree to optimise neigboring search
    pcl::search::KdTree<PointNormal>::Ptr p_kdTree(new pcl::search::KdTree<PointNormal>());

    // Output cloud
    PFHCloud pfh_cloud;

    // Compute the signatures
    pcl::PFHEstimation<PointNormal, PointNormal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(cloud);
    pfh.setSearchMethod(p_kdTree);
    pfh.setKSearch(CENTER_KNN);
    pfh.compute(pfh_cloud);

    return pfh_cloud;
}

FPFHCloud PFHEvaluation::computeFPFHSignature(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes)
{
    // First, build cloud of points+normals of every planes' centers and normals.
    PointNormalCloud::Ptr cloud = PFHEvaluation::buildPointCloud(l_planes);

    // Fill kdTree to optimise neigboring search
    pcl::search::KdTree<PointNormal>::Ptr p_kdTree(new pcl::search::KdTree<PointNormal>());

    // Output cloud
    FPFHCloud fpfh_cloud;

    // Compute the signatures
    pcl::FPFHEstimationOMP<PointNormal, PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud);
    fpfh.setSearchMethod(p_kdTree);
    // K is a percentage of number of points in cloud: to test: 5%, 10%, 15%, 20%
    int K = static_cast<int>(ceil(l_planes.size() * 0.1));
    fpfh.setKSearch(K);
    fpfh.compute(fpfh_cloud);

    return fpfh_cloud;
}

PointNormalCloud::Ptr PFHEvaluation::buildPointCloud(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes)
{
    PointNormalCloud::Ptr cloud(new PointNormalCloud);

    for(auto plane: l_planes)
    {
        vec3 c(plane.plane.getCenter());
        vec3 n(plane.plane.getNormalizedN());

        PointNormal pn;
        pn.x = c.x(); pn.y = c.y(); pn.z = c.z();
        pn.normal_x = n.x(); pn.normal_y = n.y(); pn.normal_z = n.z();
        cloud->push_back(pn);
    }

    return cloud;
}

size_t PFHEvaluation::getMinTarget(size_t i, PFHCloud source_signs, PFHCloud target_signs, float &out_error)
{
    size_t j = 0;
    float min_error = numeric_limits<float>::infinity();

    for(size_t it = 0; it < target_signs.points.size(); ++it)
    {

        float curr_error = 0;

        for(int bin_index = 0; bin_index < target_signs.points[it].descriptorSize(); ++bin_index)
        {
            curr_error += abs(source_signs.points[i].histogram[bin_index] - target_signs.points[it].histogram[bin_index]);
        }

        if(curr_error < min_error)
        {
            min_error = curr_error;
            j = it;
        }
        else if (curr_error == min_error) {
            cout << "SAME ERROR VALUE: " << curr_error << endl;
        }
    }

    out_error = min_error;
    return j;
}

int PFHEvaluation::getMinTarget(size_t i, float s_surf, vector<float> &t_surfs, FPFHCloud &source_signs, FPFHCloud &target_signs, float &out_error)
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

    for(size_t it: target_indices)
    {
        float curr_error = PFHEvaluation::computeFPFHError(i, it, source_signs, target_signs);

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

float PFHEvaluation::computeFPFHError(size_t s_id, size_t t_id, FPFHCloud &source_signs, FPFHCloud &target_signs)
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

/*
pcl::PFHSignature125 PFHEvaluation::computePFHSignature(PointNormalCloud::Ptr p_cloud, KdTreeFlann::Ptr p_kdTree, SegmentedPointsContainer::SegmentedPlane &plane)
{
    // Find the neighborhood of current plane center.
    PointNormal p;
    vector<int> indices;
    vector<float> dists;
    p_kdTree->nearestKSearch(plane.plane.getPointNormal(), CENTER_KNN, indices, dists);

    // Compute the signature
    pcl::PFHEstimation<PointNormal, PointNormal, pcl::PFHSignature125> pfh;
    Eigen::VectorXf pfh_histogram(125);
    pfh.computePointPFHSignature(*p_cloud, *p_cloud, indices, 5, pfh_histogram);

    pcl::PFHSignature125 sign;

}*/
