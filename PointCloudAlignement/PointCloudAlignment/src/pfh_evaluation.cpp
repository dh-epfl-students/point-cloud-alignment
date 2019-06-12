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

void PFHEvaluation::computeFPFHSignature(PointNormalCloud::Ptr p_cloud,
                                              pcl::search::KdTree<PointNormal>::Ptr p_kdTree,
                                              vector<SegmentedPointsContainer::SegmentedPlane> &l_planes,
                                              FPFHCloud::Ptr p_output_cloud)
{
    // Output cloud
    FPFHCloud fpfh_cloud;

    // Compute the signatures
    pcl::FPFHEstimationOMP<PointNormal, PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(p_cloud);
    fpfh.setInputNormals(p_cloud);
    fpfh.setSearchMethod(p_kdTree);
    // K is a percentage of number of points in cloud: to test: 5%, 10%, 15%, 20%
    int K = static_cast<int>(ceil(l_planes.size() * 0.1));
    fpfh.setKSearch(K);
    fpfh.compute(*p_output_cloud);
}

void PFHEvaluation::computeFPFHSignature(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes, FPFHCloud::Ptr p_output_cloud)
{
    // First, build cloud of points+normals of every planes' centers and normals.
    PointNormalCloud::Ptr cloud = PFHEvaluation::buildPointCloud(l_planes);

    // Fill kdTree to optimise neigboring search
    pcl::search::KdTree<PointNormal>::Ptr p_kdTree(new pcl::search::KdTree<PointNormal>());

    PFHEvaluation::computeFPFHSignature(cloud, p_kdTree, l_planes, p_output_cloud);
}

APFHCloud PFHEvaluation::computeAPFHSignature(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes, vector<float> &surfaces)
{
    PointNormalCloud::Ptr cloud = PFHEvaluation::buildPointCloud(l_planes);

    pcl::search::KdTree<PointNormal>::Ptr p_kdTree(new pcl::search::KdTree<PointNormal>());

    // Usual fpfh signatures
    FPFHCloud::Ptr fpfh_cloud(new FPFHCloud);
    PFHEvaluation::computeFPFHSignature(cloud, p_kdTree, l_planes, fpfh_cloud);

    // 4th Feature: Angle between a point and each of it's neighbors.
    FeatureCloud<NB_BINS_APFH>::Ptr f4_cloud(new FeatureCloud<NB_BINS_APFH>);
    // 5th Feature: Distance between a point and each of it's neighbors.
    FeatureCloud<NB_BINS_APFH>::Ptr f5_cloud(new FeatureCloud<NB_BINS_APFH>);

    // Increment constant
    float hist_incr = 100.0f / static_cast<float>(cloud->size () - 1);
    int K = static_cast<int>(ceil(l_planes.size() * 0.1));

    //For each plane in list:
    for(int i = 0; i < cloud->size(); ++i)
    {
        //Initializing 4th and 5th feature
        pcl::Histogram<NB_BINS_APFH> f4_hist;
        pcl::Histogram<NB_BINS_APFH> f5_hist;
        for(int i = 0; i < f4_hist.descriptorSize(); ++i)
        {
            f4_hist.histogram[i] = 0;
            f5_hist.histogram[i] = 0;
        }

        //  - get K Neighborhood
        vector<int> indices; vector<float> sqr_distances;
        p_kdTree->nearestKSearch(i, K+1, indices, sqr_distances);

        // Indices are already sorted by distance order
        for (size_t j = 1; j < indices.size(); ++j)
        {
            //  - compute feature angle between pipj and pipk
            size_t k = j == (indices.size() - 1) ? 1 : j+1;
            float f4 = PFHEvaluation::computeF4(cloud->at(i), cloud->at(j), cloud->at(k));

            // Increment histogram
            int b4 = PFHEvaluation::getBinIndex(f4, 2.0 * M_PI, -M_PI);
            f4_hist.histogram[b4] += hist_incr / surfaces[i];

            int b5 = PFHEvaluation::getBinIndex(sqr_distances[j], sqr_distances.back(), 0);
            f5_hist.histogram[b5] += hist_incr / surfaces[i];
        }

        f4_cloud->push_back(f4_hist);
        f5_cloud->push_back(f5_hist);
    }

    // Concatenate both histograms
    APFHCloud apfh_cloud;
    for (size_t i = 0; i < l_planes.size(); ++i)
    {
        APFHSignature apfh = PFHEvaluation::concatenateHists(fpfh_cloud->points[i], f4_cloud->points[i], f5_cloud->points[i]);
        apfh_cloud.push_back(apfh);
    }
    return apfh_cloud;
}

APFHSignature PFHEvaluation::concatenateHists(pcl::FPFHSignature33 &fpfh, pcl::Histogram<NB_BINS_APFH> &f4h, pcl::Histogram<NB_BINS_APFH> &f5h)
{
    pcl::Histogram<NB_BINS_APFH * NB_FEATURES_APFH> apfh;
    for(size_t i = 0; i < fpfh.descriptorSize(); ++i)
    {
        apfh.histogram[i] = fpfh.histogram[i];
    }
    for(size_t i = 0; i < f4h.descriptorSize(); ++i)
    {
        apfh.histogram[i + fpfh.descriptorSize()] = f4h.histogram[i];
    }
    for(size_t i = 0; i < f5h.descriptorSize(); ++i)
    {
        apfh.histogram[i + fpfh.descriptorSize() + f4h.descriptorSize()] = f5h.histogram[i];
    }

    return apfh;
}

float PFHEvaluation::computeF4(PointNormal &pi, PointNormal &pj, PointNormal &pk)
{
    vec4 p1 = pointToVec4(pi);
    vec4 p2 = pointToVec4(pj);
    vec4 p3 = pointToVec4(pk);

    // Angle between pipj and pipk
    vec4 t = p2 - p1;
    vec4 s = p3 - p1;
    vec3 t1 = vec3(t.x(), t.y(), t.z()).normalized();
    vec3 t2 = vec3(s.x(), s.y(), s.z()).normalized();
    float f4 = acos(t1.dot(t2));
    return f4;
}

void PFHEvaluation::computePairAPF(PointNormal &pi, PointNormal &pj, PointNormal &pk, float &f1, float &f2, float &f3, float &f4)
{
    vec4 p1 = pointToVec4(pi);
    vec4 p2 = pointToVec4(pj);
    vec4 n1 = normalToVec4(pi);
    vec4 n2 = normalToVec4(pj);
    vec4 p3 = pointToVec4(pk);

    float d;
    pcl::computePairFeatures(p1, n1, p2, n2, f1, f2, f3, d);


    // DEBUG print to see angles
    //cout << f1 << " " << f2 << " " << f3 << endl;

    // Angle between pipj and pipk
    vec4 t = p2 - p1;
    vec4 s = p3 - p1;
    vec3 t1 = vec3(t.x(), t.y(), t.z()).normalized();
    vec3 t2 = vec3(s.x(), s.y(), s.z()).normalized();
    f4 = acos(t1.dot(t2));
}

void PFHEvaluation::fillHist(float f1, float f2, float f3, float f4, APFHSignature &apf)
{
    // Compute bin number for each feature
    int b1 = PFHEvaluation::getBinIndex(f1, 2.0 * M_PI, -M_PI);
    int b2 = PFHEvaluation::getBinIndex(f2, 2.0 * M_PI, -M_PI);
    int b3 = PFHEvaluation::getBinIndex(f3, 2.0 * M_PI, -M_PI);
    int b4 = PFHEvaluation::getBinIndex(f4, 2.0 * M_PI, -M_PI);

    // Find flattened bin index
    int i = b1 * pow(NB_BINS_APFH, 3) + b2 * pow(NB_BINS_APFH, 2) + b3 * NB_BINS_APFH + b4;
    apf.histogram[i]++;
}

int PFHEvaluation::getBinIndex(float feature, float interval_size, float interval_lb)
{
    // NB_BINS intervals by angle.
    float interval = interval_size / NB_BINS_APFH;

    int index = 0;

    while(feature >= interval_lb + (index * interval))
    {
        ++index;
    }

    return index - 1;
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


//template<int N>
//int PFHEvaluation::getMinTarget(size_t i, float s_surf, vector<float> &t_surfs, FeatureCloud<N> &source_signs, FeatureCloud<N> &target_signs, float &out_error)
//{
//    int j = -1;
//    float min_error = numeric_limits<float>::infinity();

//    // Get subset of target planes that are in surface interval of source surface
//    vector<size_t> target_indices;
//    for(size_t t_id = 0; t_id < t_surfs.size(); ++t_id)
//    {
//        if(abs(s_surf - t_surfs[t_id]) < SURFACE_INTERVAL)
//        {
//            target_indices.push_back(t_id);
//        }
//    }

//    for(size_t it: target_indices)
//    {
//        float curr_error = PFHEvaluation::computeFPFHError(i, it, source_signs, target_signs);

//        if(curr_error < min_error /*&& abs(s_surf - t_surfs[j]) > abs(s_surf - t_surfs[it])*/)
//        {
//            min_error = curr_error;
//            j = it;
//        }
//        else if (curr_error == min_error && (abs(s_surf - t_surfs[j]) > abs(s_surf - t_surfs[it]))) {
//            // In case of same error, keep target plane that has the nearest surface
//            min_error = curr_error;
//            j = it;
//        }
//    }

//    out_error = min_error;
//    return j;
//}

//template<int N>
//float PFHEvaluation::computeFeatureError(size_t s_id, size_t t_id, FeatureCloud<N> &source_signs, FeatureCloud<N> &target_signs)
//{
//    float error = 0;

//    auto s_bin = source_signs.points[s_id];
//    auto t_bin = target_signs.points[t_id];

//    for(int i = 0; i < s_bin.descriptorSize(); ++i)
//    {
//        error += abs(s_bin.histogram[i] - t_bin.histogram[i]);
//        //error = max(error, abs(s_bin.histogram[i] - t_bin.histogram[i]));
//    }

//    return error;
//}
