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

APFHCloud PFHEvaluation::computeAPFHSignature(vector<SegmentedPointsContainer::SegmentedPlane> &l_planes)
{
    PointNormalCloud::Ptr cloud = PFHEvaluation::buildPointCloud(l_planes);

    pcl::search::KdTree<PointNormal>::Ptr p_kdTree(new pcl::search::KdTree<PointNormal>());
    p_kdTree->setInputCloud(cloud);
    int K = static_cast<int>(ceil(l_planes.size() * 0.1));

    //Output cloud
    APFHCloud apfh_cloud;

    //For each plane in list:
    for(int i = 0; i < cloud->size(); ++i)
    {
        //  - get K Neighborhood
        vector<int> indices; vector<float> sqr_distances;
        p_kdTree->nearestKSearch(i, K+1, indices, sqr_distances);

        APFHSignature625 apf;
        // Init everything at 0...
        for(size_t j = 0; j < apf.descriptorSize(); ++j)
        {
            apf.histogram[j] = 0;
        }

        // Indices are already sorted by distance order
        for (size_t j = 1; j < indices.size(); ++j)
        {
            //  - compute features for every pair (pi, pj) and angle with pk
            float f1, f2, f3, f4;
            size_t k = j == (indices.size() - 1) ? 1 : j+1;
            PFHEvaluation::computePairAPF(cloud->at(i), cloud->at(j), cloud->at(k), f1, f2, f3, f4);

            //  - Fill Signature histogram
            PFHEvaluation::fillHist(f1, f2, f3, f4, apf);
        }

        //  - Push Signature in cloud
        apfh_cloud.push_back(apf);
    }

    return apfh_cloud;
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

void PFHEvaluation::fillHist(float f1, float f2, float f3, float f4, APFHSignature625 &apf)
{
    // Compute bin number for each feature
    int b1 = PFHEvaluation::getBinIndex(f1);
    int b2 = PFHEvaluation::getBinIndex(f2);
    int b3 = PFHEvaluation::getBinIndex(f3);
    int b4 = PFHEvaluation::getBinIndex(f4);

    // Find flattened bin index
    int i = b1 * pow(NB_BINS_APFH, 3) + b2 * pow(NB_BINS_APFH, 2) + b3 * NB_BINS_APFH + b4;
    apf.histogram[i]++;
}

int PFHEvaluation::getBinIndex(float feature)
{
    // 5 intervals by angle. -Pi -> Pi
    float interval = 2.0 * M_PI / NB_BINS_APFH;

    int index = 0;

    while(feature >= -M_PI + (index * interval))
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
