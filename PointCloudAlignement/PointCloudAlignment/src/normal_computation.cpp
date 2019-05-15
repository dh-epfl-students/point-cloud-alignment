#include "normal_computation.h"

void NormalComputation::computeNormalCloud(PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree_in, bool isResampled)
{
    // parallel for loop on each point p in cloud_in
    #pragma omp parallel for
    for(size_t i = 0; i < cloud_in->size(); ++i)
    {
        // compute appropriate K value for current point
        float curv;
        int k = estimateKForPoint(i, cloud_in, kdTree_in, isResampled, curv);

        // get K neighborhood indices
        boost::shared_ptr<vector<int>> indices(new vector<int>);
        vector<float> sqrd_distances;
        kdTree_in->nearestKSearch(cloud_in->points.at(i), k, *indices, sqrd_distances);

        // compute normal
/*
        pcl::NormalEstimationOMP<PointNormalK, PointNormalK> ne;
        vec4 plane_parameters;
        float curvature;
        ne.computePointNormal(*cloud_in, *indices, plane_parameters, curvature);
*/

        // Estimate best fit plane for the indices found.
        Plane plane;
        Plane::estimatePlane(cloud_in, indices, plane);

        vec3 n = plane.getNormal().normalized();
        vec3 up(1, 1, 1);
        up.normalize();

        // Need to reorient normals.
        n = (acos(up.dot(n)) <= acos(up.dot(-n)))? n : -n;

        cloud_in->points[i].normal_x = n.x();//plane_parameters.x(); //n.x();
        cloud_in->points[i].normal_y = n.y();//plane_parameters.y(); //n.y();
        cloud_in->points[i].normal_z = n.z();//plane_parameters.z(); //n.z();
        cloud_in->points[i].curvature = roundTo(curv, 3);
        cloud_in->points[i].k = k;
    }
}

int NormalComputation::estimateKForPoint(int p_id, PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree_in, bool isResampled, float &curv)
{
    float d1(1), d2(4), e(0.1f), max_count(10), sigma(0.2f);
    int k(15), count(0);

    int max_k = isResampled ? MAX_K_RESAMPLED : MAX_K_ORIGINAL;

    float r_new, density;
    PointNormalK p = cloud_in->points.at(p_id);

    do {
        boost::shared_ptr<vector<int>> indices(new vector<int>);
        vector<float> sqrd_distances;
        kdTree_in->nearestKSearch(p, k, *indices, sqrd_distances);

        // Compute density estimation using 
        // the squared distance to farest neighbor found.
        density = k / (M_PI * sqrd_distances.back());

        curv = computeCurvature(cloud_in, indices, sqrd_distances, p);

        r_new = approxR(curv, d1, d2, sigma, e, density);

        k = std::ceil(M_PI * density * r_new * r_new);
        k = std::max(MIN_K, k);
        k = std::fmin(max_k, k);

        count++;
    } while(k < max_k && count < max_count);

    return k;
}

float NormalComputation::computeCurvature(PointNormalKCloud::Ptr cloud, boost::shared_ptr<vector<int>> indices, vector<float> sqrd_distances, PointNormalK p)
{
    if(indices->size() <= 3) return 1.0f;

    float avgDist(0.0f);
    for_each(sqrd_distances.begin(), sqrd_distances.end(), [&avgDist](const float d)
        {
            avgDist += std::sqrt(d);
        }
    );
    avgDist /= sqrd_distances.size();

    // Estimate best fit plane for the indices found.
    Plane plane;
    Plane::estimatePlane(cloud, indices, plane);

    return 2.0f * plane.distanceTo(vec3(p.x, p.y, p.z)) / (avgDist * avgDist);
}
