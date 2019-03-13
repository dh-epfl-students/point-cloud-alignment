#include "normal_computation.h"

void NormalComputation::computeNormalCloud(PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree_in)
{
    // parallel for loop on each point p in cloud_in
    #pragma omp parallel for schedule(dynamic)
    for(int i = 0; i < cloud_in->size(); ++i)
    {
        // compute appropriate K value for current point
        int k = estimateKForPoint(i, cloud_in, kdTree_in);

        // get K neighborhood indices
        vector<int> indices;
        vector<float> sqrd_distances;
        kdTree_in->nearestKSearch(cloud_in->points.at(i), k, indices, sqrd_distances);

        // compute normal
        pcl::NormalEstimationOMP<PointNormalK, pcl::Normal> ne;
        vec4 plane_parameters;
        float curvature;
        ne.computePointNormal(*cloud_in, indices, plane_parameters, curvature);

        PointNormalK pn = PointNormalK(cloud_in->points[i]);
        pn.normal_x = plane_parameters.x();
        pn.normal_y = plane_parameters.y();
        pn.normal_z = plane_parameters.z();
        pn.curvature = curvature;
        pn.k = k;

        cloud_in->points[i] = pn;
    }
}

float NormalComputation::estimateKForPoint(int p_id, PointNormalKCloud::Ptr cloud_in, KdTreeFlannK::Ptr kdTree_in)
{
    float d1(1), d2(4), e(0.1), max_k(50), max_count(10), sigma(0.2);
    int k(15), count(0);

    float r_new, density, curv;
    PointNormalK p = cloud_in->points.at(p_id);

    do {
        boost::shared_ptr<vector<int>> indices(new vector<int>);
        vector<float> sqrd_distances;
        kdTree_in->nearestKSearchT(p, k, *indices, sqrd_distances);

        // Compute density estimation using 
        // the squared distance to farest neighbor found.
        density = k / (M_PI * sqrd_distances.back());

        curv = computeCurvature(cloud_in, indices, sqrd_distances, p);

        r_new = approxR(curv, d1, d2, sigma, e, density);

        k = std::ceil(M_PI * density * r_new * r_new);
        k = std::max(10, k);
        k = std::min(50, k);

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
