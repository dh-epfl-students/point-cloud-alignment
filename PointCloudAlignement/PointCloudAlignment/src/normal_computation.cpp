#include "normal_computation.h"

void NormalComputation::computeNormalCloud(PointCloud::Ptr cloud_in, KdTreeFlann::Ptr kdTree_in, NormalCloud::Ptr normals_out)
{
    // resize normals_out.points to size of cloud_in
    normals_out->resize(cloud_in->size());

    // parallel for loop on each point p in cloud_in
    #pragma omp parallel
    {
        #pragma omp for
        for(int i = 0; i < cloud_in->size(); ++i)
        {
            int thread_id = omp_get_thread_num();
            // compute appropriate K value for current point
            int k = estimateKForPoint(i, cloud_in, kdTree_in);
            cout << "Thread " << thread_id << " Point " << i << " with k=" << k << endl;

            // get K neighborhood indices

            // compute normal
        }
    }
}

float NormalComputation::estimateKForPoint(int p_id, PointCloud::Ptr cloud_in, KdTreeFlann::Ptr kdTree_in)
{
    int thread_id = omp_get_thread_num();

    float d1(1), d2(4), e(0.1), max_k(50), max_count(10), sigma(0.2);
    int k(15), count(0);

    float r_new, density, curv;
    Point3 p = cloud_in->points.at(p_id);

    do {
        vector<int> indices;
        vector<float> sqrd_distances;
        kdTree_in->nearestKSearchT(p, k, indices, sqrd_distances);

        cout << "Thread " << thread_id << ": Point " << p_id << " neighborhood size: " << indices.size() << endl;

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

float NormalComputation::computeCurvature(PointCloud::Ptr cloud, vector<int> indices, vector<float> sqrd_distances, Point3 p)
{
    if(indices.size() <= 3) return 0.0f;

    float avgDist(0.0f);
    for_each(sqrd_distances.begin(), sqrd_distances.end(), [&avgDist](const float d)
        {
            avgDist += std::sqrt(d);
        }
    );
    avgDist /= sqrd_distances.size();

    // Estimate best fit plane for the indices found.

}
