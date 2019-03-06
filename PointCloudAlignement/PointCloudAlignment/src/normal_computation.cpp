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
    float d1(1), d2(4), e(0.1), max_k(50), max_count(10), sigma(0.2);
    int k(15), count(0);

    float r_old, r_new, density, curv;
    pcl::PointXYZ p = cloud_in->points.at(p_id);

    do {
        vector<int> indices;
        vector<float> sqrt_distances;
        kdTree_in->nearestKSearchT(p, k, indices, sqrt_distances);



        count++;
    } while(k < max_k && count < max_count);

    return k;
}
