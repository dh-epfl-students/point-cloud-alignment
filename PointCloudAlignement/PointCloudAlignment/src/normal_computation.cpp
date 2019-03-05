#include "normal_computation.h"

void NormalComputation::computeNormalCloud(PointCloud::Ptr cloud_in, KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree_in, PointCloud::Ptr normals_out)
{
    // resize normals_out.points to size of cloud_in
    normals_out->resize(cloud_in->size());

    // parallel for loop on each point p in cloup_in
    #pragma omp parallel for
    for(int i = 0; i < cloud_in->size(); ++i)
    {
        // compute appropriate K value for current point

        // get K neighborhood indices

        // compute normal
    }
}
