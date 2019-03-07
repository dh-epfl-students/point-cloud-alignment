#include "plane.h"

static Plane Plane::estimatePlane(PointCloud::Ptr cloud_in, vector<int> indices_in) {
    // Get list of points
    PointCloud::Ptr cloud_f(new PointCloud);
    pcl::ExtractIndices<Point3> iFilter();
    iFilter.setInputCloud(cloud_in);
    iFilter.setIndices(&indices);
    iFilter.filter(*cloud_f);

    vec4 center;
    pcl::compute3DCentroid(*cloud_f, center);

    Eigen::MatrixXf m;
    pcl::demeanPointCloud(*cloud_f, center, m)

    // Compute centroid


    // Compose optimisation matrix


}
