#include "plane.h"

void Plane::setCoeffs(float a, float b, float c, float d) {
    this->a = a;
    this->b = b;
    this->c = c;
    this->d = d;
}

void Plane::setCenter(vec4 p) {
    this->center = p;
}

float Plane::distanceTo(vec3 p) {
    vec3 n(0, 0, 0);
    float d(0);
    this->cartesianToNormal(n, d);

    return fabs(n.dot(p) + d);
}

void Plane::cartesianToNormal(vec3 &n, float &d) {
    vec3 v(a, b, c);
    d = this->d / v.norm();
    n = v.normalized();
}

void Plane::estimatePlane(PointNormalKCloud::Ptr cloud_in, boost::shared_ptr<vector<int>> indices_in, Plane &plane) {
    // Get list of points
    PointNormalKCloud::Ptr cloud_f(new PointNormalKCloud);
    pcl::ExtractIndices<PointNormalK> iFilter(false);
    iFilter.setInputCloud(cloud_in);
    iFilter.setIndices(indices_in);
    iFilter.filter(*cloud_f);

    vec4 center;
    pcl::compute3DCentroid(*cloud_f, center);

    // Compose optimisation matrix
    Eigen::MatrixXf m;
    pcl::demeanPointCloud(*cloud_f, center, m);

    //cout << "Optimisation matrix dimension: " << m.rows() << " " << m.cols() << endl;

    // Compute svd decomposition
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(m.block(0,0, 3, m.cols()), Eigen::ComputeThinU);
    Eigen::MatrixXf u = svd.matrixU();

    //Extract plane parameters
    float a = u(0, 2);
    float b = u(1, 2);
    float c = u(2, 2);

    // Compute plane last parameter
    float d = - a * center.x() - b * center.y() - c * center.z();

    plane.setCoeffs(a, b, c, d);
    plane.setCenter(center);
}
