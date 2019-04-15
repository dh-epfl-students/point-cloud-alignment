#pragma once

#include <Eigen/Dense>
#include <omp.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/ModelCoefficients.h>

#include "common.h"

class Plane;

class Plane {
public:
    Plane(): a(0), b(0), c(0), d(0), center(0,0,0,0) {}
    Plane(float a, float b, float c, float d):a(a), b(b), c(c), d(d){}
    Plane(const Plane &p): a(p.a), b(p.b), c(p.c), d(p.d) {}

    void setCoeffs(float a, float b, float c, float d);
    void setCenter(vec4 p);
    float distanceTo(PointNormalK p);
    float distanceTo(vec3 p);
    void cartesianToNormal(vec3 &n, float &di);
    vec3 getNormal();
    pcl::ModelCoefficients getModelCoefficients();
    float getPlaneTolerance(PointNormalKCloud::Ptr cloud, boost::shared_ptr<vector<int>> indices);
    bool pointInPlane(PointNormalK p, float epsilon);
    bool normalInPlane(PointNormalK p, float max_angle);
    vec4 getCenter();

    static void estimatePlane(PointNormalKCloud::Ptr cloud_in, boost::shared_ptr<vector<int> > indices_in, Plane &plane);
private:
    float a, b, c, d;
    vec4 center;
};

