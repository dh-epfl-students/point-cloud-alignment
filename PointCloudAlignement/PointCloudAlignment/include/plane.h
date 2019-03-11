#pragma once

#include <Eigen/Dense>
#include <omp.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>

#include "common.h"

class Plane;

class Plane {
public:
    Plane(){}
    Plane(float a, float b, float c, float d):a(a), b(b), c(c), d(d){}

    void setCoeffs(float a, float b, float c, float d);
    void setCenter(vec4 p);
    float distanceTo(vec3 p);
    void cartesianToNormal(vec3 &n, float &d);

    static void estimatePlane(PointNormalCloud::Ptr cloud_in, boost::shared_ptr<vector<int> > indices_in, Plane &plane);
private:
    float a, b, c, d;
    vec4 center;
};

