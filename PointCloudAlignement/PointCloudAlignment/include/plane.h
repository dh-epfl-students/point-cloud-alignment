#pragma once

#include <Eigen/Dense>
#include <omp.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>

#include "common.h"

class Plane;

class Plane {
public:
    Plane(): a(0), b(0), c(0), d(0), center(0,0,0,0) {}
    Plane(float a, float b, float c, float d):a(a), b(b), c(c), d(d){}

    void setCoeffs(float a, float b, float c, float d);
    void setCenter(vec4 p);
    float distanceTo(PointNormalK p);
    float distanceTo(vec3 p);
    void cartesianToNormal(vec3 &n, float &d);
    vec3 getNormal();
    float getStdDevWith(PointNormalKCloud::Ptr cloud, PointIndices::Ptr indices);
    bool pointInPlane(PointNormalK p, float epsilon);
    bool normalInPlane(PointNormalK p, float max_angle);

    static void estimatePlane(PointNormalKCloud::Ptr cloud_in, boost::shared_ptr<vector<int> > indices_in, Plane &plane);
private:
    float a, b, c, d;
    vec4 center;
};

