#pragma once

#include <Eigen/Dense>
#include <omp.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>

#include "common.h"

class Plane {
public:
    Plane(float a, float b, float c, float d):a(a), b(b), c(c), d(d){}


    static Plane estimatePlane(PointCloud::Ptr cloud_in, vector<int> indices_in);
private:
    float a, b, c, d;
}

