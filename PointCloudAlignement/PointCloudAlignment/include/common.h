#pragma once

#include <functional>

#include <Eigen/Core>

#include "point_normal_k.h"

using namespace std;

typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;
typedef Eigen::Vector4f vec4;
typedef Eigen::Vector3d vec3d;
typedef Eigen::Matrix4f mat4;
typedef Eigen::Matrix3f mat3;

typedef pcl::PointIndices PointIndices;

inline float squaredDistance(vec3 p1, vec3 p2) {
    vec3 p = p1 - p2;
    return p.dot(p);
}

inline float distance(vec3 p1, vec3 p2) {
    return std::sqrt(squaredDistance(p1, p2));
}

inline std::vector<uint> intersect_sets(std::vector<uint> &v1, std::vector<uint> &v2) {
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());

    std::vector<uint> vOut;
    std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(vOut));

    return vOut;
}

inline float approxR(float curv, float d1, float d2, float sigma, float epsilon, float density) {
    float sig2 = sigma * sigma;
    float left = d1 * sigma / std::sqrt(epsilon * density);
    float right = d2 * sig2;
    float r = std::pow(1.0f / curv * (left + right), 1.0f/3.0f);
    return r;
}
