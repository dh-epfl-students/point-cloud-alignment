#include "registration.h"

void Registration::setClouds(vector<SegmentedPointsContainer::SegmentedPlane> &source, vector<SegmentedPointsContainer::SegmentedPlane> &target, bool isMesh)
{
    this->source = source;
    this->target = target;
    this->targetIsMesh = isMesh;
}

mat3 Registration::findAlignment()
{
    if(target.empty() || source.empty()) return mat3::Identity();

    mat3 R = findRotation();
    R *= -1;

    return R;
}

mat3 Registration::findRotation()
{
    computeM();
    mat3 H = computeH();
    return computeR(H);
}

mat3 Registration::findTranslation()
{
    return mat3::Zero();
}

void Registration::computeM()
{
    if(target.empty() || source.empty()) return;

    M.resize(source.size(), target.size());

    #pragma omp parallel for
    for(size_t i = 0; i < source.size(); ++i)
    {
        vec3 ni = source[i].plane.getNormal() * source[i].indices_list.size();

        for(size_t j = 0; j < target.size(); ++j)
        {
            vec3 nj = target[j].plane.getNormal();
            if(!targetIsMesh && nj.norm() == 1.0f) nj *= target[j].indices_list.size();
            M(i, j) = squaredDistance(ni, nj);
        }
    }
}

mat3 Registration::computeH()
{
    // Compute centroids
    vec3 cs = computeCentroid(source, false);
    vec3 ct = computeCentroid(target, targetIsMesh);

    // Compute difference sets
    vector<vec3> qs = computeDifSet(source, cs, false);
    vector<vec3> qt = computeDifSet(target, ct, targetIsMesh);

    mat3 H = mat3::Zero();

    for(int i = 0; i < qs.size(); ++i)
    {
        for(int j = 0; j < qt.size(); ++j)
        {
            H += M(i, j) * qs[i] * qt[j].transpose();
        }
    }

    return H;
}

mat3 Registration::computeR(mat3 H)
{
    Eigen::JacobiSVD<mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    mat3 u = svd.matrixU();
    mat3 v = svd.matrixV();
    return v * u.transpose();
}

vec3 Registration::computeCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list, bool isMesh)
{
    vec3 c = vec3::Zero();

    for(auto plane: list)
    {
        vec3 n = plane.plane.getNormal();
        if(!isMesh && (n.norm() == 1.0f)) n *= plane.indices_list.size();
        c += n;
    }

    c /= list.size();
    return c;
}

vector<vec3> Registration::computeDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid, bool isMesh)
{
    vector<vec3> demeaned(list.size());

    #pragma omp parallel for
    for(int i = 0; i < list.size(); ++i)
    {
        vec3 n = list[i].plane.getNormal();
        if(!isMesh && (n.norm() == 1.0f)) n *= list[i].indices_list.size();
        demeaned[i] = n - centroid;
    }

    return demeaned;
}
