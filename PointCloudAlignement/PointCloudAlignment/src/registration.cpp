#include "registration.h"
#include <cmath>

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
    //R *= -1;
    return R;
}

mat3 Registration::findRotation()
{

    vec3 cS, cT, nS, nT;
    vector<vec3> l_cS, l_cT, l_nS, l_nT;

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            cS = computeCentersCentroid(source);
            l_cS = computeCentersDifSet(source, cS);
        }

        #pragma omp section
        {
            cT = computeCentersCentroid(target);
            l_cT = computeCentersDifSet(target, cT);
        }

        #pragma omp section
        {
            nS = computeCentroid(source, false);
            l_nS = computeDifSet(source, nS, false);
        }

        #pragma omp section
        {
            nT = computeCentroid(target, targetIsMesh);
            l_nT = computeDifSet(target, nT, targetIsMesh);
        }
    }

    computeMwithCentroids(l_cS, l_cT);

    mat3 H = computeHwithNormals(l_nS, l_nT);
    //mat3 H = computeHWithCentroids(l_cS, l_cT);
    mat3 R = computeR(H);

    /*if(R.determinant() < 0.0f)
    {
        cout << "det of R is less than 0" << endl;
        R.col(2) = -1 * R.col(2);
    }*/

    return R;
}

mat3 Registration::findTranslation()
{
    return mat3::Zero();
}

void Registration::computeMwithNormals()
{
    if(target.empty() || source.empty()) return;

    M.resize(source.size(), target.size());

    #pragma omp parallel for
    for(size_t i = 0; i < source.size(); ++i)
    {
        //TODO change that
        vec3 ni = source[i].plane.getNormalizedN() * source[i].indices_list.size();

        for(size_t j = 0; j < target.size(); ++j)
        {
            vec3 nj = target[j].plane.getNormal();
            if(!targetIsMesh)
            {
                nj = nj.normalized() * target[j].indices_list.size();
            }

            float sqd = pow(ni.norm() - nj.norm(), 2);

            if(sqd == 0.0f)
            {
                M(i, j) = 10000.0f;
            }
            else
            {
                // Works of for small rotations but can get stuck in local minimas
                M(i, j) = 1.0f / sqd;
            }
            //M(i, j) = exp(-0.5*sqd);

        }
    }

    cout << "M:" << endl << M.block(0, 0, 5, 5).matrix() << endl;
}

void Registration::computeMwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT)
{
    if(l_cT.empty() || l_cS.empty()) return;

    M.resize(l_cS.size(), l_cT.size());

    //#pragma omp parallel for
    for(size_t i = 0; i < l_cS.size(); ++i)
    {
        float normCSi = l_cS[i].norm();

        for(size_t j = 0; j < l_cT.size(); ++j)
        {
            M(i, j) = exp(-0.2 * abs(normCSi - l_cT[j].norm()));
            /*float m = pow(normCSi - l_cT[j].norm(), 2);
            if(m == 0)
            {
                M(i, j) = 100000.0f;
            }
            else
            {
                M(i, j) = 1.0f / m;
            }*/
        }
    }

    M.normalize();

    //cout << "M:" << endl << M.block(0, 0, 30, 30);
}

mat3 Registration::computeHwithNormals(vector<vec3> qs, vector<vec3> qt)
{
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

mat3 Registration::computeHwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT)
{
    mat3 H = mat3::Zero();

    for(size_t i = 0; i < l_cS.size(); ++i)
    {
        for(size_t j = 0; j < l_cT.size(); ++j)
        {
            H += M(i, j) * l_cS[i].normalized() * l_cT[j].normalized().transpose();
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
        vec3 n = plane.plane.getNormalizedN();
        /*if(!isMesh)
        {
            n = n.normalized() * plane.indices_list.size();
        }*/
        c += n;
    }

    c /= list.size();
    return c;
}

vector<vec3> Registration::computeDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid, bool isMesh)
{
    vector<vec3> demeaned(list.size());

    #pragma omp parallel for
    for(size_t i = 0; i < list.size(); ++i)
    {
        vec3 n = list[i].plane.getNormalizedN();
        /*if(!isMesh)
        {
            n = n.normalized() * list[i].indices_list.size();
        }*/
        demeaned[i] = n - centroid;
    }

    return demeaned;
}

vec3 Registration::computeCentersCentroid(vector<SegmentedPointsContainer::SegmentedPlane> &list)
{
    vec3 c(0, 0, 0);

    for(auto p: list)
    {
        c += p.plane.getCenter();
    }

    c /= list.size();
    return c;
}

vector<vec3> Registration::computeCentersDifSet(vector<SegmentedPointsContainer::SegmentedPlane> &list, vec3 centroid)
{
    vector<vec3> q(list.size());

    #pragma omp parallel for shared(q)
    for(size_t i = 0; i < list.size(); ++i)
    {
        q[i] = list[i].plane.getCenter() - centroid;
    }

    return q;
}
