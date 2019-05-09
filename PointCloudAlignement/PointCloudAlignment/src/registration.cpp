#include "registration.h"
#include <cmath>
#include <Eigen/Dense>

void Registration::setClouds(vector<SegmentedPointsContainer::SegmentedPlane> &source, vector<SegmentedPointsContainer::SegmentedPlane> &target, bool isMesh, PointNormalKCloud::Ptr p_cloud)
{
    this->source = source;
    this->target = target;
    this->targetIsMesh = isMesh;
    this->p_cloud = p_cloud;
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
    vector<float> angles_S, angles_T;

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

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            angles_S = computeAngleDifs(l_cS, source);
        }

        #pragma omp section
        {
            angles_T = computeAngleDifs(l_cT, target);
        }
    }

    computeMwithCentroids(l_cS, l_cT, angles_S, angles_T);

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

void Registration::computeMwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT, vector<float> &l_aS, vector<float> &l_aT)
{
    if(l_cT.empty() || l_cS.empty() || l_aS.empty() || l_aT.empty()) return;

    vector<float> surfaces = estimatePlanesSurface(p_cloud, source);

    M.resize(l_cS.size(), l_cT.size());

    //#pragma omp parallel for
    for(size_t i = 0; i < l_cS.size(); ++i)
    {
        float normCSi = l_cS[i].norm();

        for(size_t j = 0; j < l_cT.size(); ++j)
        {
            //M(i, j) = exp(-2 * abs((normCSi - l_cT[j].norm()) * (l_aS[i] - l_aT[j])));
            float m = pow((normCSi - l_cT[j].norm()) * (l_aS[i] - l_aT[j]), 2);
            if(m == 0)
            {
                M(i, j) = 1000000.0f;
            }
            else
            {
                M(i, j) = 1.0f / m;
            }
        }
    }

    //M.normalize();

    vector<mat3::Index> indices;
    Eigen::MatrixXf M_binary = Eigen::MatrixXf::Zero(M.rows(), M.cols());

    for (int i = 0; i < M_binary.rows(); ++i) {
        Eigen::MatrixXf::Index id;
        M.row(i).maxCoeff(&id);

        M_binary(i, id) = 1;
    }

    M = M_binary;
    cout << "M" << endl << M << endl;

    // TEST: Change color of first plane of source and corresponding plan in target to white
    Eigen::MatrixXf::Index id;
    M.row(1).maxCoeff(&id);
    display_update_callable(source[1], target[id], ivec3(255, 255, 255));
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

vector<float> Registration::computeAngleDifs(vector<vec3> &l_shifted_centroids, vector<SegmentedPointsContainer::SegmentedPlane> &l_planes)
{
    vector<float> angles;

    if(l_shifted_centroids.size() != l_planes.size())
    {
        cout << "Computing Angle Diffs: Not same size of vectors... stopping" << endl;
        return angles;
    }

    for(size_t i = 0; i < l_shifted_centroids.size(); ++i)
    {
        // Cosine of angle between ci and ni
        float angle = l_shifted_centroids[i].normalized().dot(l_planes[i].plane.getNormalizedN());
        angles.push_back(angle);
    }

    return angles;
}

vector<float> Registration::estimatePlanesSurface(PointNormalKCloud::Ptr p_cloud, vector<SegmentedPointsContainer::SegmentedPlane> &l_planes)
{
    vector<float> surfaces(l_planes.size());

    #pragma omp parallel for
    for(size_t i = 0; i < l_planes.size(); ++i)
    {
        surfaces[i] = estimatePlaneSurface(p_cloud, l_planes[i]);
    }

    return surfaces;
}

float Registration::estimatePlaneSurface(PointNormalKCloud::Ptr p_cloud, SegmentedPointsContainer::SegmentedPlane &plane)
{
    vec3 e1, e2;
    computePlaneBase(plane, e1, e2);

    vector<vec2> l_2dPoints = pointsTo2D(p_cloud, plane, e1, e2);

    // Estimating plane surface by finding principal directions by doing svd decomposition on congruence matrix formed by points in 2d
    // Compute centroid 2D
    vec2 center = compute2dCentroid(l_2dPoints);

    // Compute covariance matrix
    Eigen::Matrix2Xf cov;
    cov.resize(2, l_2dPoints.size());
    float alpha = 1.0f / (l_2dPoints.size() - 1);

    for(size_t i = 0; i < l_2dPoints.size(); ++i)
    {
        cov.col(i) << alpha * vec2(((l_2dPoints[i] - center).x() * (l_2dPoints[i] - center).x()),
                                  ((l_2dPoints[i] - center).y() * (l_2dPoints[i] - center).y()));
    }

    // SVD Decomposition
    Eigen::JacobiSVD<Eigen::Matrix2Xf> svd(cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix2Xf u = svd.matrixU();
    vec2 s = svd.singularValues();

    // Non correlated variances
    s = s.array().abs().sqrt().matrix();

    // Extract eigenvectors
    vec2 v1 = vec2(u(0, 0), u(1, 0));
    vec2 v2 = vec2(u(0, 1), u(1, 1));

    // Apply scale factor with correction
    v1 = s(0) * v1 * 1.5;
    v2 = s(1) * v2 * 1.5;

    // compute surface estimation
    return s(0) * s(1) * 4 * 3; // ???
}

vector<vec2> Registration::pointsTo2D(PointNormalKCloud::Ptr p_cloud, SegmentedPointsContainer::SegmentedPlane &plane, vec3 e1, vec3 e2)
{
    vec3 center = plane.plane.getCenter();

    vector<vec2> t;

    for(auto i: plane.indices_list)
    {
        vec2 ti;
        ti.x() = e1.dot(pclToVec3(p_cloud->points[i]) - center);
        ti.y() = e2.dot(pclToVec3(p_cloud->points[i]) - center);
        t.push_back(ti);
    }

    return t;
}

void Registration::computePlaneBase(SegmentedPointsContainer::SegmentedPlane &plane, vec3 &e1, vec3 &e2)
{
    vec3 n = plane.plane.getNormalizedN();

    // Build e1 by rotating plane normal by 90 degrees
    e1 = vec3(n.y(), -n.x(), n.z()).normalized();

    // Base should be orthogoal
    e2 = n.cross(e1).normalized();

    cout << "Plane " << plane.id << " : e1: " << e1.transpose() << " e2: " << e2.transpose() << endl;
}

vec2 Registration::compute2dCentroid(vector<vec2> l_points)
{
    vec2 c(0, 0);
    for_each(l_points.begin(), l_points.end(), [&c](vec2 i){
        c += i;
    });
    c /= l_points.size();
    return c;
}
