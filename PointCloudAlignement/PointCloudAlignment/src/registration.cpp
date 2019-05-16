#include "registration.h"

void Registration::filterPlanes(int nb_planes, vector<SegmentedPointsContainer::SegmentedPlane> &planes, vector<float> &surfaces)
{
    //Build list of available indices that will be sorted in decreasing surface order
    vector<size_t> id_list;
    for(size_t i = 0; i < planes.size(); ++i)
    {
        id_list.push_back(i);
    }

    // Sort index list by decreasing surface order
    sort(id_list.begin(), id_list.end(), [&surfaces](size_t i, size_t j){
        return surfaces[i] > surfaces[j];
    });

    vector<SegmentedPointsContainer::SegmentedPlane> new_planes;
    vector<float> new_surfaces;

    for(size_t i = 0; i < nb_planes; ++i)
    {
        new_planes.push_back(planes[id_list[i]]);
        new_surfaces.push_back(surfaces[id_list[i]]);
    }

    planes.swap(new_planes);
    surfaces.swap(new_surfaces);
}

void Registration::setClouds(vector<SegmentedPointsContainer::SegmentedPlane> &source, vector<SegmentedPointsContainer::SegmentedPlane> &target, bool targetIsMesh, bool sourceIsMesh,
                             PointNormalKCloud::Ptr p_source_cloud, PointNormalKCloud::Ptr p_target_cloud)
{
    this->source = source;
    this->target = target;
    this->targetIsMesh = targetIsMesh;
    this->sourceIsMesh = sourceIsMesh;
    this->p_cloud = p_source_cloud;

    // Compute surfaces if surface vectors are empty
    if(this->source_surfaces.empty())
    {
        if(sourceIsMesh)
        {
            // Compute mesh planes surface
            for(auto s: source)
            {
                this->source_surfaces.push_back(s.plane.getNormal().norm());
            }
        }
        else
        {
            this->source_surfaces = computeDelaunaySurfaces(p_source_cloud, source);//estimatePlanesSurface(p_source_cloud, source);
        }


        // Filter out planes to keep only MIN_SURFACE biggest planes for each set
        //filterPlanes(MIN_SURFACE, this->source, source_surfaces);
    }

    if(this->target_surfaces.empty())
    {
        if(targetIsMesh)
        {
            // Compute mesh planes surface
            for(auto t: target)
            {
                target_surfaces.push_back(t.plane.getNormal().norm());
            }
        }
        else
        {
            this->target_surfaces = computeDelaunaySurfaces(p_target_cloud, target);//estimatePlanesSurface(p_target_cloud, target);
        }

        // Filter out planes to keep only MIN_SURFACE biggest planes for each set
        //filterPlanes(MIN_SURFACE, this->target, target_surfaces);
    }
}

mat3 Registration::findAlignment()
{
    if(target.empty() || source.empty()) return mat3::Identity();

    mat3 R = findRotation();
    mat4 T = findTranslation();

    cout << "T: " << endl << T << endl;

    //R *= -1;
    return R;
}

mat3 Registration::findRotation()
{
    vec3 cS, cT, nS, nT;
    vector<vec3> l_cS, l_cT, l_nS, l_nT;
    vector<float> angles_S, angles_T;
    vector<float> angles_cS, angles_cT;

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
            nS = computeCentroid(source, sourceIsMesh);
            l_nS = computeDifSet(source, nS, sourceIsMesh);
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

        #pragma omp section
        {
            angles_cS = computeCenterAngles(l_cS);
        }

        #pragma omp section
        {
            angles_cT = computeCenterAngles(l_cT);
        }
    }

    //computeMwithCentroids(l_cS, l_cT, angles_S, angles_T, angles_cS, angles_cT);

    M = computeMWithPFHSignature(MAX_SOURCE_PLANES);

    // Write M to file
    string filename("myMatrixMFile.txt");
    ofstream myFile;
    myFile.open(filename);
    myFile << M << endl;
    myFile.close();

    //mat3 H = computeHwithNormals(l_nS, l_nT);
    mat3 H = computeHwithCentroids(l_cS, l_cT);
    mat3 R = computeR(H);

    if(R.determinant() < 0.0f)
    {
        cout << "det of R is less than 0" << endl;
        //vec4 eigv = R.eigenvalues();

        R.col(1) = -1 * R.col(1);
    }

    return R;
}

mat4 Registration::findTranslation()
{
    // Compute the mean of centers from selected corresponding source and target planes
    vec3 source_cmean(0, 0, 0);
    vec3 target_cmean(0, 0, 0);

    for(auto t: this->selected_planes)
    {
        auto i = get<0>(t);
        auto j = get<1>(t);

        source_cmean += source[i].plane.getCenter();
        target_cmean += target[j].plane.getCenter();
    }

    source_cmean /= selected_planes.size();
    target_cmean /= selected_planes.size();

    // Compute translation vector from source center mean to target center mean.
    vec3 t_vec = target_cmean - source_cmean;

    // Compute translation matrix
    mat4 T = Eigen::Affine3f(Eigen::Translation3f(t_vec)).matrix();

    return T;
}

void Registration::computeMwithNormals()
{
    if(target.empty() || source.empty()) return;

    M.resize(source.size(), target.size());

    #pragma omp parallel for
    for(size_t i = 0; i < source.size(); ++i)
    {
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

void Registration::computeMwithCentroids(vector<vec3> &l_cS, vector<vec3> &l_cT, vector<float> &l_aS, vector<float> &l_aT, vector<float> &angles_cS, vector<float> &angles_cT)
{
    if(l_cT.empty() || l_cS.empty() || l_aS.empty() || l_aT.empty() || angles_cS.empty() || angles_cT.empty()) return;

    M.resize(l_cS.size(), l_cT.size());

    //#pragma omp parallel for
    for(size_t i = 0; i < l_cS.size(); ++i)
    {
        float normCSi = l_cS[i].norm();

        for(size_t j = 0; j < l_cT.size(); ++j)
        {
            M(i, j) = exp(-abs(((normCSi - l_cT[j].norm()) + /*(l_aS[i] - l_aT[j]) +*/ (source_surfaces[i] - target_surfaces[j])) /* (angles_cS[i] - angles_cT[j])*/));
            /*float m = abs((normCSi - l_cT[j].norm()) * (l_aS[i] - l_aT[j]) * (source_surfaces[i] - target_surfaces[j]));
            if(m == 0)
            {
                M(i, j) = 1000000.0f;
            }
            else
            {
                M(i, j) = 1.0f / m;
            }*/
        }
    }

    //M.normalize();
    /*
    Eigen::MatrixXf M_binary = Eigen::MatrixXf::Zero(M.rows(), M.cols());

    for (int i = 0; i < M_binary.rows(); ++i) {
        Eigen::MatrixXf::Index id;
        M.row(i).maxCoeff(&id);

        M_binary(i, id) = 1;
    }

    M = M_binary;
    */
    //cout << "M" << endl << M << endl;

    // TEST: Change color of second plane of source and corresponding plane in target to black
    //Eigen::MatrixXf::Index id;
    //M.row(1).maxCoeff(&id);
    //display_update_callable(source[1], target[id], ivec3(0, 0, 0));

    // Write M to file
    string filename("myMatrixMFile.txt");
    ofstream myFile;
    myFile.open(filename);
    myFile << M << endl;
    myFile.close();
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

vector<float> Registration::computeCenterAngles(vector<vec3> &l_shifted_centroids)
{
    vector<float> angles;

    // Compute mean
    vec3 center_mean(0, 0, 0);
    for(vec3 a: l_shifted_centroids)
    {
        center_mean += a;
    }
    center_mean.normalize();

    // Compute angles
    for(vec3 ci: l_shifted_centroids)
    {
        ci.normalize();
        float cos_a = ci.dot(center_mean);
        angles.push_back(cos_a);
    }

    return angles;
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
    mat2 cov = mat2::Zero();

    float alpha = 1.0f / (l_2dPoints.size() - 1);

    for(size_t i = 0; i < l_2dPoints.size(); ++i)
    {
        cov += alpha * (l_2dPoints[i] - center) * (l_2dPoints[i] - center).transpose();
    }

    // SVD Decomposition
    Eigen::JacobiSVD<mat2> svd(cov);
    vec2 s = svd.singularValues();

    // Non correlated variances
    s = s.array().abs().sqrt().matrix();

    // compute surface estimation
    return s.x() * s.y() * 4 * 3; // ???
}

vector<float> Registration::computeDelaunaySurfaces(PointNormalKCloud::Ptr p_cloud, vector<SegmentedPointsContainer::SegmentedPlane> &l_planes)
{
    vector<float> surfaces;

    for(auto plane: l_planes)
    {
        surfaces.push_back(computeDelaunaySurface(p_cloud, plane));
    }

    return surfaces;
}

float Registration::computeDelaunaySurface(PointNormalKCloud::Ptr p_cloud, SegmentedPointsContainer::SegmentedPlane &plane)
{
    // Find Plane base
    vec3 e1, e2;
    computePlaneBase(plane, e1, e2);

    // Convert 3D points to 2D
    vector<vec2> list_2d = pointsTo2D(p_cloud, plane, e1, e2);

    // Compute Rectangle formed by points and fill vector of Point2f
    vec2 min(0, 0), max(0, 0);
    vector<cv::Point2f> l_p2f;
    for(vec2 i: list_2d)
    {
        min.x() = min.x() < i.x() ? min.x() : i.x();
        min.y() = min.y() < i.y() ? min.y() : i.y();
        max.x() = max.x() > i.x() ? max.x() : i.x();
        max.y() = max.y() > i.y() ? max.y() : i.y();

        l_p2f.push_back(cv::Point2f(i.x(), i.y()));
    }

    //DEBUG print points to 2d
    ofstream file;
    file.open("2dPoints.txt");
    for(auto v: l_p2f)
    {
        file << v.x << " " << v.y << endl;
    }
    file.close();

    // Compute delauney triangle
    cv::Rect rect(min.x() - 1, min.y() - 1, max.x()-min.x() + 2, max.y()-min.y() + 2);
    cv::Subdiv2D sub(rect);
    sub.insert(l_p2f);

    vector<cv::Vec6f> triangles;
    sub.getTriangleList(triangles);

    vector<cv::Vec4f> edges;
    sub.getEdgeList(edges);

    // Sum up every triangles' surface
    ofstream file2;
    file2.open("triangles.txt");

    float surface = 0;
    vec2 x1, x2, x3;
    for(size_t i = 0; i < triangles.size(); ++i)
    {
        // Triangles vertices in 2D
        cv::Vec6f t = triangles[i];
        x1 = vec2(t[0], t[1]);
        x2 = vec2(t[2], t[3]);
        x3 = vec2(t[4], t[5]);

        if(rect.contains(cv::Point2f(x1.x(), x1.y())) &&
           rect.contains(cv::Point2f(x2.x(), x2.y())) &&
           rect.contains(cv::Point2f(x3.x(), x3.y())))
        {
            vec2 v1 = x2 - x1;
            vec2 v2 = x3 - x1;

            float surf = 0.5f * abs(crossProduct(v1, v2));

            file2 << x1.transpose() << ", " << x2.transpose() << ", " << x3.transpose() << ", surface: " << surf << endl;
            surface += surf;
        }
    }
    file2.close();

    return surface;
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

    // Base should be orthogonal
    e2 = n.cross(e1).normalized();
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

/////======================== PFH Based computation of M =====================================================================

Eigen::MatrixXf Registration::computeMWithPFHSignature(int source_nb)
{
    PFHCloud source_signs = PFHEvaluation::computePFHSignatures(source);
    PFHCloud target_signs = PFHEvaluation::computePFHSignatures(target);

    // Construct indice list of planes
    vector<size_t> source_indices;
    size_t i = 0;
    for(auto p: source)
    {
        source_indices.push_back(i);
        i++;
    }

    // Sort index list by decreasing surface order
    sort(source_indices.begin(), source_indices.end(), [this](size_t i, size_t j){
        return this->source_surfaces[i] > this->source_surfaces[j];
    });

    Eigen::MatrixXf M_pfh = Eigen::MatrixXf::Zero(source.size(), target.size());
    vector<size_t> ignore_list;

    // Fill M by associating each source to one target based on pfh histograms errors
    for (i = 0; i < (int)source_indices.size() / 2.0f/*std::min((int)source_indices.size(), source_nb)*/; ++i)
    {
        size_t j = PFHEvaluation::getMinTarget(source_indices[i], source_signs, target_signs, ignore_list);
        M_pfh(source_indices[i], j) += 1;
        ignore_list.push_back(j);

        //List of selected plane tuples (source, target) for translation computation
        selected_planes.push_back(make_tuple(source_indices[i], j));
    }
    /*
    for(i = 0; i < source_signs.size(); ++i)
    {
        for(size_t j = 0; j < target_signs.size(); ++j)
        {
            M_pfh(i, j) = computePFHError(i, j, source_signs, target_signs);
        }
    }*/

    cout << M_pfh << endl;

    return M_pfh;
}

float Registration::computePFHError(size_t i, size_t j, PFHCloud &source, PFHCloud &target)
{
    float err(0);

    auto hist_source = source[i].histogram;
    auto hist_target = target[j].histogram;

    for(size_t k = 0; k < source[i].descriptorSize(); ++k)
    {
        err += abs(hist_source[k] - hist_target[k]);
    }

    return exp(-0.01*err);
}
