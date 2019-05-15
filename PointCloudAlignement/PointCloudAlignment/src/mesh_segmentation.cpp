#include "mesh_segmentation.h"

bool MeshSegmentation::loadMesh(string filename)
{
    p_mesh = pcl::PolygonMeshPtr(new pcl::PolygonMesh);

    if(pcl::io::loadPolygonFilePLY(filename, *p_mesh) == -1)
    {
        cout << "Failed to load given file" << endl;
        return false;
    }

    cout << "Loaded mesh " << filename << endl;

    // Converting PolygonMesh->cloud to PointCloud<PointNormal>
    p_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(p_mesh->cloud, *p_cloud);

    return true;
}

void MeshSegmentation::segmentPlanes()
{
    ivec3 color(0, 0, 0);

    //#pragma omp parallel for
    for (size_t i = 0; i < p_mesh->polygons.size(); ++i)
    {
        // Get vertices
        vector<vec3> vertices;
        fillVertices(p_mesh->polygons.at(i), vertices);

        // Compute surface normal vector
        vec3 n = computeFaceNormal(vertices);

        if(!n.isZero())
        {
            // Compute surface centroid
            vec3 c = (vertices[0] + vertices[1] + vertices[2]) / 3.0f;

            // Assign a random color to this plane for visualization purposes
            color.setRandom();
            color = positive_modulo(color, 255);

            // Create and store plane
            vector<int> v_int(p_mesh->polygons.at(i).vertices.begin(), p_mesh->polygons.at(i).vertices.end());
            Plane p(c, n);
            planes.push_back(SegmentedPointsContainer::SegmentedPlane(static_cast<int>(i), color, v_int, p));
        }
    }

    cout << "Plane segmentation of mesh finished first phase." << endl;
}

void MeshSegmentation::mergePlanes()
{
    if(planes.empty()) return;

    p_available_indices = boost::shared_ptr<vector<int>>(new vector<int>);

    // Construct point cloud of centroids
    p_centroid_cloud = pcl::PointCloud<pcl::PointXYZ>().makeShared();
    for(size_t i = 0; i < planes.size(); ++i)
    {
        p_centroid_cloud->points.push_back(planes[i].plane.getCenterPCL());
        p_available_indices->push_back(i);
    }

    // Fill kdTree of centroids
    p_kdTree = pcl::KdTreeFLANN<pcl::PointXYZ>().makeShared();
    p_kdTree->setInputCloud(p_centroid_cloud);

    mergeRecursive();

    // Removing merged planes from planes list, and planes with small surface
    vector<SegmentedPointsContainer::SegmentedPlane> final_planes;
    for_each(p_available_indices->begin(), p_available_indices->end(), [&final_planes, this](int index){
        if(planes[index].plane.getNormal().norm() > MIN_SURFACE)
        {
            final_planes.push_back(planes[index]);
        }
    });
    planes.swap(final_planes);

    cout << "Finished mesh planes merging" << endl;

    //Update colors in pc
    updatePCcolors();

    isSegmented = true;
}

void MeshSegmentation::mergeRecursive()
{
    cout << "Starting one level of merge. Available indices: " << p_available_indices->size() << endl;

    vector<int> merged_planes;

    for(size_t i = 0; i < p_available_indices->size(); ++i)
    {
        if(!binary_search(merged_planes.begin(), merged_planes.end(), p_available_indices->at(i)))
        {
            SegmentedPointsContainer::SegmentedPlane plane = planes[p_available_indices->at(i)];

            // Do a knn search on neighboring centroids
            vector<int> nghbrs(KNN_MESH);
            vector<float> dists(KNN_MESH);
            p_kdTree->nearestKSearch(plane.plane.getCenterPCL(), KNN_MESH, nghbrs, dists);

            for(size_t j = 1; j < nghbrs.size(); ++j)
            {
                int nghbr_id = nghbrs[j];

                if(!binary_search(merged_planes.begin(), merged_planes.end(), nghbr_id))
                {
                    SegmentedPointsContainer::SegmentedPlane nghbr_plane = planes[nghbr_id];
                    if(planesAreMergeable(plane, nghbr_plane))
                    {
                        plane.merge(nghbr_plane);

                        // Update list of planes and centroid cloud
                        planes[p_available_indices->at(i)] = plane;
                        p_centroid_cloud->points[p_available_indices->at(i)] = plane.plane.getCenterPCL();

                        merged_planes.insert(upper_bound(merged_planes.begin(), merged_planes.end(), nghbr_id), nghbr_id);
                    }
                }
            }
        }
    }

    // Update availaible_indices and kdTree
    vector<int> new_list;
    set_difference(p_available_indices->begin(), p_available_indices->end(), merged_planes.begin(), merged_planes.end(), back_inserter(new_list));
    p_available_indices->swap(new_list);

    p_kdTree->setInputCloud(p_centroid_cloud, p_available_indices);

    cout << "Finishing one level of merge. Available indices: " << p_available_indices->size() << ", merged planes: " << merged_planes.size() << endl;

    if(merged_planes.size() > 0){
        mergeRecursive();
    }
}

bool MeshSegmentation::planesAreMergeable(SegmentedPointsContainer::SegmentedPlane &p1, SegmentedPointsContainer::SegmentedPlane &p2)
{
    vec3 n1 = p1.plane.getNormal().normalized();
    vec3 n2 = p2.plane.getNormal().normalized();

    // may need to reorient normal
    n2  = n2.dot(n1) < 0.0f ? -n2 : n2;

    return (n2.dot(n1) >= COS_5) && haveCommonVertex(p1, p2);
}

bool MeshSegmentation::haveCommonVertex(SegmentedPointsContainer::SegmentedPlane &p1, SegmentedPointsContainer::SegmentedPlane &p2)
{
    for(auto i: p1.indices_list)
    {
        for(auto j: p2.indices_list)
        {
            if(squaredDistance(pclToVec3(p_cloud->points[i]), pclToVec3(p_cloud->points[j])) <= V_ERROR) return true;
        }
    }

    return false;
}

void MeshSegmentation::fillVertices(pcl::Vertices verts, vector<vec3> &vertices)
{
    for(auto v: verts.vertices)
    {
        pcl::PointXYZRGB p(p_cloud->points[v]);
        vertices.push_back(vec3(p.x, p.y, p.z));
    }
}

vec3 MeshSegmentation::computeFaceNormal(vector<vec3> &vertices)
{
    //Consider that this is a triangle mesh
    vec3 t1 = vertices[1] - vertices[0];
    vec3 t2 = vertices[2] - vertices[0];

    // Return normal vector not normalized
    return t1.cross(t2) / 2.0f;
}

void MeshSegmentation::updatePCcolors()
{
    #pragma omp parallel for
    for(size_t i = 0; i < planes.size(); ++i)
    {
        ivec3 color = planes[i].color;

        for(int v_id : planes[i].indices_list)
        {
            p_cloud->points[v_id].rgba = static_cast<uint8_t>(color.x()) << 16 |
                                         static_cast<uint8_t>(color.y()) << 8 |
                                         static_cast<uint8_t>(color.z());
        }
    }

    pcl::PCLPointCloud2 pc2;
    pcl::toPCLPointCloud2(*p_cloud, pc2);
    p_mesh->cloud = pc2;
}

bool MeshSegmentation::isMeshSegmented()
{
    return isSegmented;
}

vector<SegmentedPointsContainer::SegmentedPlane> MeshSegmentation::getSegmentedPlanes()
{
    return this->planes;
}

void MeshSegmentation::updateColors(SegmentedPointsContainer::SegmentedPlane p, ivec3 color)
{
    for(auto i: p.indices_list)
    {
        p_cloud->points[i].rgba = static_cast<uint8_t>(color.x()) << 16 |
                                  static_cast<uint8_t>(color.y()) << 8 |
                                  static_cast<uint8_t>(color.z());
    }

    pcl::PCLPointCloud2 pc2;
    pcl::toPCLPointCloud2(*p_cloud, pc2);
    p_mesh->cloud = pc2;
}
