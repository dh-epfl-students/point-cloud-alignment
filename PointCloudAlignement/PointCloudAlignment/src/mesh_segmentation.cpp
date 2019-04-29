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
    p_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromPCLPointCloud2(p_mesh->cloud, *p_cloud);

    return true;
}

void MeshSegmentation::segmentPlanes()
{
    ivec3 color(0, 0, 0);

    #pragma omp parallel for
    for (size_t i = 0; i < p_mesh->polygons.size(); ++i) {
        // Get vetices
        vector<vec3> vertices;
        fillVertices(p_mesh->polygons.at(i), vertices);

        // Compute surface normal vector
        vec3 n = computeFaceNormal(vertices);

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

    cout << "Plane segmentation of mesh finished first phase." << endl;
}

void MeshSegmentation::mergePlanes()
{

}

void MeshSegmentation::fillVertices(pcl::Vertices verts, vector<vec3> &vertices)
{
    for(auto v: verts.vertices)
    {
        pcl::PointNormal p(p_cloud->points[v]);
        vertices.push_back(vec3(p.x, p.y, p.z));
    }
}

vec3 MeshSegmentation::computeFaceNormal(vector<vec3> &vertices)
{
    //Consider that this is a triangle mesh
    vec3 t1 = vertices[1] - vertices[0];
    vec3 t2 = vertices[2] - vertices[0];

    // Return normal vector not normalized
    return t1.cross(t2);
}
