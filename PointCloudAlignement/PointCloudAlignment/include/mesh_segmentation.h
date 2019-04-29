#pragma once

#include <iostream>

#include <omp.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include "common.h"
#include "segmented_points_container.h"

class MeshSegmentation {
public:
    bool loadMesh(string filename);
    void segmentPlanes();
    void mergePlanes();

    pcl::PolygonMeshPtr getMeshPtr() { return p_mesh; }

private:
    pcl::PolygonMeshPtr p_mesh;
    pcl::PointCloud<pcl::PointNormal>::Ptr p_cloud;
    vector<SegmentedPointsContainer::SegmentedPlane> planes;

    void fillVertices(pcl::Vertices verts, vector<vec3> &vertices);
    vec3 computeFaceNormal(vector<vec3> &vertices);
};
