#pragma once

#include <iostream>

#include <omp.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include "common.h"
#include "segmented_points_container.h"

#define KNN_MESH 50
#define COS_2 0.999f
#define COS_5 0.996f
#define V_ERROR 0.5f

class MeshSegmentation {
public:
    bool loadMesh(string filename);
    void segmentPlanes();
    void mergePlanes();

    bool isMeshSegmented();

    pcl::PolygonMeshPtr getMeshPtr() { return p_mesh; }
    vector<SegmentedPointsContainer::SegmentedPlane> getSegmentedPlanes();

private:
    pcl::PolygonMeshPtr p_mesh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_centroid_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr p_kdTree;
    vector<SegmentedPointsContainer::SegmentedPlane> planes;
    boost::shared_ptr<vector<int>> p_available_indices;
    bool isSegmented = false;

    void mergeRecursive();
    bool planesAreMergeable(SegmentedPointsContainer::SegmentedPlane &p1, SegmentedPointsContainer::SegmentedPlane &p2);
    bool haveCommonVertex(SegmentedPointsContainer::SegmentedPlane &p1, SegmentedPointsContainer::SegmentedPlane &p2);
    void fillVertices(pcl::Vertices verts, vector<vec3> &vertices);
    vec3 computeFaceNormal(vector<vec3> &vertices);
    void updatePCcolors();
};