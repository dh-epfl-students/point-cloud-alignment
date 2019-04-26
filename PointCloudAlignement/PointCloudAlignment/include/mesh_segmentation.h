#pragma once

#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include "common.h"

class MeshSegmentation {
public:
    bool loadMesh(string filename);
    void segmentPlanes();

private:
    pcl::PolygonMeshPtr p_mesh;
};
