#include "mesh_segmentation.h"

bool MeshSegmentation::loadMesh(string filename)
{
    p_mesh = pcl::PolygonMeshPtr(new pcl::PolygonMesh);

    if(pcl::io::loadPolygonFilePLY(filename, *p_mesh) == -1)
    {
        cout << "Failed to load given file" << endl;
        return false;
    }

    cout << "Loaded mesh " << filename << " : " << p_mesh << endl;

    return true;
}

void MeshSegmentation::segmentPlanes()
{

}
