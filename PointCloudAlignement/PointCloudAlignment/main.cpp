#include <iostream>
#include <string>

#include "common.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "plane_segmentation.h"

using namespace std;

PlaneSegmentation algo;

void keyboardCallback(const pcl::visualization::KeyboardEvent &event,
                      void* viewer_void) {
    if(event.getKeySym() == "l" && event.keyDown()) {
        cout << "Segmentation started!" << endl;

        algo.start_pause();

    }
}

// Start and setup viewer
pcl::visualization::PCLVisualizer::Ptr setupViewer() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    //viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();

    viewer->registerKeyboardCallback(keyboardCallback, (void*)viewer.get());

    return viewer;
}

int main()
{
    string pcFile("/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply");

    pcl::visualization::PCLVisualizer::Ptr viewer = setupViewer();

    algo.init(pcFile);

    pcl::visualization::PointCloudColorHandlerGenericField<Point3N> curv(algo.getPointCloud(), "curvature");
    //pcl::visualization::PointCloudColorHandlerCustom<Point3N> single_color(algo.getPointCloud(), 0, 255, 0);
    viewer->addPointCloud(algo.getPointCloud(), curv, "point_cloud");

    //viewer->addPointCloudNormals<Point3N, Point3N>(algo.getPointCloud(), algo.getPointCloud(), 1, 1, "normals");

    // Drawing loop
    while(!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
