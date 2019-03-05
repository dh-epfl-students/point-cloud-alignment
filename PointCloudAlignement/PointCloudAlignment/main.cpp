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
/*
    PointCloud::Ptr p_cloud(new PointCloud);
    if(pcl::io::loadPLYFile(pcFile, *p_cloud) == -1) {
        PCL_ERROR("Could not read given ply file\n");
        return EXIT_FAILURE;
    }

    // Compute normals estimations
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(p_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr p_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(p_tree);
    //ne.setRadiusSearch(10);
    ne.setKSearch(25);
    NormalCloud::Ptr p_normals(new NormalCloud);

    ne.compute(*p_normals);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(p_cloud, 0, 255, 0);
    viewer->addPointCloud(p_cloud, single_color, "point_cloud");

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(p_cloud, p_normals, 5, 1, "normals");
*/
    // Drawing loop
    while(!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
