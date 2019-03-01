#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;


int main()
{
    string pcFile("/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply");

    PointCloud::Ptr p_cloud(new PointCloud);
    if(pcl::io::loadPLYFile(pcFile, *p_cloud) == -1) {
    	PCL_ERROR("Could not read given ply file\n");
    	return EXIT_FAILURE;
    }

    // Compute normals estimations
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(p_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr p_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(p_tree);
    ne.setRadiusSearch(10);
    NormalCloud::Ptr p_normals(new NormalCloud);

    ne.compute(*p_normals);

    // Setup and display viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(p_cloud, 0, 255, 0);
    viewer->addPointCloud(p_cloud, single_color, "point_cloud");

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(p_cloud, p_normals, 10, 1, "normals");
    viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();

    // Drawing loop
    while(!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
