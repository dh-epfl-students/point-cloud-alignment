#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main()
{
    string filename1 = "/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply";
    string filename2 = "/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safeTarget.ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPLYFile(filename1, *cloud) == -1) {
        PCL_ERROR("Could not read point cloud ply file\n");
        return EXIT_FAILURE;
    }

    if(pcl::io::loadPLYFile(filename2, *target) == -1) {
        PCL_ERROR("Could not read target cloud ply file\n");
        return EXIT_FAILURE;
    }

    cout << "Loaded "
         << cloud->width * cloud->height
         << " data points from point cloud in ply format."
         << std::endl;

    pcl::visualization::CloudViewer viewer("Simple cloud viewer");
    //viewer.showCloud(cloud);
    viewer.showCloud(target);

    while(!viewer.wasStopped()) {}

    return (0);
}

