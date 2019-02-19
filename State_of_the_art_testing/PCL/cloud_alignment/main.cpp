#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main()
{
    string filename = "/home/loris/Documents/EPFL/Master/Master_Project/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPLYFile(filename, *cloud) == -1) {
        PCL_ERROR("Could not read ply file\n");
        return EXIT_FAILURE;
    }

    cout << "Loaded "
         << cloud->width * cloud->height
         << " data points from point cloud in ply format."
         << std::endl;

    pcl::visualization::CloudViewer viewer("Simple cloud viewer");
    viewer.showCloud(cloud);

    while(!viewer.wasStopped()) {}

    return (0);
}

