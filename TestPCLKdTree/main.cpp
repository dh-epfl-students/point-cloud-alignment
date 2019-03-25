#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
    string pcFile("/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply");

    //Load point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile(pcFile, *p_cloud);

    pcl::KdTreeFLANN<pcl::PointXYZRGB, flann::L2_Simple<float>>::Ptr p_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGB, flann::L2_Simple<float>>);
    p_kdtree->setInputCloud(p_cloud);

    int index = 24630;
    vector<int> indices;
    vector<float> distances;
    pcl::PointXYZRGB p = p_cloud->points[index];
    cout << p.x << " " << p.y << " " << p.z << endl;

    p_kdtree->nearestKSearch(p_cloud->points[index], 50, indices, distances);

    for(int i: indices)
    {
        p_cloud->points[i].r = 15;
        p_cloud->points[i].g = 255;
        p_cloud->points[i].b = 15;
    }
    p_cloud->points[index].g = 15;
    p_cloud->points[index].r = 255;

    pcl::visualization::PCLVisualizer::Ptr p_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    p_viewer->setBackgroundColor(0.4, 0.4, 0.4);
    p_viewer->initCameraParameters();

    /*
    //pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> single_color(algo.getPointCloud(), 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(algo.getPointCloud());
    p_viewer->addPointCloud(algo.getPointCloud(), rgb, "point_cloud");
*/

    p_viewer->addPointCloud(p_cloud, "point_cloud");

    // Drawing loop
    while(!p_viewer->wasStopped())
    {
        p_viewer->spinOnce(100);
    }

    return 0;
}
