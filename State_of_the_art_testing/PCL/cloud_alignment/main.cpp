#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "feature_cloud.h"
#include "template_alignment.h"

using namespace std;

int main()
{
    string filename = "/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPLYFile(filename, *cloud) == -1) {
        PCL_ERROR("Could not read point cloud ply file\n");
        return EXIT_FAILURE;
    }

    cout << "Loaded "
         << cloud->width * cloud->height
         << " data points from point cloud in ply format."
         << std::endl;

    // First center the target point cloud
    Eigen::Vector4f centroid = Eigen::Vector4f::UnitZ();
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.translation() << -centroid.x(), -centroid.y(), -centroid.z();
    pcl::transformPointCloud(*cloud, *cloud, trans);

    // Create transform
    float theta = M_PI / 10.0f;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    // Create viewer
    pcl::visualization::PCLVisualizer viewer("Test Viewer");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 1, 255, 1); // Green
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem (1.0, "original_cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    FeatureCloud target_cloud;
    target_cloud.setInputCloud(cloud);

    FeatureCloud template_cloud;
    template_cloud.setInputCloud(transformed_cloud);

    TemplateAlignment template_align;
    template_align.addTemplateCloud(template_cloud);
    template_align.setTargetCloud(target_cloud);

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment (best_alignment);

    // Print the alignment fitness score (values less than 0.00002 are good)
    std::printf ("Best fitness score: %f\n", best_alignment.fitness_score);

    pcl::PointCloud<pcl::PointXYZ>::Ptr best_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*transformed_cloud, *best_cloud, best_alignment.final_transformation);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> best_cloud_color_handler (best_cloud, 20, 20, 230); // Blue
    viewer.addPointCloud (best_cloud, best_cloud_color_handler, "best_cloud");

    while(!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return (0);
}

