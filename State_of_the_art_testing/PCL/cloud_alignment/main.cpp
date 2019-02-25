#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "feature_cloud.h"
#include "template_alignment.h"

using namespace std;

int main()
{
    string filename = "/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply";
    string personfile = "/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/person.pcd";
    string templatefile = "/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/object_template_0.pcd";


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPCDFile(personfile, *cloud) == -1) {
        PCL_ERROR("Could not read point cloud ply file\n");
        return EXIT_FAILURE;
    }

    cout << "Loaded "
         << cloud->width * cloud->height
         << " data points from point cloud in ply format."
         << std::endl;
/*
    // First center the target point cloud
    Eigen::Vector4f centroid = Eigen::Vector4f::UnitZ();
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.translation() << -centroid.x(), -centroid.y(), -centroid.z();
    pcl::transformPointCloud(*cloud, *cloud, trans);
*/
    // downsampling the point cloud
    // This does not seem to improve the computation time by a lot.
    const float voxel_grid_size = 0.01f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud (cloud);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter (*tempCloud);
    cloud = tempCloud;
    /*
    // Create transform
    float theta = M_PI / 4.0f;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    */

    FeatureCloud target_cloud;
    target_cloud.setInputCloud(cloud);
    //target_cloud.loadInputCloud(personfile);

    FeatureCloud template_cloud;
    //template_cloud.setInputCloud(transformed_cloud);
    template_cloud.loadInputCloud(templatefile);

    TemplateAlignment template_align;
    template_align.addTemplateCloud(template_cloud);
    template_align.setTargetCloud(target_cloud);

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment (best_alignment, "KFPCS");

    // Print the alignment fitness score
    std::printf ("Best fitness score: %f\n", best_alignment.fitness_score);

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

    std::printf ("\n");
    std::printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    std::printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    std::printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    std::printf ("\n");
    std::printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

    pcl::PointCloud<pcl::PointXYZ>::Ptr best_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*(template_cloud.getPointCloud()), *best_cloud, best_alignment.final_transformation);

    // Create viewer
    pcl::visualization::PCLVisualizer viewer("Test Viewer");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (target_cloud.getPointCloud(), 1, 255, 1); // Green
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (target_cloud.getPointCloud(), source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (template_cloud.getPointCloud(), 230, 20, 20); // Red
    viewer.addPointCloud (template_cloud.getPointCloud(), transformed_cloud_color_handler, "transformed_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> best_cloud_color_handler (best_cloud, 20, 20, 230); // Blue
    viewer.addPointCloud (best_cloud, best_cloud_color_handler, "best_cloud");

    viewer.addCoordinateSystem (1.0, "original_cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    while(!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return (0);
}

