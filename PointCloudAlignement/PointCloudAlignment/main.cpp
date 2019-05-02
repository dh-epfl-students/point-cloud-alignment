#include <iostream>
#include <string>
#include <limits>

#include <omp.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "common.h"
#include "plane_segmentation.h"
#include "plane_merging.h"
#include "mesh_segmentation.h"
#include "registration.h"

using namespace std;

static PlaneSegmentation algo;
static PlaneMerging merger;
static MeshSegmentation meshSeg;
static Registration registration;
static pcl::visualization::PCLVisualizer::Ptr p_viewer;

bool isNormalDisplayed = false;
bool isExclusionDisplayed = false;
bool pc_has_changed = false;
bool normal_cloud_changed = false;
bool refresh_mesh = false;
bool mesh_is_segmented = false;

int plane_nb = 0;

string mesh_filename;

void keyboardCallback(const pcl::visualization::KeyboardEvent &event,
                      void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

    if(event.getKeySym() == "l" && event.keyDown())
    {
        cout << "Segmentation started!" << endl;

        algo.start_pause();
    }
    else if(event.getKeySym() == "F1" && event.keyDown())
    {
        viewer->removePointCloud("point_cloud");
        pcl::visualization::PointCloudColorHandlerRGBField<PointNormalK> rgb(algo.getPointCloud());
        viewer->addPointCloud(algo.getPointCloud(), rgb, "point_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
    }
    else if(event.getKeySym() == "F2" && event.keyDown())
    {
        viewer->removePointCloud("point_cloud");
        pcl::visualization::PointCloudColorHandlerGenericField<PointNormalK> curv(algo.getPointCloud(), "curvature");
        viewer->addPointCloud(algo.getPointCloud(), curv, "point_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
    }
    else if(event.getKeySym() == "F3" && event.keyDown())
    {
        viewer->removePointCloud("point_cloud");
        pcl::visualization::PointCloudColorHandlerGenericField<PointNormalK> k(algo.getPointCloud(), "k");
        viewer->addPointCloud(algo.getPointCloud(), k, "point_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
    }
    else if(event.getKeySym() == "F4" && event.keyDown())
    {
        if(isNormalDisplayed)
        {
            isNormalDisplayed = false;
            viewer->removePointCloud("normal_cloud");
        }
        else
        {
            isNormalDisplayed = true;
            viewer->addPointCloudNormals<PointNormalK, PointNormalK>(algo.getPointCloud(), algo.getPointCloud(), 5, 1, "normal_cloud");
        }
    }
    else if(event.getKeySym() == "F5" && event.keyDown())
    {
        #pragma omp parallel
        {
            #pragma omp single
            {
                // Load target mesh
                if(meshSeg.loadMesh(mesh_filename))
                {
                    refresh_mesh = true;

                    // Start plane segmentation
                    meshSeg.segmentPlanes();
                    meshSeg.mergePlanes();

                    mesh_is_segmented = true;
                    refresh_mesh = true;
                }
            }
        }
    }
    else if(event.getKeySym() == "F6" && event.keyDown())
    {
        string filename = "myPC.ply";

        cout << "Saving ply file " << filename << endl;

        // Writing PC to file
        pcl::io::savePLYFile(filename, *algo.getPointCloud());
    }
    else if(event.getKeySym() == "F7" && event.keyDown())
    {
        if(isExclusionDisplayed)
        {
            cout << "Remove clouds" << endl;

            viewer->removePointCloud("available_cloud");
            viewer->removePointCloud("excluded_cloud");
            isExclusionDisplayed = false;
        }
        else
        {
            cout << "Display clouds" << endl;

            PointNormalKCloud::Ptr red_c = algo.getExcludedPointCloud();
            PointNormalKCloud::Ptr green_c = algo.getAvailablePointCloud();

            // Display the available points in green and excluded points in red
            pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> red(red_c, 255, 15, 15);
            pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> green(green_c, 15, 255, 15);

            // Add 2 point clouds: 1 for available, 1 for excluded
            viewer->addPointCloud(green_c, green, "available_cloud");
            viewer->addPointCloud(red_c, red, "excluded_cloud");

            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "available_cloud");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "excluded_cloud");

            isExclusionDisplayed = true;
        }
    }
    else if(event.getKeySym() == "F8" && event.keyDown())
    {
        // Filter out points with high curvature
        float max_curvature = algo.getCurvBound();

        cout << "Filtering out points with curvature greater than " << max_curvature << endl;

        algo.filterOutCurvature(max_curvature);
    }
    else if(event.getKeySym() == "F9" && event.keyDown())
    {
        algo.preprocessCloud();
    }
    else if(event.getKeySym() == "F10" && event.keyDown())
    {
        // Resample cloud
        algo.resampleCloud();
    }
    else if(event.getKeySym() == "F11" && event.keyDown())
    {
        if(algo.isCloudSegmented())
        {
            vector<SegmentedPointsContainer::SegmentedPlane> planes_list = algo.getSegmentedPlanes();
            // Merge similar planes
            merger.start_merge(planes_list, algo.getPointCloud());
        }
        else
        {
            cout << "Could not merge planes because the cloud is not segmented." << endl;
        }
    }
    else if(event.getKeySym() == "a" && event.keyDown())
    {
        if(!(merger.isCloudMerged() && meshSeg.isMeshSegmented())) return;

        // Get both segmented sets of planes
        vector<SegmentedPointsContainer::SegmentedPlane> source = merger.getSegmentedPlanes();
        vector<SegmentedPointsContainer::SegmentedPlane> target = meshSeg.getSegmentedPlanes();

        // Find Rotation
        registration.setClouds(source, target, true);
        mat3 R = registration.findAlignment();

        // We need to compose the transformation matrix -> translate point cloud to origin
        //                                              -> apply the rotation
        //                                              -> translate it back to original position
        vec4 centroid;
        pcl::compute3DCentroid(*algo.getPointCloud(), centroid);

        mat4 T1 = Eigen::Affine3f(Eigen::Translation3f(vec3(-centroid.x(), -centroid.y(), -centroid.z()))).matrix();
        cout << " T1:" << endl << T1 << endl;

        mat4 T2 = T1.inverse();
        cout << " T2:" << endl << T2 << endl;

        mat4 M = mat4::Identity();
        M.block(0, 0, 3, 3) << R;
        cout << "R:" << endl << M << endl;

        mat4 finalTransform = T2 * /*M * */M * T1;

        cout << "Final Transformation:" << endl << finalTransform << endl;
        cout << "Inverse of final transform:" << endl << finalTransform.inverse() << endl;

        PointNormalKCloud::Ptr p_transformed_cloud = PointNormalKCloud().makeShared();
        pcl::transformPointCloud(*algo.getPointCloud(), *p_transformed_cloud, finalTransform);

        pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> color(p_transformed_cloud, 0, 255, 0);
        p_viewer->removePointCloud("transformed_cloud");
        p_viewer->addPointCloud(p_transformed_cloud, color, "transformed_cloud");
        p_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "transformed_cloud");

        // Update point cloud
        algo.setPointCloud(p_transformed_cloud);

        // Update merger list of normals and centroids
        merger.applyTransform(M);
    }
    else if(event.keyDown())
    {
        cout << event.getKeySym() << ", " << event.getKeyCode() << endl;
    }
}

void display_update_callback(PointNormalKCloud::Ptr p_cloud, ivec3 color, vector<int> indices)
{
    for(int i: indices)
    {
        p_cloud->points[i].rgba = static_cast<uint8_t>(color.x()) << 16 |
                                  static_cast<uint8_t>(color.y()) << 8 |
                                  static_cast<uint8_t>(color.z());
    }

    pc_has_changed = true;
}

void add_plane_callback(pcl::ModelCoefficients coeffs, float x, float y, float z)
{
    string plane_n = &"plane" [ plane_nb];
    cout << "Adding " << plane_n << endl;
    p_viewer->addPlane(coeffs, x, y, z, plane_n);
    ++plane_nb;
}

void update_normal_cloud_callback()
{
    if(isNormalDisplayed) normal_cloud_changed = true;
}

function<void(PointNormalKCloud::Ptr, ivec3 color, vector<int> indices)> display_update_callable = &display_update_callback;
function<void(pcl::ModelCoefficients, float, float, float)> add_plane_callable = &add_plane_callback;
function<void(void)> update_normal_cloud_callable = &update_normal_cloud_callback;

// Start and setup viewer
pcl::visualization::PCLVisualizer::Ptr setupViewer()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.3, 0.3, 0.3);
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback(keyboardCallback, static_cast<void*>(viewer.get()));

    return viewer;
}

int main()
{
    // Target meshes
    string mesh_region3_1("/home/loris/Documents/EPFL/Master/master-project-2019/Data/BUILDING_Geneva/geneva_region-03/region-03_2018_seg1_shifted_float.ply");
    string mesh_region3_2("/home/loris/Documents/EPFL/Master/master-project-2019/Data/BUILDING_Geneva/geneva_region-03/region-03_2018_seg2_shifted_float.ply");
    string mesh_region3_3("/home/loris/Documents/EPFL/Master/master-project-2019/Data/BUILDING_Geneva/geneva_region-03/region-03_2018_seg3_shifted_float.ply");
    string mesh_region3_4("/home/loris/Documents/EPFL/Master/master-project-2019/Data/BUILDING_Geneva/geneva_region-03/region-03_2018_seg4_shifted_float.ply");
    string mesh_region3_5("/home/loris/Documents/EPFL/Master/master-project-2019/Data/BUILDING_Geneva/geneva_region-03/region-03_2018_seg5_shifted_float.ply");
    mesh_filename = mesh_region3_4;

    // Original PC sources
    string pcLIDAR_region3_2017_seg1("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg1_shifted_float.ply");
    string pcLIDAR_region3_2017_seg2("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg2_shifted_float.ply");
    string pcLIDAR_region3_2017_seg3("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg3_shifted_float.ply");
    string pcLIDAR_region3_2017_seg4("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg4_shifted_float.ply");
    string pcLIDAR_region3_2017_seg5("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg5_shifted_float.ply");

    // Rotated PC sources
    string pcLIDAR_region3_2017_seg1_rotated_1("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg1_shifted_float_rotated_1.ply");
    string pcLIDAR_region3_2017_seg1_rotated_2("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg1_shifted_float_rotated_2.ply");
    string pcLIDAR_region3_2017_seg1_rotated_3("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg1_shifted_float_rotated_3.ply");
    string pcLIDAR_region3_2017_seg4_rotated_1("/home/loris/Documents/EPFL/Master/master-project-2019/Data/LIDAR_Geneva/geneva_region-03/region-03_2017-aerial/2504000_1116000_seg4_shifted_float_rotated_1.ply");


    p_viewer = setupViewer();

    algo.init(pcLIDAR_region3_2017_seg4_rotated_1);
    algo.setViewerUpdateCallback(display_update_callable);
    algo.setAddPlaneCallback(add_plane_callable);
    algo.setUpdateNormalCloudCallback(update_normal_cloud_callable);

    merger.init(display_update_callable);

    pcl::visualization::PointCloudColorHandlerRGBField<PointNormalK> rgb(algo.getPointCloud());
    p_viewer->addPointCloud(algo.getPointCloud(), rgb, "point_cloud");
    p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
    p_viewer->resetCamera();

    // Start plane segmentation thread and viewer thread
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            cout << "Viewer loop executed by thread " << omp_get_thread_num() << endl;

            // Drawing loop
            while(!p_viewer->wasStopped())
            {
                if(pc_has_changed)
                {
                    pc_has_changed = false;

                    // The function updatePointCloud doesn't work, thus it is necessary to remove and add the cloud
                    p_viewer->removePointCloud("point_cloud");
                    pcl::visualization::PointCloudColorHandlerRGBField<PointNormalK> rgb(algo.getPointCloud());
                    p_viewer->addPointCloud(algo.getPointCloud(), rgb, "point_cloud");
                    p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
                }

                if(isNormalDisplayed && normal_cloud_changed)
                {
                    normal_cloud_changed = false;
                    p_viewer->removePointCloud("normal_cloud");
                    p_viewer->addPointCloudNormals<PointNormalK, PointNormalK>(algo.getPointCloud(), algo.getPointCloud(), 5, 1, "normal_cloud");
                }

                if(refresh_mesh)
                {
                    refresh_mesh = false;
                    p_viewer->removePolygonMesh("city_mesh");
                    p_viewer->addPolygonMesh(*meshSeg.getMeshPtr(), "city_mesh");
                }

                p_viewer->spinOnce(100);
            }

            cout << "Viewer was stopped" << endl;

            algo.stop();
        }

        #pragma omp section
        {
            cout << "Segmentation loop started by thread " << omp_get_thread_num() << endl;

            algo.runMainLoop();
        }
    }

    return 0;
}
