#include <iostream>
#include <string>

#include <omp.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "common.h"
#include "plane_segmentation.h"

using namespace std;

PlaneSegmentation algo;
bool isNormalDisplayed = false;
bool isExclusionDisplayed = false;
bool pc_has_changed = false;
pcl::visualization::PCLVisualizer::Ptr p_viewer;
int plane_nb = 0;

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
        algo.runOneStep();
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

function<void(PointNormalKCloud::Ptr, ivec3 color, vector<int> indices)> display_update_callable = &display_update_callback;
function<void(pcl::ModelCoefficients, float, float, float)> add_plane_callable = &add_plane_callback;

// Start and setup viewer
pcl::visualization::PCLVisualizer::Ptr setupViewer()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.2, 0.2, 0.2);
    //viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();

    viewer->registerKeyboardCallback(keyboardCallback, static_cast<void*>(viewer.get()));

    return viewer;
}

int main()
{
    string pcFile("/home/loris/Documents/EPFL/Master/master-project-2019/State_of_the_art_testing/PCL/cloud_alignment/samples/2009geneve1safe.ply");
    string pcFileWithPreprocessed("/home/loris/Documents/EPFL/Master/master-project-2019/PointCloudAlignement/build-PointCloudAlignment-Desktop-Default/myPC.ply");

    p_viewer = setupViewer();

    algo.init(pcFileWithPreprocessed);
    algo.setViewerUpdateCallback(display_update_callable);
    algo.setAddPlaneCallback(add_plane_callable);

    pcl::visualization::PointCloudColorHandlerRGBField<PointNormalK> rgb(algo.getPointCloud());
    p_viewer->addPointCloud(algo.getPointCloud(), rgb, "point_cloud");
    p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
    p_viewer->resetCamera();

    // Start plane segmentation thread
    #pragma omp parallel
    {
        #pragma omp master
        {
            // Drawing loop
            while(!p_viewer->wasStopped())
            {
                if(pc_has_changed)
                {
                    pc_has_changed = false;

                    // The function updatePointCloud doesn't work, thus it is necessary to remove and add the cloud
                    p_viewer->removePointCloud("point_cloud");
                    p_viewer->addPointCloud(algo.getPointCloud(), rgb, "point_cloud");
                    p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
                }

                p_viewer->spinOnce(100);
            }
            algo.stop();
        }

        #pragma omp single nowait
        {
            algo.runMainLoop();
        }
    }

    return 0;
}
