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
    else if(event.getKeySym() == "t" && event.keyDown())
    {
        viewer->removePointCloud("point_cloud");
        pcl::visualization::PointCloudColorHandlerGenericField<PointNormalK> curv(algo.getPointCloud(), "curvature");
        viewer->addPointCloud(algo.getPointCloud(), curv, "point_cloud");
    }
    else if(event.getKeySym() == "z" && event.keyDown())
    {
        viewer->removePointCloud("point_cloud");
        pcl::visualization::PointCloudColorHandlerGenericField<PointNormalK> k(algo.getPointCloud(), "k");
        viewer->addPointCloud(algo.getPointCloud(), k, "point_cloud");
    }
    else if(event.getKeySym() == "i" && event.keyDown())
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
    else if(event.keyDown())
    {
        cout << event.getKeySym() << ", " << event.getKeyCode() << endl;
    }
}

void display_update_callback(PointNormalKCloud::Ptr p_cloud)
{
    p_viewer->updatePointCloud<PointNormalK>(p_cloud, "point_cloud");
}

void add_plane_callback(pcl::ModelCoefficients coeffs, float x, float y, float z)
{
    string plane_n = "plane" + plane_nb;
    p_viewer->addPlane(coeffs, x, y, z, plane_n);
    ++plane_nb;
}

function<void(PointNormalKCloud::Ptr)> display_update_callable = &display_update_callback;
function<void(pcl::ModelCoefficients, float, float, float)> add_plane_callable = &add_plane_callback;

// Start and setup viewer
pcl::visualization::PCLVisualizer::Ptr setupViewer()
{
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

    p_viewer = setupViewer();

    algo.init(pcFile);
    algo.setViewerUpdateCallback(display_update_callable);
    algo.setAddPlaneCallback(add_plane_callable);

    pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> single_color(algo.getPointCloud(), 0, 255, 0);
    p_viewer->addPointCloud(algo.getPointCloud(), single_color, "point_cloud");

    // Start plane segmentation thread
    #pragma omp parallel
    {
        #pragma omp master
        {
            // Drawing loop
            while(!p_viewer->wasStopped())
            {
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
