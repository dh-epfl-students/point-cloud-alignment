#include <string>
#include <fstream>

#include <omp.h>

#include <Eigen/StdVector>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "common.h"
#include "plane_segmentation.h"
#include "plane_merging.h"
#include "registration.h"

#define MAX_CURVATURE 0.5f
#define MAX_VIEWPORT_NB 4

using namespace std;

struct AlignmentResults
{
    mat4 transform;
    vector<SegmentedPointsContainer::SegmentedPlane> source_planes;
};

/**
 * @brief The TestingSet struct contains a target and sources clouds of the same area
 * that will be aligned.
 */
struct TestingSet
{
    string target_file;
    vector<string> source_files;
    PointNormalKCloud::Ptr target_cloud;
    vector<PointNormalKCloud::Ptr> source_clouds;

    vector<SegmentedPointsContainer::SegmentedPlane> target_planes;
    vector<AlignmentResults, Eigen::aligned_allocator<AlignmentResults> > results;

    TestingSet(): target_file("") {}
    TestingSet(string target): target_file(target) {}
    void addSource(string source) { source_files.push_back(source); }
    bool isInitialized() { return !target_file.empty() && !source_files.empty(); }
};

/// The Viewer in which the results will be displayed
static pcl::visualization::PCLVisualizer::Ptr p_viewer;
static vector<TestingSet> testingSet;
static vector<int> viewports(4);
static size_t currTestingSet;

/**
 * @brief Perform plane segmentation and plane merging with given cloud
 * @param filename The point cloud file
 * @param isSource True if the given point cloud must be treated as as source, False if it is a target
 * @param out_planes Output the resulting segmented planes
 * @return A pointer to the loaded point cloud object
 */
static PointNormalKCloud::Ptr segmentCloud(string filename, bool isSource, vector<SegmentedPointsContainer::SegmentedPlane> &out_planes)
{
    vector<SegmentedPointsContainer::SegmentedPlane> segmented_planes;

    PlaneSegmentation segmentation;
    segmentation.init(filename, isSource);

    if(segmentation.isReady())
    {
        segmentation.filterOutCurvature(MAX_CURVATURE);
        segmentation.start_pause();
        segmentation.runMainLoop();
        segmented_planes = segmentation.getSegmentedPlanes();
    }
    else {
        cout << "Error: " << filename << " is not preprocessed. Exiting..." << endl;
        exit(EXIT_FAILURE);
    }

    PlaneMerging merger;
    merger.init(nullptr, false);
    merger.start_merge(segmented_planes, segmentation.getPointCloud());
    out_planes = merger.getSegmentedPlanes();

    return segmentation.getPointCloud();
}

static void testSet(TestingSet &set)
{
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            set.target_cloud = segmentCloud(set.target_file, false, set.target_planes);
        }

        #pragma omp section
        {
            set.source_clouds.resize(set.source_files.size());
            set.results.resize(set.source_files.size());

            // For now, only with cloud
            #pragma omp parallel for
            for(size_t i = 0; i < set.source_files.size(); ++i)
            {
                string source_file = set.source_files[i];

                vector<SegmentedPointsContainer::SegmentedPlane> source_planes;
                PointNormalKCloud::Ptr source_cloud;

                set.source_clouds[i] = segmentCloud(source_file, true, set.results[i].source_planes);
            }
        }
    }

    cout << "Plane segmentation finished, starting alignment..." << endl;

    #pragma omp parallel for
    for(size_t i = 0; i < set.source_clouds.size(); ++i)
    {
        auto source_planes = set.results[i].source_planes;
        auto source_cloud = set.source_clouds[i];

        // Registration step
        Registration registration;
        registration.setClouds(source_planes, set.target_planes, false, false, source_cloud, set.target_cloud);
        mat4 M = registration.findAlignment();
        mat4 finalM = M;
        bool realign = registration.applyTransform(M);

        if(realign)
        {
            finalM = M * finalM;
        }

        set.results[i].transform = finalM;

        //TODO: save alignment statistics in set.results[i]
    }
}

void displayTestSet(TestingSet &set)
{
    p_viewer->removeAllShapes();
    p_viewer->removeAllPointClouds();

    PointNormalKCloud::Ptr green_c = set.target_cloud;
    pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> green(green_c, 15, 255, 15);
    p_viewer->addPointCloud(green_c, green, "target_point_cloud");
    p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_point_cloud");

    for(size_t i = 0; i < min(MAX_VIEWPORT_NB, static_cast<int>(set.source_clouds.size())); ++i)
    {
        // Add to each viewport: target cloud in green, source cloud in red, aligned cloud in blue

        stringstream ss;
        ss << "source_cloud_vp" << i;
        string source_id = ss.str();

        stringstream ss2;
        ss2 << "aligned_cloud_vp" << i;
        string aligned_id = ss2.str();

        PointNormalKCloud::Ptr red_c = set.source_clouds[i];
        pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> red(red_c, 255, 15, 15);
        p_viewer->addPointCloud(red_c, red, source_id, viewports[i]);
        p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, source_id, viewports[i]);

        // Transform source cloud
        PointNormalKCloud::Ptr aligned_cloud(new PointNormalKCloud);
        pcl::transformPointCloudWithNormals(*red_c, *aligned_cloud, set.results[i].transform);
        pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> blue(aligned_cloud, 15, 15, 255);
        p_viewer->addPointCloud(aligned_cloud, blue, aligned_id, viewports[i]);
        p_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, aligned_id, viewports[i]);
    }

    p_viewer->resetCamera();
}

void keyboardCallback(const pcl::visualization::KeyboardEvent &event,
                      void* viewer_void)
{
    if(event.getKeySym() == "n" && event.keyDown())
    {
        // Go to next testing set
        displayTestSet(testingSet[positive_modulo(++currTestingSet, testingSet.size())]);
    }
    else if (event.getKeySym() == "b" && event.keyDown())
    {
        // Go to previous testing set
        displayTestSet(testingSet[positive_modulo(--currTestingSet, testingSet.size())]);
    }
}

int main()
{
    vector<TestingSet> testing_set;

    //Read test set file to fill testing set
    ifstream file("/home/loris/Documents/EPFL/Master/master-project-2019/Data/TestingSet/PCTestingSet.txt");

    string line;
    TestingSet set;
    bool nextIsTarget = false;

    while(getline(file, line))
    {
        if(line.compare("group") == 0)
        {
            nextIsTarget = true;

            if(set.isInitialized())
            {
                testing_set.push_back(set);
            }
        }
        else if(nextIsTarget)
        {
            set = TestingSet(line);
            nextIsTarget = false;
        }
        else if(!line.empty())
        {
            set.addSource(line);
        }
        else if(line.empty() && set.isInitialized())
        {
            testing_set.push_back(set);
        }
    }

    // Launch the tests for all testing set
    #pragma omp parallel for
    for(size_t i = 0; i < testing_set.size(); ++i)
    {
        testSet(testing_set[i]);
    }

    testingSet = testing_set;

    //TODO print stats in a file

    // Display final alignments
    p_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
    p_viewer->initCameraParameters();
    p_viewer->registerKeyboardCallback(keyboardCallback, static_cast<void*>(p_viewer.get()));

    // Setup viewer with 4 viewports
    p_viewer->createViewPort(0, 0, 0.5, 0.5, viewports[0]);
    p_viewer->createViewPort(0.5, 0, 1, 0.5, viewports[1]);
    p_viewer->createViewPort(0, 0.5, 0.5, 1, viewports[2]);
    p_viewer->createViewPort(0.5, 0.5, 1, 1, viewports[3]);

    // Fill viewer with first testing set results
    currTestingSet = 0;
    displayTestSet(testingSet[currTestingSet]);

    while(!p_viewer->wasStopped())
    {
        p_viewer->spinOnce(100);
    }

    return 0;
}
