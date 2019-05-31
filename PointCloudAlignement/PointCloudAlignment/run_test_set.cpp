#include <string>
#include <fstream>

#include <boost/algorithm/string.hpp>

#include <omp.h>

#include <Eigen/StdVector>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "common.h"
#include "plane_segmentation.h"
#include "plane_merging.h"
#include "mesh_segmentation.h"
#include "registration.h"
#include "test_set.h"

using namespace std;

/// The Viewer in which the results will be displayed
static pcl::visualization::PCLVisualizer::Ptr p_viewer;
static vector<TestingSet> testing_set;
static vector<int> viewports(4);
static size_t currTestingSet;


void displayTestSet(TestingSet &set)
{
    p_viewer->removeAllShapes();
    p_viewer->removeAllPointClouds();

    set.display(p_viewer, viewports);

    p_viewer->resetCamera();
}

void keyboardCallback(const pcl::visualization::KeyboardEvent &event,
                      void* viewer_void)
{
    if(event.getKeySym() == "n" && event.keyDown())
    {
        // Go to next testing set
        displayTestSet(testing_set[positive_modulo(++currTestingSet, testing_set.size())]);
    }
    else if (event.getKeySym() == "b" && event.keyDown())
    {
        // Go to previous testing set
        displayTestSet(testing_set[positive_modulo(--currTestingSet, testing_set.size())]);
    }
}

static bool parseTestingFile(string file_path)
{
    //Read test set file to fill testing set
    ifstream objectsFile(file_path);

    string line;
    TestingSet set;
    bool nextIsTarget = false;

    while(getline(objectsFile, line))
    {
        if(line.compare("group") == 0)
        {
            nextIsTarget = true;

            if(set.isInitialized())
            {
                testing_set.push_back(set);
            }
        }
        else if(line.empty() && set.isInitialized())
        {
            testing_set.push_back(set);
        }
        else
        {
            // In this case the line is composed by 'filename m/c' m=mesh and c=cloud
            // Thus, plit the line in two
            vector<string> tokens;
            boost::split(tokens, line, boost::is_any_of(" "), boost::token_compress_on);

            if(tokens.size() != 2)
            {
                cout << "Synthax Error in " << file_path << endl;
                return false;
            }

            bool isCloud = tokens[1] == "c";

            if(nextIsTarget)
            {
                set = TestingSet(tokens[0], isCloud);
                nextIsTarget = false;
            }
            else
            {
                set.addSource(tokens[0], isCloud);
            }
        }
    }

    return !testing_set.empty();
}

int main()
{
    if(!parseTestingFile("/home/loris/Documents/EPFL/Master/master-project-2019/Data/TestingSet/testing_set_list.txt"))
    {
        cout << "Error while parsing input file" << endl;
        exit(EXIT_FAILURE);
    }

    // Launch the tests for all testing set
    #pragma omp parallel for
    for(size_t i = 0; i < testing_set.size(); ++i)
    {
        testing_set[i].runTests();
    }

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
    displayTestSet(testing_set[currTestingSet]);

    while(!p_viewer->wasStopped())
    {
        p_viewer->spinOnce(100);
    }

    return EXIT_SUCCESS;
}
