#include "plane_segmentation.h"

#include <algorithm>

//#include <fstream>

int PlaneSegmentation::init(string cloud_file)
{
    p_cloud = PointNormalKCloud::Ptr(new PointNormalKCloud);

    int r = pcl::io::loadPCDFile(cloud_file, *p_cloud);

    if(r == -1 && pcl::io::loadPLYFile(cloud_file, *p_cloud) == -1)
    {
        PCL_ERROR("Could not read given file\n");
        return EXIT_FAILURE;
    }

    cout << "Pointcloud containing " << p_cloud->points.size() << " points loaded." << endl;

    p_excluded_indices = PointIndices::Ptr(new PointIndices);
    p_indices = PointIndices::Ptr(new PointIndices);
    p_indices->indices.resize(p_cloud->points.size());

    #pragma omp parallel for
    for(int i = 0; i < p_cloud->points.size(); ++i)
    {
        p_indices->indices[i] = i;
    }

    cout << "PointIndices is filled" << endl;

    // Fill kdtree search strucuture
    p_kdtree = KdTreeFlannK::Ptr(new KdTreeFlannK);
    p_kdtree->setInputCloud(p_cloud);

    cout << "Starting to compute normal cloud..." << endl;

    NormalComputation nc;
    nc.computeNormalCloud(p_cloud, p_kdtree);

    cout << "Normal computation successfully ended." << endl;

    is_ready = true;
    return EXIT_SUCCESS;
}

void PlaneSegmentation::setViewerUpdateCallback(function<void(PointNormalKCloud::Ptr)> callable)
{
    display_update_callable = callable;
}

bool PlaneSegmentation::isReady()
{
    return this->is_ready;
}

void PlaneSegmentation::start_pause()
{
    if(!is_ready) {
        cout << "Can't start algorithm, not initialized properly" << endl;
        return;
    }

    is_started = !is_started;
}

void PlaneSegmentation::runMainLoop()
{
    int index;
    while(dont_quit)
    {
        while(is_started && (index = getRegionGrowingStartLocation()) != -1)
        {
            segmentPlane(index);
        }
    }
}

void PlaneSegmentation::stop()
{
    is_started = false;
    dont_quit = false;
}

void PlaneSegmentation::performFirstPhase(int start_index)
{

}

void PlaneSegmentation::performSndPhase()
{

}

void PlaneSegmentation::segmentPlane(int start_index)
{
    cout << "Coucou from thread: " << omp_get_thread_num() << ". Starting segmentation from index " << start_index << endl;

    performFirstPhase(start_index);
    performSndPhase();

    is_started = false;
}

int PlaneSegmentation::getRegionGrowingStartLocation()
{
    if(p_indices->indices.size() == 0) return -1;

    // Copy indices
    vector<int> tmp_indices(p_indices->indices);
    sort(tmp_indices.begin(), tmp_indices.end(), [this](const int &lhs, const int &rhs){
        return this->p_cloud->points[lhs].curvature < this->p_cloud->points[rhs].curvature;
    });

    /*
    ofstream myfile;
    myfile.open ("/home/loris/Documents/EPFL/Master/master-project-2019/Data/Plots/curvature.txt");
    int j = 1;
    for_each(tmp_indices.begin(), tmp_indices.end(), [this, &myfile, &j](const int &i){
        myfile << j << ";" << this->p_cloud->points[i].curvature << endl;
        ++j;
    });
    myfile.close();
    */

    return tmp_indices.at(0);
}
