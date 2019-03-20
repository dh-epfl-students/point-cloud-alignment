#include "plane_segmentation.h"

#include <algorithm>

//#include <fstream>

// ========================================================================================== //
// PlaneSegmentation::RunProperties
// ========================================================================================== //

void PlaneSegmentation::RunProperties::setupNextPlane(PointNormalK &p)
{
    plane = Plane();
    root_p = p;
    plane_nb++;
    iteration = 0;
    prev_size = 0;
    max_search_distance = 0;
    epsilon = 0;
    p_nghbrs_indices->clear();
    p_new_points_indices->clear();
    n = vec3(0, 0, 0);
}

void PlaneSegmentation::RunProperties::addToNeighborhood(vector<int> &new_points)
{
    if(iteration < PHASE1_ITERATIONS ||
            p_nghbrs_indices->size() < MIN_STABLE_SIZE)
    {
        new_points.swap(*p_nghbrs_indices);
    }
    else
    {
        if(p_new_points_indices->empty())
        {
            // It is the first time new_points vector will be used
            new_points.swap(*p_nghbrs_indices);
            p_new_points_indices->reserve(p_nghbrs_indices->size());
            p_new_points_indices->insert(p_new_points_indices->end(),
                                                 p_nghbrs_indices->begin(),
                                                 p_nghbrs_indices->end());
        }
        else
        {
            new_points.swap(*p_new_points_indices);
            p_nghbrs_indices->reserve(p_nghbrs_indices->size() +
                                              p_new_points_indices->size());
            p_nghbrs_indices->insert(p_nghbrs_indices->end(),
                                             p_new_points_indices->begin(),
                                             p_new_points_indices->end());
        }
    }
}

// ========================================================================================== //
// PlaneSegmentation
// ========================================================================================== //

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

    p_excluded_indices = boost::shared_ptr<vector<int>>(new vector<int>(0));
    p_indices = boost::shared_ptr<vector<int>>(new vector<int>(p_cloud->points.size()));

    #pragma omp parallel for
    for(size_t i = 0; i < p_cloud->points.size(); ++i)
    {
        p_indices->at(i) = static_cast<int>(i);
    }

    cout << "PointIndices is filled" << endl;

    // Fill kdtree search strucuture
    p_kdtree = KdTreeFlannK::Ptr(new KdTreeFlannK);
    p_kdtree->setInputCloud(p_cloud);

    cout << "Starting to compute normal cloud..." << endl;

    NormalComputation nc;
    nc.computeNormalCloud(p_cloud, p_kdtree);

    cout << "Normal computation successfully ended." << endl;

    // Initialize remaining variables
    safety_distance = 0;

    //TODO: initialize segmented_points_container

    is_ready = true;
    return EXIT_SUCCESS;
}

void PlaneSegmentation::setViewerUpdateCallback(function<void(PointNormalKCloud::Ptr)> callable)
{
    display_update_callable = callable;
}

void PlaneSegmentation::setAddPlaneCallback(function<void(pcl::ModelCoefficients, float, float, float)> callable)
{
    add_plane_callable = callable;
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
            cout << "Starting new plane segmentation from index: " << index << endl;

            // Start of a new plane segmentation
            // -> reinitialise variables
            current_run.setupNextPlane(p_cloud->points[index]);

            segmentPlane();
        }
    }
}

void PlaneSegmentation::stop()
{
    is_started = false;
    dont_quit = false;
}

void PlaneSegmentation::stop_current_plane_segmentation()
{   
    //TODO: dunno
    //is_started = false;
}

bool PlaneSegmentation::initRegionGrowth()
{
    // Get first neighborhood
    vector<float> sqr_distances(current_run.root_p.k);
    p_kdtree->nearestKSearch(current_run.root_p, current_run.root_p.k,
                             *current_run.p_nghbrs_indices,
                             sqr_distances);

    // Compute mean of min distances in the neighborhood
    current_run.max_search_distance = 2.0f * getMeanOfMinDistances();

    float dist_to_kth = std::sqrt(sqr_distances[current_run.root_p.k]);

    // At 10th iteration, we ensure that we are not trying to
    // segment an already treated zone.
    if(current_run.plane_nb > 10 &&
            (dist_to_kth > 3 * safety_distance / (float)current_run.plane_nb))
    {
        // This means that the algo is looking an area
        // that has probably been already processed and is
        // trying to compute the plane of the remaining points
        // that have not been accepted in the plane previously.

        // Thus, we add the points to exclusion list since we don't want to
        // consider them again.

        //TODO: Add to exclusion list
        exclude_points(*current_run.p_nghbrs_indices);

        return false;
    }

    // Update safety distance.
    safety_distance += dist_to_kth;

    return true;
}

void PlaneSegmentation::performRegionGrowth()
{
    while(is_started)
    {
        cout << "Segmenting plane " << current_run.plane_nb << ": iteration " << current_run.iteration << endl;

        //TODO: Check for termination conditions
        if(segmented_points_container.getNbOfSegmentedPoints() > 0.8 * p_cloud->size())
        {
            //Stop, majority of points have been segmented -> Success
            stop();
            return;
        }

        current_run.prev_size = current_run.p_nghbrs_indices->size();

        //TODO: If first 3 iterations, check area for valid plane
        if(current_run.iteration == PHASE1_ITERATIONS)
        {
            if(!PFHEvaluation::isValidPlane(p_cloud, *current_run.p_nghbrs_indices))
            {
                // Add to exclusion
                exclude_points(*current_run.p_nghbrs_indices);

                stop_current_plane_segmentation();
                return;
            }

            // We are on a plane -> increase search radius to speed things up
            current_run.max_search_distance *= 2.0f;
            current_run.epsilon *= 2.0f;
        }

        //TODO: Check if neighborhood has shrinked
        if(current_run.p_nghbrs_indices->size() < 3)
        {
            exclude_points(*current_run.p_nghbrs_indices);
            stop_current_plane_segmentation();
            return;
        }

        //TODO: Compute current plane
        if(current_run.p_nghbrs_indices->size() < MIN_STABLE_SIZE)
        {
            Plane curr_plane;
            Plane::estimatePlane(p_cloud, current_run.p_nghbrs_indices, curr_plane);
            current_run.plane = curr_plane;

            //TODO: Update normal and epsilon
            current_run.n = curr_plane.getNormal();
            current_run.epsilon = 2.0f * curr_plane.getStdDevWith(p_cloud, current_run.p_nghbrs_indices);

            // Display plane
            if(current_run.iteration == 0)
            {
                add_plane_callable(current_run.plane.getModelCoefficients(), current_run.root_p.x, current_run.root_p.y, current_run.root_p.z);
            }
        }

        //TODO: Find new candidates
        vector<int> candidates;
        if(current_run.iteration < PHASE1_ITERATIONS 
            || current_run.p_nghbrs_indices->size() < MIN_STABLE_SIZE
            || current_run.p_new_points_indices->empty())
        {
            getNeighborsOf(current_run.p_nghbrs_indices, current_run.max_search_distance, candidates);
        }
        else
        {
            getNeighborsOf(current_run.p_new_points_indices, current_run.max_search_distance, candidates);
        }

        //TODO: Test them with current plane
        //TODO: Move that to its own function
        vector<int> points_in_plane;

        for(size_t i = 0; i < candidates.size(); ++i)
        {
            int index = candidates[i];
            if(current_run.plane.pointInPlane(p_cloud->points[index], current_run.epsilon))
            {
                if(current_run.iteration > PHASE1_ITERATIONS)
                {
                    if(current_run.plane.normalInPlane(p_cloud->points[index], max_normal_angle))
                    {
                        points_in_plane.push_back(index);
                    }
                }
                else
                {
                    points_in_plane.push_back(index);
                }
            }
        }

        //TODO: Add good candidates to neighborhood
        current_run.addToNeighborhood(points_in_plane);

        //TODO: Update available Indices
        if(current_run.p_nghbrs_indices->size() > MIN_STABLE_SIZE)
        {
            exclude_points(*current_run.p_new_points_indices);
        }

        //TODO: Check for region growth
        if(current_run.prev_size == current_run.p_nghbrs_indices->size())
        {
            cout << "Plane growth stopped" << endl;
            //TODO: store segmented plane
        }
        else
        {
            current_run.iteration++;
        }
    }
}

void PlaneSegmentation::segmentPlane()
{
    cout << "Thread " << omp_get_thread_num() << " starting segmentation." << endl;

    if(initRegionGrowth())
    {
        performRegionGrowth();
    }
}

float PlaneSegmentation::getMeanOfMinDistances()
{
    const int K(2);
    float acc(0);

    #pragma omp parallel for shared(acc)
    for(size_t i = 0; i < current_run.p_nghbrs_indices->size(); ++i)
    {
        int p_id = current_run.p_nghbrs_indices->at(i);
        vector<int> indices(K);
        vector<float> sqrd_distances(K);
        p_kdtree->nearestKSearch(p_cloud->points[p_id], K, indices, sqrd_distances);
        float dist = std::sqrt(sqrd_distances[1]);

        #pragma omp critical
        acc += dist;
    }

    return acc / current_run.p_nghbrs_indices->size();
}

int PlaneSegmentation::getRegionGrowingStartLocation()
{
    if(p_indices->size() == 0) return -1;

    // Copy indices
    vector<int> tmp_indices(*p_indices);
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

void PlaneSegmentation::getNeighborsOf(boost::shared_ptr<vector<int>> indices_in, float search_d, vector<int> &indices_out)
{
    vector<vector<int>> candidates_lists(indices_in->size());
    size_t total_size = 0;

    #pragma omp parallel for shared(candidates_lists, total_size)
    for(size_t i = 0; i < indices_in->size(); ++i)
    {
        int p_id = indices_in->at(i);
        vector<float> distances;
        //p_kdtree->radiusSearch(p_cloud->points[p_id], search_d, candidates_lists[i], distances);
        p_kdtree->nearestKSearchT(p_cloud->points[p_id], p_cloud->points[p_id].k, candidates_lists[i], distances);

        #pragma omp critical
        total_size += candidates_lists[i].size();
    }

    indices_out.reserve(total_size);

    // Merging all lists of candidates in one list
    for(int j = 0; j < candidates_lists.size(); ++j)
    {
        indices_out.insert(indices_out.end(), candidates_lists[j].begin(), candidates_lists[j].end());
    }

    // Remove duplicates
    sort(indices_out.begin(), indices_out.end());
    vector<int>::iterator it = unique(indices_out.begin(), indices_out.end());
    indices_out.resize(std::distance(indices_out.begin(), it));
}

void PlaneSegmentation::exclude_points(vector<int> indices)
{
    if(indices.empty()) return;

    // Ensure that the given list is sorted
    sort(indices.begin(), indices.end());

    // Add points to exclusion list
    p_excluded_indices->insert(p_excluded_indices->end(), indices.begin(), indices.end());

    // Remove points from available indices list
    boost::shared_ptr<vector<int>> tmp(new vector<int>(0));
    set_difference(p_indices->begin(), p_indices->end(), indices.begin(), indices.end(), back_inserter(*tmp));
    p_indices = tmp;

    // Update searching tree indices
    p_kdtree->setInputCloud(p_cloud, p_indices);
}
