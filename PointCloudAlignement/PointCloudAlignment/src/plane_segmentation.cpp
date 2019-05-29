#include "plane_segmentation.h"

#include <algorithm>

// ========================================================================================== //
// PlaneSegmentation::RunProperties
// ========================================================================================== //

void PlaneSegmentation::RunProperties::setupNextPlane(int index, PointNormalK &p, ivec3 color, int plane_id)
{
    p_index = index;
    curr_color = color;
    plane = Plane();
    root_p = p;
    plane_nb = plane_id;
    iteration = 0;
    prev_size = 0;
    max_search_distance = 0;
    epsilon = 0;
    p_nghbrs_indices->clear();
    p_new_points_indices->clear();
}

void PlaneSegmentation::RunProperties::addToNeighborhood(vector<int> &new_points)
{
    if(iteration < PHASE1_ITERATIONS ||
            p_nghbrs_indices->size() < MIN_STABLE_SIZE)
    {
        cout << "Iteration " << iteration << ": Adding points to neighborhood list only." << endl;
        new_points.swap(*p_nghbrs_indices);
    }
    else
    {
        if(p_new_points_indices->empty())
        {
            cout << "Iteration " << iteration << ": First time adding to new point list." << endl;
            // It is the first time new_points vector will be used
            new_points.swap(*p_nghbrs_indices);
            p_new_points_indices->reserve(p_nghbrs_indices->size());
            p_new_points_indices->insert(p_new_points_indices->end(),
                                                 p_nghbrs_indices->begin(),
                                                 p_nghbrs_indices->end());
        }
        else
        {
            cout << "Iteration " << iteration << ": Swaping to new point list and adding to neighborhood list." << endl;

            new_points.swap(*p_new_points_indices);
            p_nghbrs_indices->reserve(p_nghbrs_indices->size() +
                                              p_new_points_indices->size());
            p_nghbrs_indices->insert(p_nghbrs_indices->end(),
                                             p_new_points_indices->begin(),
                                             p_new_points_indices->end());
        }
    }

    cout << "End of neighborhood add. Current status is " << p_nghbrs_indices->size() << " points in neighborhood and " << p_new_points_indices->size() << " points in new points list." << endl;
}

// ========================================================================================== //
// PlaneSegmentation
// ========================================================================================== //

int PlaneSegmentation::init(PointNormalKCloud::Ptr p_object, bool isSource)
{
    this->isSource = isSource;
    this->p_cloud = p_object;

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

    if(p_cloud->points[0].k == 0)
    {
        cout << "Loaded point cloud is not preprocessed." << endl;
        is_ready = false;
    }
    else
    {
        cout << "Loaded point cloud is already preprocessed." << endl;

        is_ready = true;
    }

    // Initialize remaining variables
    safety_distance = 0;
    curv_bound = 0.6f;

    // Initialize segmented_points_container
    p_segmented_points_container = SegmentedPointsContainer::Ptr(new SegmentedPointsContainer);

    // Fill segmented points container if the cloud was already segmented
    if(p_cloud->points[0].plane_id != -1)
    {
        fillSegmentedPointsContainer();
    }

    return EXIT_SUCCESS;
}

int PlaneSegmentation::init(string cloud_file, bool isSource)
{

    p_cloud = PointNormalKCloud::Ptr(new PointNormalKCloud);

    int r = pcl::io::loadPCDFile(cloud_file, *p_cloud);

    if(r == -1 && pcl::io::loadPLYFile(cloud_file, *p_cloud) == -1)
    {
        PCL_ERROR("Could not read given file\n");
        return EXIT_FAILURE;
    }

    cout << "Pointcloud containing " << p_cloud->points.size() << " points loaded." << endl;

    return this->init(p_cloud, isSource);
}

void PlaneSegmentation::preprocessCloud()
{
    if(is_ready) return;

    cout << "Starting normal, curvature and k computation." << endl;

    NormalComputation nc;
    nc.computeNormalCloud(p_cloud, p_kdtree, isResampled);

    cout << "Normal computation successfully ended." << endl;
    is_ready = true;
}

void PlaneSegmentation::resampleCloud()
{
    if(isResampled) return;

    is_ready = false;

    PointNormalKCloud::Ptr p_cloud_filtered(new PointNormalKCloud);

    pcl::VoxelGrid<PointNormalK>::Ptr filter(new pcl::VoxelGrid<PointNormalK>);
    filter->setInputCloud(p_cloud);
    filter->setLeafSize(1, 1, 1);
    filter->filter(*p_cloud_filtered);

    cout << "Cloud filtered from " << p_cloud->size() << " to " << p_cloud_filtered->size() << " points." << endl;

    p_cloud->clear();
    p_cloud = p_cloud_filtered;

    p_indices->resize(p_cloud->size());
    p_kdtree->setInputCloud(p_cloud, p_indices);

    isResampled = true;
}

void PlaneSegmentation::setViewerUpdateCallback(function<void(PointNormalKCloud::Ptr, ivec3, vector<int>, bool)> callable)
{
    display_update_callable = callable;
}

void PlaneSegmentation::setAddPlaneCallback(function<void(pcl::ModelCoefficients, float, float, float)> callable)
{
    add_plane_callable = callable;
}

void PlaneSegmentation::setUpdateNormalCloudCallback(function<void(void)> callable)
{
    update_normal_cloud_callable = callable;
}

bool PlaneSegmentation::isReady()
{
    return this->is_ready;
}

void PlaneSegmentation::start_pause()
{
    if(!is_ready)
    {
        cout << "Can't start algorithm, not initialized properly" << endl;
        return;
    }

    if(isSegmented)
    {
        cout << "The cloud is already segmented." << endl;
        return;
    }

    is_started = !is_started;
}

void PlaneSegmentation::runMainLoop()
{
    int index = 0;
    while(dont_quit)
    {
        while(is_started && (index = getRegionGrowingStartLocation()) != -1)
        {
            cout << "Starting new plane segmentation from index: " << index << endl;

            // Start of a new plane segmentation
            // -> reinitialise variables
            int new_plane_id = p_segmented_points_container->getNbPlanes() + 1;
            current_run.setupNextPlane(index, p_cloud->points[index], p_segmented_points_container->getNextPlaneColor(), new_plane_id);
            segmentPlane();
        }

        if(index == -1)
        {
            dont_quit = false;
            isSegmented = true;
            cout << "Segmented " << p_segmented_points_container->getNbPlanes() << " planes. Excluded " << p_segmented_points_container->getNbOfExcludedPoints() << endl;
        }
    }
}

void PlaneSegmentation::runOneStep()
{
    if(isSegmented)
    {
        cout << "The cloud is already segmented." << endl;
        return;
    }

    int index;

    if(is_plane_initialized)
    {
        is_plane_initialized = regionGrowthOneStep();
    }
    else if((index = getRegionGrowingStartLocation()) != -1)
    {
        int new_plane_id = p_segmented_points_container->getNbPlanes() + 1;
        current_run.setupNextPlane(index, p_cloud->points[index], p_segmented_points_container->getNextPlaneColor(), new_plane_id);
        initRegionGrowth();
        is_plane_initialized = true;
    }
}

void PlaneSegmentation::stop()
{
    is_started = false;
    dont_quit = false;
}

bool PlaneSegmentation::initRegionGrowth()
{
    current_run.p_nghbrs_indices->clear();

    // Get first neighborhood
    vector<float> sqr_distances(current_run.root_p.k);

    p_kdtree->nearestKSearch(current_run.root_p, current_run.root_p.k,
                             *current_run.p_nghbrs_indices,
                             sqr_distances);

    // Compute mean of min distances in the neighborhood
    current_run.max_search_distance = 3.0f * getMeanOfMinDistances();

    float dist_to_kth = std::sqrt(sqr_distances[current_run.root_p.k - 1]);

    // At 10th iteration, we ensure that we are not trying to
    // segment an already treated zone.
    if(current_run.plane_nb > 10 &&
            (dist_to_kth > 3 * safety_distance / static_cast<float>(current_run.plane_nb)))
    {
        // This means that the algo is looking an area
        // that has probably been already processed and is
        // trying to compute the plane of the remaining points
        // that have not been accepted in the plane previously.

        // Thus, we add the points to exclusion list since we don't want to
        // consider them again.

        cout << "Trying to start segmentation in already classified area. Point index: " << current_run.p_index << endl;

        // Add to exclusion list
        exclude_points(*current_run.p_nghbrs_indices);
        p_segmented_points_container->addExcludedPoints(*current_run.p_nghbrs_indices);

        return false;
    }

    // Update safety distance.
    safety_distance += dist_to_kth;

    // Display selected points in green and root point in blue
    color_points(*current_run.p_nghbrs_indices, ivec3(15, 255, 15));
    color_point(current_run.p_index, ivec3(15, 15, 255));

    cout << "Starting plane " << current_run.plane_nb << " at index " << current_run.p_index << endl;

    return true;
}

void PlaneSegmentation::performRegionGrowth()
{
    while(is_started)
    {
        if(!regionGrowthOneStep()) return;
    }
}

bool PlaneSegmentation::regionGrowthOneStep()
{
    cout << "Plane " << current_run.plane_nb << ": iteration " << current_run.iteration << " current size " << current_run.p_nghbrs_indices->size() << endl;

    // Check for termination conditions
    if((p_segmented_points_container->getNbOfSegmentedPoints() > 0.9 * p_cloud->size()) ||
            (p_indices->size() == 0))
    {
        cout << "Majority of points have been segmented -> Success" << endl;

        // majority of points have been segmented -> Success
        stop();
        return false;
    }

    current_run.prev_size = current_run.p_nghbrs_indices->size();

    // If first 3 iterations, check area for valid plane
    if(current_run.iteration == PHASE1_ITERATIONS)
    {
        if(!PFHEvaluation::isValidPlane(p_cloud, *current_run.p_nghbrs_indices))
        {
            cout << "Current plane is invalid" << endl;
            // Add to exclusion
            exclude_points(*current_run.p_nghbrs_indices);
            p_segmented_points_container->addExcludedPoints(*current_run.p_nghbrs_indices);

            return false;
        }

        cout << "Current plane is valid" << endl;

        // We are on a plane -> increase search radius to speed things up
        current_run.max_search_distance *= 2.0f;
        //current_run.epsilon *= 2.0f;
    }

    // Check if neighborhood has shrinked
    if(planeHasShrinked())
    {
        cout << "Current plane has shrinked" << endl;

        // Exclude starting point
        vector<int> rootP;
        rootP.push_back(current_run.p_index);
        exclude_points(rootP);
        p_segmented_points_container->addExcludedPoint(current_run.p_index);

        return false;
    }

    // Compute current plane
    if(current_run.iteration < PHASE1_ITERATIONS ||
            current_run.p_nghbrs_indices->size() < MIN_STABLE_SIZE)
    {
        cout << "Computing new plane parameters" << endl;
        Plane curr_plane;
        Plane::estimatePlane(p_cloud, current_run.p_nghbrs_indices, curr_plane);
        current_run.plane = curr_plane;

        // Update epsilon
        current_run.epsilon = curr_plane.getPlaneTolerance(p_cloud, current_run.p_nghbrs_indices);
    }

    // Find new candidates
    vector<int> candidates;
    if(current_run.iteration <= PHASE1_ITERATIONS
        || current_run.p_nghbrs_indices->size() < MIN_STABLE_SIZE
        || current_run.p_new_points_indices->empty())
    {
        getNeighborsOf(current_run.p_nghbrs_indices, current_run.max_search_distance, candidates);
    }
    else
    {
        getNeighborsOf(current_run.p_new_points_indices, current_run.max_search_distance, candidates);
    }

    cout << "Found " << candidates.size() << " candidates." << endl;
    cout << "Epsilon = " << current_run.epsilon << endl;

    // Test them with current plane
    // TODO: Move that to its own function
    vector<int> points_in_plane;

    for(size_t i = 0; i < candidates.size(); ++i)
    {
        int index = candidates[i];
        if(current_run.plane.pointInPlane(p_cloud->points[index], current_run.epsilon))
        {
            if(current_run.iteration >= PHASE1_ITERATIONS)
            {
                if(current_run.plane.normalInPlane(p_cloud->points[index], MAX_NORMAL_ANGLE))
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

    cout << "Number of candidates in plane: " << points_in_plane.size() << endl;

    // Add good candidates to neighborhood
    current_run.addToNeighborhood(points_in_plane);

    // Update available Indices
    if(current_run.p_nghbrs_indices->size() >= MIN_STABLE_SIZE && current_run.iteration >= PHASE1_ITERATIONS)
    {
        exclude_from_search(*current_run.p_new_points_indices);

        // Color added points in green
        color_points(*current_run.p_new_points_indices, ivec3(15, 255, 15));
    }
    else
    {
        // Color added points in green
        color_points(*current_run.p_nghbrs_indices, ivec3(15, 255, 15));
    }

    // Check for region growth stop
    // Either the region stopped to grow naturally, or we are stuck in an infinite loop and we exit at iteration MAX_ITERATIONS
    if(((current_run.iteration > PHASE1_ITERATIONS) &&
            (current_run.prev_size == current_run.p_nghbrs_indices->size())) ||
            (current_run.iteration == MAX_ITERATIONS))
    {
        cout << "Plane growth stopped, registering plane containing " << current_run.p_nghbrs_indices->size() << " points." << endl;

        // Computing the plane geometric center
        current_run.plane.setCenter(computePlaneCenter(p_cloud, *current_run.p_nghbrs_indices));

        // Set plane_id of segmented planes
        #pragma omp parallel for
        for(size_t i = 0; i < current_run.p_nghbrs_indices->size(); ++i)
        {
            p_cloud->points[current_run.p_nghbrs_indices->at(i)].plane_id = current_run.plane_nb;
        }

        // store segmented plane
        SegmentedPointsContainer::SegmentedPlane plane(current_run.plane_nb, current_run.curr_color, *current_run.p_nghbrs_indices, current_run.plane);
        p_segmented_points_container->addSegmentedPoints(plane);
        color_points(*current_run.p_nghbrs_indices, current_run.curr_color);
        exclude_from_search(*current_run.p_nghbrs_indices);

        return false;
    }

    cout << "End of iteration nb " << current_run.iteration << ". New nb of points in plane: " << current_run.p_nghbrs_indices->size() << endl;
    current_run.iteration++;

    return true;
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
    if(p_indices->size() == 0)
    {
        cout << "No available index. Segmentation stopped." << endl;
        return -1;
    }

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
    //cout << "Searching distance : " << search_d << endl;

    vector<vector<int>> candidates_lists(indices_in->size());
    size_t total_size = 0;

    #pragma omp parallel for shared(candidates_lists, total_size)
    for(size_t i = 0; i < indices_in->size(); ++i)
    {
        int p_id = indices_in->at(i);
        vector<float> distances;
        //p_kdtree->radiusSearch(p_cloud->points[p_id], search_d, candidates_lists[i], distances);

        // Nearest K search is way faster than radius search for kdtrees
        p_kdtree->nearestKSearch(p_cloud->points[p_id], p_cloud->points[p_id].k, candidates_lists[i], distances);

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

void PlaneSegmentation::exclude_from_search(vector<int> &indices)
{
    // Ensure that the given list is sorted
    sort(indices.begin(), indices.end());

    // Remove points from available indices list
    boost::shared_ptr<vector<int>> tmp(new vector<int>(0));
    set_difference(p_indices->begin(), p_indices->end(), indices.begin(), indices.end(), back_inserter(*tmp));
    p_indices = tmp;

    cout << "Adding " << indices.size() << " to exclusion list" << endl;

    if(p_indices->size() == 0) return;

    // Update searching tree indices
    p_kdtree->setInputCloud(p_cloud, p_indices);
}

void PlaneSegmentation::exclude_points(vector<int> indices)
{
    if(indices.empty()) return;

    // Add points to exclusion list
    p_excluded_indices->insert(p_excluded_indices->end(), indices.begin(), indices.end());

    // Remove points from search tree
    exclude_from_search(indices);

    // Update point display
    this->callDisplayCallback(p_cloud, p_segmented_points_container->getMiscColor(), indices, isSource);
}

void PlaneSegmentation::filterOutCurvature(float max_curvature)
{
    // Fill vector of points' indices to exclude
    vector<int> indices;
    copy_if(p_indices->begin(), p_indices->end(), back_inserter(indices), [&max_curvature, this](int index){
        return this->p_cloud->points[index].curvature > max_curvature;
    });

    exclude_points(indices);
    p_segmented_points_container->addExcludedPoints(indices);
}

void PlaneSegmentation::color_points(vector<int> indices, ivec3 color)
{
    this->callDisplayCallback(p_cloud, color, indices, isSource);
}

void PlaneSegmentation::color_point(int index, ivec3 color)
{
    vector<int> indices;
    indices.push_back(index);
    this->callDisplayCallback(p_cloud, color, indices, isSource);
}

PointNormalKCloud::Ptr PlaneSegmentation::getAvailablePointCloud()
{
    PointNormalKCloud::Ptr filtered(new PointNormalKCloud);
    pcl::ExtractIndices<PointNormalK> extract;
    extract.setInputCloud(p_cloud);
    extract.setIndices(p_indices);
    extract.setNegative(false);
    extract.filter(*filtered);
    return filtered;
}

PointNormalKCloud::Ptr PlaneSegmentation::getExcludedPointCloud()
{
    PointNormalKCloud::Ptr filtered(new PointNormalKCloud);
    pcl::ExtractIndices<PointNormalK> extract;
    extract.setInputCloud(p_cloud);
    extract.setIndices(p_indices);
    extract.setNegative(true);
    extract.filter(*filtered);
    return filtered;
}

bool PlaneSegmentation::planeHasShrinked()
{
    return current_run.p_nghbrs_indices->size() < MIN_PLANE_SIZE;
}

void PlaneSegmentation::reorient_normals(PointNormalKCloud::Ptr cloud_in, vector<int> indices, vec3 pn)
{
    #pragma omp parallel for
    for(size_t i = 0; i < indices.size(); ++i)
    {
        vec3 n = vec3(cloud_in->points[indices[i]].normal_x,
                      cloud_in->points[indices[i]].normal_y,
                      cloud_in->points[indices[i]].normal_z);

        n = (acos(pn.dot(n)) <= acos(pn.dot(-n)))? n : -n;

        cloud_in->points[indices[i]].normal_x = n.x();
        cloud_in->points[indices[i]].normal_y = n.y();
        cloud_in->points[indices[i]].normal_z = n.z();
    }

    update_normal_cloud_callable();
}

float PlaneSegmentation::getCurvBound()
{
    return curv_bound;
}

void PlaneSegmentation::fillSegmentedPointsContainer()
{
    if(isSegmented) return;

    bool allSegmented = true;

    // For each point: read it's plane_id. If excluded simply add it to exclusion list
    //          If not:
    //              If the plane is already created -> Add the point index.
    //              Else -> Create the plane container and add the point.
    for(int index: *p_indices)
    {
        if(p_cloud->points[index].plane_id == 0)
        {
            p_segmented_points_container->addExcludedPoint(index);
        }
        else if(p_cloud->points[index].plane_id > 0)
        {
            if(p_segmented_points_container->getNbPlanes() < p_cloud->points[index].plane_id)
            {
                ivec3 c(p_cloud->points[index].r,
                        p_cloud->points[index].g,
                        p_cloud->points[index].b);
                p_segmented_points_container->createPlane(p_cloud->points[index].plane_id, c);
                p_segmented_points_container->addSegmentedPoint(p_cloud->points[index].plane_id, index);
            }
        }
        else
        {
            // Not segmented
            allSegmented = false;
        }
    }

    if(allSegmented)
    {
        //TODO: compute segmented plane parameters

        isSegmented = true;
    }
    else
    {
        // TODO: Setup cloud to resume segmentation.
        //      - Set available indices list for kdtree
    }
}

void PlaneSegmentation::callDisplayCallback(PointNormalKCloud::Ptr p_cloud, ivec3 c, vector<int> indices, bool isSource)
{
    if(this->display_update_callable != nullptr)
    {
        this->display_update_callable(p_cloud, c, indices, isSource);
    }
}
