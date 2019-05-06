#include "plane_merging.h"

void PlaneMerging::init(function<void(PointNormalKCloud::Ptr, ivec3, vector<int>)> callable)
{
    display_update_callable = callable;

    p_kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    p_plane_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void PlaneMerging::start_merge(vector<SegmentedPointsContainer::SegmentedPlane> &p_list, PointNormalKCloud::Ptr p_cloud)
{
    // We assume that the plane list must be sorted by plane id order.
    this->plane_list = p_list;

    p_point_cloud = p_cloud;

    //fill center clouds and indices list
    p_plane_cloud->points.reserve(p_list.size());
    p_plane_indices = boost::shared_ptr<vector<int>>(new vector<int>);
    p_plane_indices->reserve(p_list.size());
    int i(0);

    for(auto p: p_list)
    {
        p_plane_cloud->points.push_back(p.plane.getCenterPCL());
        p_plane_indices->push_back(i);
        ++i;
    }

    //fill search tree with centers
    p_kdtree->setInputCloud(p_plane_cloud);

    merge();

    // Remove merged planes from the list of planes
    vector<SegmentedPointsContainer::SegmentedPlane> new_list;
    for_each(p_plane_indices->begin(), p_plane_indices->end(), [&new_list, this](int index){
        new_list.push_back(this->plane_list[index]);
    });
    plane_list.swap(new_list);

    isMerged = true;
}

void PlaneMerging::filter_small_planes(vector<SegmentedPointsContainer::SegmentedPlane> &p_list, int min_size)
{
    remove_if(p_list.begin(), p_list.end(), [min_size](SegmentedPointsContainer::SegmentedPlane p){
        return p.indices_list.size() < static_cast<size_t>(min_size);
    });
}

void PlaneMerging::merge()
{
    cout << "Starting to merge" << endl;

    bool continue_merging = false;

    for(size_t i = 0; i < plane_list.size(); ++i)
    {
        if(find(merged_planes_indices.begin(), merged_planes_indices.end(), i) == merged_planes_indices.end())
        {
            SegmentedPointsContainer::SegmentedPlane plane = plane_list[i];

            // Search nearest neighbors plane centers
            vector<int> indices;
            vector<float> sqrd_dists;
            p_kdtree->nearestKSearch(plane.plane.getCenterPCL(), KNN, indices, sqrd_dists);

            vector<int> remove_list;

            // Test all planes
            for(int j: indices)
            {
                if(plane_list[j].id != plane.id)
                {
                    //May need to reorient normal
                    vec3 n = plane.plane.getNormal().normalized();
                    vec3 ni = plane_list[j].plane.getNormal().normalized();
                    n = ni.dot(n) >= ni.dot(-n) ? n : -n;

                    // Filter by plane normal vector and then plane overlap
                    if(ni.dot(n) >= cos(NORMAL_ERROR) && planeOverlap(plane, plane_list[j]))
                    {
                        cout << "Plane " << plane.id << " is merged with plane " << plane_list[j].id << endl;

                        //  - change color of points
                        display_update_callable(p_point_cloud, plane.color, plane_list[j].indices_list);

                        //TODO:  - change plane_id for merged points

                        //  - merge indices lists
                        plane_list[i].indices_list.insert(plane_list[i].indices_list.end(), plane_list[j].indices_list.begin(), plane_list[j].indices_list.end());

                        //  - recompute center
                        plane_list[i].plane.setCenter(computePlaneCenter(p_point_cloud, plane_list[i].indices_list));
                        p_plane_cloud->points[i] = plane_list[i].plane.getCenterPCL();

                        //  - recompute plane parameters??? -> not for now

                        //  - add merged plane id to remove list
                        remove_list.push_back(j);
                        merged_planes_indices.push_back(j);
                    }
                }
            }

            if(!remove_list.empty())
            {
                // Update available plane indices
                vector<int> new_list;
                sort(remove_list.begin(), remove_list.end());
                set_difference(p_plane_indices->begin(), p_plane_indices->end(), remove_list.begin(), remove_list.end(), back_inserter<vector<int>>(new_list));
                p_plane_indices->swap(new_list);

                // Update plane list by removing merged planes from search tree
                p_kdtree->setInputCloud(p_plane_cloud, p_plane_indices);

                continue_merging = true;
            }
        }
    }

    if(continue_merging)
    {
        merge();
    }
    else
    {
        //TODO: update number of planes
        cout << "Plane merging finished: merged " << p_plane_cloud->size() - p_plane_indices->size() <<
                " planes. There is now " << p_plane_indices->size() << " planes." << endl;
    }
}

bool PlaneMerging::planeOverlap(SegmentedPointsContainer::SegmentedPlane &p1, SegmentedPointsContainer::SegmentedPlane &p2, float d_tolerance)
{
    vec3 dir1 = p2.plane.getCenter() - p1.plane.getCenter();
    vec3 dir2 = p1.plane.getCenter() - p2.plane.getCenter();

    float d = dir1.norm();

    // Select points to consider between both center points.
    float r1 = farestPointInDir(p1, dir1);
    float r2 = farestPointInDir(p2, dir2);

    // Compare distances to find overlap
    return d <= r1 + r2 + d_tolerance;
}

float PlaneMerging::farestPointInDir(SegmentedPointsContainer::SegmentedPlane &plane, vec3 dir)
{
    float max_r(0);
    dir.normalize();

    for(int i: plane.indices_list)
    {
        vec3 pi(p_point_cloud->points[i].x, p_point_cloud->points[i].y, p_point_cloud->points[i].z);
        vec3 p_dir = pi - plane.plane.getCenter();
        float p_dir_norm = p_dir.norm();
        p_dir.normalize();
        if(dir.dot(p_dir) >= cos(OVERLAP_ANGLE) && p_dir_norm > max_r)
        {
            max_r = p_dir_norm;
        }
    }

    return max_r;
}

vector<SegmentedPointsContainer::SegmentedPlane> PlaneMerging::getSegmentedPlanes()
{
    return this->plane_list;
}

bool PlaneMerging::isCloudMerged()
{
    return isMerged;
}

void PlaneMerging::applyTransform(mat4 M)
{
    if(!isMerged) return;

    #pragma omp parallel for
    for(size_t i = 0; i < plane_list.size(); ++i)
    {
        vec3 n = plane_list[i].plane.getNormal();
        vec4 n4(n.x(), n.y(), n.z(), 1);
        vec4 Mn = M * n4;
        plane_list[i].plane.setNormal(vec3(Mn.x(), Mn.y(), Mn.z()));

        //TODO: Apply M to centroids
    }
}

void PlaneMerging::printVectorsInFile(string filename)
{
    // Open file
    ofstream file;
    file.open(filename);

    // Write each normal vector in format [ x, y, z ]
    for(auto p: this->plane_list)
    {
        vec3 n = p.plane.getNormal();
        file << "[ " << n.x() << ", " << n.y() << ", " << n.z() << " ]" << endl;
    }

    file.close();
}
