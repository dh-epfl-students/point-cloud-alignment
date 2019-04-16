#include "plane_merging.h"

void PlaneMerging::start_merge(vector<SegmentedPointsContainer::SegmentedPlane> &p_list, PointNormalKCloud::Ptr p_cloud)
{
    // We assume that the plane list must be sorted by plane id.
    this->plane_list = p_list;

    p_point_cloud = p_cloud;

    //TODO: Compute bounding points of planes
    /*for(SegmentedPointsContainer::SegmentedPlane p: plane_list)
    {
        compute_boundaries(p);
    }*/

    //fill center clouds
    p_plane_cloud->points.reserve(p_list.size());
    for(auto p: p_list)
    {
        pcl::PointXYZ pXYZ(p.plane.getCenter().x(), p.plane.getCenter().y(), p.plane.getCenter().z());
        p_plane_cloud->points.push_back(pXYZ);
    }

    //fill search tree with centers
    p_kdtree->setInputCloud(p_plane_cloud);

    //TODO: merge
    merge();
}

void PlaneMerging::filter_small_planes(vector<SegmentedPointsContainer::SegmentedPlane> &p_list, int min_size)
{
    remove_if(p_list.begin(), p_list.end(), [min_size](SegmentedPointsContainer::SegmentedPlane p){
        return p.indices_list.size() < static_cast<size_t>(min_size);
    });
}

void PlaneMerging::merge()
{
    for(auto plane: plane_list)
    {
        // Search nearest neighbors plane centers
        pcl::PointXYZ pXYZ(plane.plane.getCenter().x(), plane.plane.getCenter().y(), plane.plane.getCenter().z());
        vector<int> indices;
        vector<float> sqrd_dists;
        p_kdtree->nearestKSearch(pXYZ, KNN, indices, sqrd_dists);

        // Test all planes
        for(int i: indices)
        {
            //May need to reorient normal
            vec3 n = plane.plane.getNormal();
            vec3 ni = plane_list[i].plane.getNormal();
            n = ni.dot(n) >= ni.dot(-n) ? n : -n;

            // Filter by plane normal vector
            if(acos(ni.dot(n)) <= NORMAL_ERROR)
            {
                // Filter by plane overlap
                if(planeOverlap(plane, plane_list[i]))
                {
                    // Merge planes

                    // Update plane list

                }
            }
        }
    }
}

bool PlaneMerging::planeOverlap(SegmentedPointsContainer::SegmentedPlane &p1, SegmentedPointsContainer::SegmentedPlane &p2, float d_tolerance)
{
    vec3 dir1 = p2.plane.getCenter3() - p1.plane.getCenter3();
    vec3 dir2 = p1.plane.getCenter3() - p2.plane.getCenter3();

    // Select points to consider between both center points.
    float r1 = farestPointInDir(p1, dir1);
    float r2 = farestPointInDir(p2, dir2);

    // Compare distances to find overlap
    float d = dir1.norm();

    return d <= r1 + r2 + d_tolerance;
}

float PlaneMerging::farestPointInDir(SegmentedPointsContainer::SegmentedPlane &plane, vec3 dir)
{
    float max_r(0);
    dir.normalize();

    for(int i: plane.indices_list)
    {
        vec3 pi(p_point_cloud->points[i].x, p_point_cloud->points[i].y, p_point_cloud->points[i].z);
        vec3 p_dir = pi - plane.plane.getCenter3();
        float p_dir_norm = p_dir.norm();
        p_dir.normalize();
        if(dir.dot(p_dir) >= OVERLAP_ANGLE && p_dir_norm > max_r)
        {
            max_r = p_dir_norm;
        }
    }

    return max_r;
}
