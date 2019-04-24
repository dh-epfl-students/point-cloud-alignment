#include "segmented_points_container.h"

void SegmentedPointsContainer::addExcludedPoints(vector<int> point_list)
{
    excluded_points.insert(excluded_points.end(), point_list.begin(), point_list.end());
}

void SegmentedPointsContainer::addExcludedPoint(int p_id)
{
    excluded_points.push_back(p_id);
}

void SegmentedPointsContainer::addSegmentedPoints(SegmentedPointsContainer::SegmentedPlane plane)
{
    this->planes_list.push_back(plane);
    this->segmented_points += plane.indices_list.size();
}

size_t SegmentedPointsContainer::getNbOfExcludedPoints()
{
    return excluded_points.size();
}

int SegmentedPointsContainer::getNbOfSegmentedPoints()
{
    return segmented_points;
}

size_t SegmentedPointsContainer::getNbPlanes()
{
    return planes_list.size();
}

ivec3 SegmentedPointsContainer::getNextPlaneColor()
{
    ivec3 c = *curr_color;

    curr_color++;
    curr_color = curr_color == color_list.end()? color_list.begin() : curr_color;

    return c;
}

void SegmentedPointsContainer::createPlane(int plane_id, ivec3 &c)
{
    if(plane_id > planes_list.size())
        planes_list.resize(plane_id);

    planes_list[plane_id].id = plane_id;
    planes_list[plane_id].color = c;
}

void SegmentedPointsContainer::addSegmentedPoint(int plane_id, int point_id)
{
    if(plane_id > planes_list.size()) return;

    this->planes_list[plane_id-1].indices_list.push_back(point_id);
}
