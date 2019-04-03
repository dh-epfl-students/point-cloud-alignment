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
