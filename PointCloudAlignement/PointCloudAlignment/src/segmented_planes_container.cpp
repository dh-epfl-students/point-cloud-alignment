#include "segmented_points_container.h"

void SegmentedPointsContainer::addExcludedPoints(PointNormalKCloud::Ptr cloud)
{
    *excluded_points += *cloud;
}

void SegmentedPointsContainer::addPlane(PointNormalKCloud::Ptr cloud)
{
    this->planes_list.push_back(cloud);
    this->segmented_points += cloud->points.size();
}

size_t SegmentedPointsContainer::getNbOfExcludedPoints()
{
    return excluded_points->size();
}

int SegmentedPointsContainer::getNbOfSegmentedPoints()
{
    return segmented_points;
}

ivec3 SegmentedPointsContainer::getNextPlaneColor()
{
    return ivec3(15, 255, 15);
}
