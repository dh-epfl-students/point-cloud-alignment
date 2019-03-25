#pragma once

#include "common.h"

class SegmentedPointsContainer
{
public:
    SegmentedPointsContainer(): segmented_points(0), excluded_points(new PointNormalKCloud) {}
    ~SegmentedPointsContainer()
    {
        excluded_points.reset();
        planes_list.clear();
    }

    void addPlane(PointNormalKCloud::Ptr cloud);
    void addExcludedPoints(PointNormalKCloud::Ptr cloud);
    int getNbOfSegmentedPoints();
    size_t getNbOfExcludedPoints();
    ivec3 getNextPlaneColor();

private:
    int segmented_points;

    vector<PointNormalKCloud::Ptr> planes_list;
    PointNormalKCloud::Ptr excluded_points;
};
