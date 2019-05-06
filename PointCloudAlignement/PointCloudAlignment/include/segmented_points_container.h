#pragma once

#include "common.h"
#include "plane.h"

class SegmentedPointsContainer
{
public:
    typedef struct _SegmentedPlane
    {
        int id;
        ivec3 color;
        vector<int> indices_list;
        Plane plane;

        _SegmentedPlane(): id(0), color(0, 0, 0) {}
        _SegmentedPlane(int id, ivec3 c, vector<int> list, Plane p): id(id), color(c), indices_list(list), plane(p) {}
        ~_SegmentedPlane(){}

        void merge(_SegmentedPlane &p);
    } SegmentedPlane;

    SegmentedPointsContainer(): segmented_points(0), excluded_points(0)
    {
        ivec3 c[] = {ivec3(229,115,115), ivec3(255,143,64), ivec3(153,86,38),
                    ivec3(102,72,51), ivec3(127,106,0), ivec3(255,223,64),
                    ivec3(204,187,102), ivec3(115,153,0), ivec3(207,255,64),
                    ivec3(67,77,38), ivec3(34,102,0), ivec3(51,26,30),
                    ivec3(0,255,21), ivec3(115,230,124), ivec3(0,51,26),
                    ivec3(89,179,134), ivec3(0,230,210), ivec3(0,77,70),
                    ivec3(89,179,171), ivec3(26,51,49), ivec3(0,136,204),
                    ivec3(0,51,77), ivec3(128,213,255), ivec3(64,106,128),
                    ivec3(0,32,128), ivec3(64,112,255), ivec3(26,45,102),
                    ivec3(128,159,255), ivec3(64,80,128), ivec3(86,57,230),
                    ivec3(104,89,179), ivec3(89,0,153), ivec3(30,0,51),
                    ivec3(202,128,255), ivec3(81,51,102), ivec3(204,0,204),
                    ivec3(179,0,179), ivec3(153,0,153), ivec3(255,0,149),
                    ivec3(102,0,60), ivec3(179,45,123), ivec3(255,128,202),
                    ivec3(77,38,61), ivec3(127,64,74)};
        vector<ivec3> list(c, c + sizeof(c) / sizeof(ivec3));
        color_list = list;
        curr_color = color_list.begin();

        misc_color = ivec3(255, 0, 0);
    }

    ~SegmentedPointsContainer()
    {
        excluded_points.clear();
        planes_list.clear();
    }

    void addSegmentedPoints(SegmentedPlane &plane);
    void addExcludedPoints(vector<int> point_list);
    void addExcludedPoint(int p_id);
    void createPlane(int plane_id, ivec3 &c);
    void addSegmentedPoint(int plane_id, int point_id);
    int getNbOfSegmentedPoints();
    size_t getNbOfExcludedPoints();
    size_t getNbPlanes();
    ivec3 getNextPlaneColor();
    ivec3 getMiscColor() { return misc_color; }
    vector<SegmentedPlane> getPlanes() { return this->planes_list; }
    void printVectorsInFile(string filename);

    typedef boost::shared_ptr<SegmentedPointsContainer> Ptr;

private:
    int segmented_points;
    vector<ivec3> color_list;
    ivec3 misc_color;
    vector<ivec3>::iterator curr_color;

    vector<SegmentedPlane> planes_list;
    vector<int> excluded_points;
};
