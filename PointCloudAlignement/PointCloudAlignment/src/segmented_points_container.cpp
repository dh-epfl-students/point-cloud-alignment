#include "segmented_points_container.h"

void SegmentedPointsContainer::SegmentedPlane::merge(SegmentedPlane &p)
{
    //cout << "Merging plane " << p.id << " into plane " << id << endl;

    this->indices_list.insert(indices_list.end(), p.indices_list.begin(), p.indices_list.end());
    sort(this->indices_list.begin(), this->indices_list.end());
    vector<int>::iterator it = unique(indices_list.begin(), indices_list.end());
    indices_list.resize(distance(indices_list.begin(), it));

    // Average the centroids
    this->plane.setCenter((this->plane.getCenter() * this->indices_list.size() + p.plane.getCenter() * p.indices_list.size()) / (this->indices_list.size() + p.indices_list.size()));

    // Ensure that both normals are orientated in the same quadrant
    vec3 n1 = this->plane.getNormal();
    vec3 n2 = p.plane.getNormal();
    n2 = n2.normalized().dot(n1.normalized()) < 0.0f ? -n2 : n2;

    // Set the new plane normal
    this->plane.setNormal(n1 + n2);
}

void SegmentedPointsContainer::addExcludedPoints(vector<int> point_list)
{
    excluded_points.insert(excluded_points.end(), point_list.begin(), point_list.end());
}

void SegmentedPointsContainer::addExcludedPoint(int p_id)
{
    excluded_points.push_back(p_id);
}

void SegmentedPointsContainer::addSegmentedPoints(SegmentedPlane &plane)
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

void SegmentedPointsContainer::printVectorsInFile(string filename)
{
    // Open file
    ofstream file;
    file.open(filename);

    // Write each normal vector in format [ x, y, z ]
    for(auto p: this->planes_list)
    {
        vec3 n = p.plane.getNormal();
        file << "[ " << n << " ]" << endl;
    }

    file.close();
}
