#pragma once

#include <sstream>

#include <omp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

#include "common.h"
#include "plane_segmentation.h"
#include "plane_merging.h"
#include "segmented_points_container.h"
#include "mesh_segmentation.h"
#include "registration.h"

using namespace std;

class AlignObjectInterface
{
public:
    AlignObjectInterface(string file, bool isSource): filename(file), isSrc(isSource) {}
    virtual ~AlignObjectInterface();

    bool isSource();

    string getFilename();
    void setFilename(string file);

    virtual void displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport = 0, string id_prefix = "") = 0;

    /**
     * @brief Perform plane segmentation and plane merging
     * @param out_planes Output the resulting segmented planes
     */
    virtual void segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes) = 0;

    virtual void preprocess() = 0;

    virtual void transform(mat4 &M) = 0;

    virtual void saveObject(string suffix) = 0;

    virtual bool isCloud() = 0;

private:
    string filename;
    bool isSrc;

protected:
    bool isCld;
};

class CloudObject : public AlignObjectInterface
{
public:
    CloudObject(string file, bool isSource): AlignObjectInterface(file, isSource)
    {
        this->isCld = true;
        this->p_object = nullptr;
    }

    CloudObject(CloudObject &object): AlignObjectInterface (object.getFilename(), object.isSource())
    {
        this->isCld = true;
        this->p_object = PointNormalKCloud::Ptr(new PointNormalKCloud);
        pcl::copyPointCloud(*object.getObject(), *this->p_object);
    }

    void displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport = 0, string id_prefix = "");

    void preprocess();

    void segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes);

    void transform(mat4 &M);

    void saveObject(string suffix);

    PointNormalKCloud::Ptr getObject();
    void setObject(PointNormalKCloud::Ptr object_ptr);

    bool isCloud();

private:
    PointNormalKCloud::Ptr p_object;
};

class MeshObject : public AlignObjectInterface
{
public:
    MeshObject(string file, bool isSource): AlignObjectInterface(file, isSource)
    {
        this->isCld = false;
        this->p_object = nullptr;
    }

    MeshObject(MeshObject &object): AlignObjectInterface(object.getFilename(), object.isSource())
    {
        this->isCld = false;
        this->p_object = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh(*object.getObject()));
    }

    void displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport = 0, string id_prefix = "");

    void preprocess();

    void segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes);

    void transform(mat4 &M);

    void saveObject(string suffix);

    pcl::PolygonMesh::Ptr getObject();
    void setObject(pcl::PolygonMesh::Ptr object_ptr);

    bool isCloud();

private:
    pcl::PolygonMesh::Ptr p_object;
};

struct AlignmentResults
{
    mat4 transform;
    vector<SegmentedPointsContainer::SegmentedPlane> source_planes;
};

/**
 * @brief The TestingSet struct contains a target and sources clouds of the same area
 * that will be aligned.
 */
class TestingSet
{
public:
    TestingSet() {}
    TestingSet(string targetfile, bool isCloud = true);

    void addSource(string sourcefile, bool isCloud = true);
    bool isInitialized();

    void runTests();

    void display(pcl::visualization::PCLVisualizer::Ptr p_viewer, vector<int> &viewports);

    void applyRandomTransforms();
    void preprocessClouds();
    void saveObjectsPLY();
    void writeTestSet(ofstream &output);

private:
    shared_ptr<AlignObjectInterface> p_target;
    vector<shared_ptr<AlignObjectInterface>> sources;
    vector<shared_ptr<AlignObjectInterface>> sources_aligned;

    vector<SegmentedPointsContainer::SegmentedPlane> target_planes;
    vector<AlignmentResults, Eigen::aligned_allocator<AlignmentResults> > results;

    void runAlignment(size_t source_id);
};
