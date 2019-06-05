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
    AlignObjectInterface(string file, bool isSource, mat4 m): filename(file), isSrc(isSource), originalT(m) {}
    virtual ~AlignObjectInterface();

    bool isSource();

    string getFilename();
    void setFilename(string file);

    mat4 getOriginalTransform();
    void setOriginalTransform(mat4 &t);

    virtual void loadObject() = 0;

    virtual void displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport = 0, string id_prefix = "") = 0;

    /**
     * @brief Perform plane segmentation and plane merging
     * @param out_planes Output the resulting segmented planes
     */
    virtual void segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes) = 0;

    virtual void preprocess() = 0;

    virtual void transform(mat4 &M) = 0;

    virtual void saveObject(string suffix, int set_id) = 0;

    virtual bool isCloud() = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    string filename;
    bool isSrc;

protected:
    bool isCld;
    mat4 originalT;
};

class CloudObject : public AlignObjectInterface
{
public:
    CloudObject(string file, bool isSource): AlignObjectInterface(file, isSource)
    {
        this->isCld = true;
        this->p_object = nullptr;
    }

    CloudObject(string file, bool isSource, mat4 m): AlignObjectInterface(file, isSource, m) {
        this->isCld = true;
        this->p_object = nullptr;
    }

    CloudObject(CloudObject &object): AlignObjectInterface (object.getFilename(), object.isSource(), object.getOriginalTransform())
    {
        this->isCld = true;
        this->p_object = PointNormalKCloud::Ptr(new PointNormalKCloud);
        pcl::copyPointCloud(*object.getObject(), *this->p_object);
    }

    void loadObject();

    void displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport = 0, string id_prefix = "");

    void preprocess();

    void segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes);

    void transform(mat4 &M);

    void saveObject(string suffix, int set_id);

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

    MeshObject(string file, bool isSource, mat4 m): AlignObjectInterface(file, isSource, m) {
        this->isCld = false;
        this->p_object = nullptr;
    }

    MeshObject(MeshObject &object): AlignObjectInterface(object.getFilename(), object.isSource(), object.getOriginalTransform())
    {
        this->isCld = false;
        this->p_object = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh(*object.getObject()));
    }

    void loadObject();

    void displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport = 0, string id_prefix = "");

    void preprocess();

    void segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes);

    void transform(mat4 &M);

    void saveObject(string suffix, int set_id);

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

    void addSource(string sourcefile, bool isCloud = true,  mat4 m = mat4::Identity());
    bool isInitialized();

    void loadSet();

    void runTests();

    void display(pcl::visualization::PCLVisualizer::Ptr p_viewer, vector<int> &viewports);

    void applyRandomTransforms();
    void preprocessClouds();
    void saveObjectsPLY(int set_id);
    void writeTestSet(ofstream &output);

    void writeResults();

private:
    shared_ptr<AlignObjectInterface> p_target;
    vector<shared_ptr<AlignObjectInterface>> sources;
    vector<shared_ptr<AlignObjectInterface>> sources_aligned;

    vector<SegmentedPointsContainer::SegmentedPlane> target_planes;
    vector<AlignmentResults, Eigen::aligned_allocator<AlignmentResults> > results;

    void runAlignment(size_t source_id);

    string getMatStr(mat4 &m);
};
