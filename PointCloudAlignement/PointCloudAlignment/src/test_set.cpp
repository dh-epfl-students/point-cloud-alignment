#include "test_set.h"

// =========================== // AlignObjectInterface // ============================================== //

AlignObjectInterface::~AlignObjectInterface() {}

string AlignObjectInterface::getFilename()
{
    return this->filename;
}

bool AlignObjectInterface::isSource()
{
    return this->isSrc;
}


// =========================== // CloudObject // ============================================== //

void CloudObject::displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport, string id_prefix)
{
    stringstream ss;
    ss << id_prefix << (isSource()? "source_cloud_vp" : "target_cloud_vp") << viewport;
    string object_id = ss.str();

    pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> colorHandler(p_object, color.x(), color.y(), color.z());
    p_viewer->addPointCloud(p_object, colorHandler, object_id, viewport);
    p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, object_id);
}

void CloudObject::segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes)
{
    vector<SegmentedPointsContainer::SegmentedPlane> segmented_planes;

    PlaneSegmentation segmentation;
    segmentation.init(this->getFilename(), this->isSource());

    if(segmentation.isReady())
    {
        segmentation.filterOutCurvature(MAX_CURVATURE);
        segmentation.start_pause();
        segmentation.runMainLoop();
        segmented_planes = segmentation.getSegmentedPlanes();
    }
    else
    {
        cout << "Error: " << this->getFilename() << " is not preprocessed. Exiting..." << endl;
        exit(EXIT_FAILURE);
    }

    PlaneMerging merger;
    merger.init(nullptr, false);
    merger.start_merge(segmented_planes, segmentation.getPointCloud());
    out_planes = merger.getSegmentedPlanes();

    // We also keep the pointer to the cloud for later useage
    this->p_object = segmentation.getPointCloud();
}

void CloudObject::transform(mat4 &M)
{
    // Transform source cloud
    PointNormalKCloud::Ptr aligned_cloud(new PointNormalKCloud);
    pcl::transformPointCloudWithNormals(*this->p_object, *aligned_cloud, M);
    this->p_object = aligned_cloud;
}


PointNormalKCloud::Ptr CloudObject::getObject()
{
    return this->p_object;
}

void CloudObject::setObject(PointNormalKCloud::Ptr object_ptr)
{
    this->p_object = object_ptr;
}

bool CloudObject::isCloud()
{
    return this->isCld;
}

// =========================== // MeshObject // ============================================== //

void MeshObject::displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport, string id_prefix)
{
    stringstream ss;
    ss << id_prefix << (isSource()? "source_mesh_vp" : "target_mesh_vp") << viewport;
    string object_id = ss.str();

    p_viewer->addPolygonMesh(*p_object, object_id, viewport);
}

void MeshObject::segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes)
{

}

void MeshObject::transform(mat4 &M)
{

}

pcl::PolygonMesh::Ptr MeshObject::getObject()
{
    return this->p_object;
}

void MeshObject::setObject(pcl::PolygonMesh::Ptr object_ptr)
{
    this->p_object = object_ptr;
}

bool MeshObject::isCloud()
{
    return this->isCld;
}

// =========================== // TestingSet // ============================================== //

TestingSet::TestingSet(string targetfile, bool isCloud)
{
    if(isCloud)
    {
        p_target = unique_ptr<AlignObjectInterface>(new CloudObject(targetfile, false));
    }
    else
    {
        p_target = unique_ptr<AlignObjectInterface>(new MeshObject(targetfile, false));
    }
}

void TestingSet::addSource(string sourcefile, bool isCloud)
{
    if(isCloud)
    {
        sources.emplace_back(new CloudObject(sourcefile, true));
    }
    else
    {
        sources.emplace_back(new MeshObject(sourcefile, true));
    }
}

bool TestingSet::isInitialized()
{
    return !sources.empty();
}

void TestingSet::runTests()
{
    // First the objects are segmented

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            p_target->segment(this->target_planes);
        }

        #pragma omp section
        {
            this->results.resize(sources.size());

            #pragma omp parallel for
            for(size_t i = 0; i < this->sources.size(); ++i)
            {
                this->sources[i]->segment(this->results[i].source_planes);
            }
        }
    }

    cout << "Plane segmentation finished, starting alignment..." << endl;

    // Then the registration
    this->sources_aligned.resize(this->sources.size());

    #pragma omp parallel for
    for(size_t i = 0; i < this->sources.size(); ++i)
    {
        this->runAlignment(i);
    }
}

void TestingSet::runAlignment(size_t source_id)
{
    auto source_planes = this->results[source_id].source_planes;

    // Registration step
    Registration registration;

    // Need to get the point cloud pointer in case of cloud object
    PointNormalKCloud::Ptr t_cloud = nullptr;
    PointNormalKCloud::Ptr s_cloud = nullptr;

    if(this->p_target->isCloud())
    {
        t_cloud = (dynamic_cast<CloudObject*>(p_target.get()))->getObject();
    }

    if(this->sources[source_id]->isCloud())
    {
        s_cloud = (dynamic_cast<CloudObject*>(this->sources[source_id].get()))->getObject();
    }

    registration.setClouds(source_planes, this->target_planes, !this->p_target->isCloud(), !this->sources[source_id]->isCloud(), s_cloud, t_cloud);
    mat4 M = registration.findAlignment();
    mat4 finalM = M;
    bool realign = registration.applyTransform(M);

    if(realign)
    {
        finalM = M * finalM;
    }

    this->results[source_id].transform = finalM;

    // Save aligned object in the sources_aligned vector
    // First deep copy of current source
    shared_ptr<AlignObjectInterface> source_aligned;

    if(this->sources[source_id]->isCloud())
    {
        source_aligned = shared_ptr<CloudObject>(new CloudObject(*dynamic_cast<CloudObject*>(this->sources[source_id].get())));
    }
    else
    {
        source_aligned = shared_ptr<MeshObject>(new MeshObject(*dynamic_cast<MeshObject*>(this->sources[source_id].get())));
    }

    // transform the object
    source_aligned->transform(finalM);

    // store the transformed object
    this->sources_aligned[source_id] = source_aligned;

    //TODO: save alignment statistics in results[i]
}

void TestingSet::display(pcl::visualization::PCLVisualizer::Ptr p_viewer, vector<int> &viewports)
{
    // Display target in all viewports
    this->p_target->displayObjectIn(p_viewer, TARGET_COLOR);

    // Display one source and its aligned equivalent by viewport
    for(size_t i = 0; i < min(viewports.size(), this->sources.size()); ++i)
    {
        this->sources[i]->displayObjectIn(p_viewer, SOURCE_COLOR, viewports[i]);

        this->sources_aligned[i]->displayObjectIn(p_viewer, ALIGNED_COLOR, viewports[i], "aligned_");
    }
}
