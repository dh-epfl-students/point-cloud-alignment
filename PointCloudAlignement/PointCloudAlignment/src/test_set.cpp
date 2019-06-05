#include "test_set.h"

// =========================== // AlignObjectInterface // ============================================== //

AlignObjectInterface::~AlignObjectInterface() {}

void AlignObjectInterface::setFilename(string file)
{
    this->filename = file;
}

string AlignObjectInterface::getFilename()
{
    return this->filename;
}

mat4 AlignObjectInterface::getOriginalTransform()
{
    return this->originalT;
}

void AlignObjectInterface::setOriginalTransform(mat4 &t)
{
    this->originalT = t;
}

bool AlignObjectInterface::isSource()
{
    return this->isSrc;
}


// =========================== // CloudObject // ============================================== //

void CloudObject::loadObject()
{
    p_object = PointNormalKCloud::Ptr(new PointNormalKCloud);
    int r = pcl::io::loadPCDFile(this->getFilename(), *p_object);

    if(r == -1 && pcl::io::loadPLYFile(this->getFilename(), *p_object) == -1)
    {
        PCL_ERROR("Could not read given file\n");
        exit(EXIT_FAILURE);
    }
}

void CloudObject::displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport, string id_prefix)
{
    stringstream ss;
    ss << id_prefix << (isSource()? "source_cloud_vp" : "target_cloud_vp") << viewport;
    string object_id = ss.str();

    pcl::visualization::PointCloudColorHandlerCustom<PointNormalK> colorHandler(p_object, color.x(), color.y(), color.z());
    p_viewer->addPointCloud(p_object, colorHandler, object_id, viewport);
    p_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, object_id);
}

void CloudObject::preprocess()
{
    PlaneSegmentation seg;

    if(this->p_object == nullptr)
    {
        seg.init(this->getFilename(), this->isSource());
    }
    else {
        seg.init(this->p_object, this->isSource());
    }

    seg.resampleCloud();
    seg.preprocessCloud();

    this->p_object = seg.getPointCloud();
}

void CloudObject::segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes)
{
    vector<SegmentedPointsContainer::SegmentedPlane> segmented_planes;

    PlaneSegmentation segmentation;

    if(this->p_object == nullptr)
    {
        segmentation.init(this->getFilename(), this->isSource());
    }
    else
    {
        segmentation.init(this->p_object, this->isSource());
    }

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

    // We also keep the pointer to the cloud for later use
    this->p_object = segmentation.getPointCloud();
}

void CloudObject::transform(mat4 &M)
{
    if(this->p_object == nullptr) this->loadObject();

    // Transform source cloud
    PointNormalKCloud::Ptr aligned_cloud(new PointNormalKCloud);
    pcl::transformPointCloudWithNormals(*this->p_object, *aligned_cloud, M);
    this->p_object = aligned_cloud;
}

void CloudObject::saveObject(string suffix, int set_id)
{
    if(this->p_object == nullptr) this->loadObject();

    boost::filesystem::path p(this->getFilename());
    string name = p.stem().string();

    stringstream ss;
    ss << set_id << "_" << name << suffix << ".ply";
    auto new_p = p.remove_filename();
    new_p.append(ss.str());
    this->setFilename(new_p.string());

    pcl::io::savePLYFile(this->getFilename(), *this->p_object, true);
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

void MeshObject::loadObject()
{
    this->p_object = pcl::PolygonMeshPtr(new pcl::PolygonMesh);
    if(pcl::io::loadPolygonFilePLY(this->getFilename(), *p_object) == -1)
    {
        cout << "Failed to load given mesh file" << endl;
        exit(EXIT_FAILURE);
    }
}

void MeshObject::displayObjectIn(pcl::visualization::PCLVisualizer::Ptr p_viewer, ivec3 color, int viewport, string id_prefix)
{
    // Color mesh vertices in color
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2(this->p_object->cloud, cloud);

    // Color whole point in blue
    for(size_t i = 0; i < cloud.size(); ++i)
    {
        cloud.points[i].rgba = static_cast<uint8_t>(color.x()) << 16 |
                 static_cast<uint8_t>(color.y()) << 8 |
                 static_cast<uint8_t>(color.z());
    }

    pcl::toPCLPointCloud2(cloud, this->p_object->cloud);

    stringstream ss;
    ss << id_prefix << (isSource()? "source_mesh_vp" : "target_mesh_vp") << viewport;
    string object_id = ss.str();

    p_viewer->addPolygonMesh(*this->p_object, object_id, viewport);
}

void MeshObject::preprocess()
{
    // DO NOTHING BECAUSE IT IS NOT NECESSARY
    // TO PREPROCESS A MESH BEFORE PLANE SEGMENTATION
}

void MeshObject::segment(vector<SegmentedPointsContainer::SegmentedPlane> &out_planes)
{
    MeshSegmentation seg;
    seg.loadMesh(this->p_object);
    seg.segmentPlanes();
    seg.mergePlanes();
    out_planes = seg.getSegmentedPlanes();
    this->p_object = seg.getMeshPtr();
}

void MeshObject::transform(mat4 &M)
{
    if(this->p_object == nullptr) this->loadObject();

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2(this->p_object->cloud, cloud);
    pcl::transformPointCloud(cloud, cloud, M);
    pcl::toPCLPointCloud2(cloud, this->p_object->cloud);
}

void MeshObject::saveObject(string suffix, int set_id)
{
    if(this->p_object == nullptr) this->loadObject();

    boost::filesystem::path p(this->getFilename());
    string name = p.stem().string();

    stringstream ss;
    ss << set_id << "_" << name << suffix << ".ply";
    auto new_p = p.remove_filename();
    new_p.append(ss.str());
    this->setFilename(new_p.string());

    pcl::io::savePolygonFilePLY(this->getFilename(), *this->p_object);
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

void TestingSet::addSource(string sourcefile, bool isCloud, mat4 m)
{
    if(isCloud)
    {
        sources.emplace_back(new CloudObject(sourcefile, true, m));
    }
    else
    {
        sources.emplace_back(new MeshObject(sourcefile, true, m));
    }
}

bool TestingSet::isInitialized()
{
    return !sources.empty();
}

void TestingSet::loadSet()
{
    this->p_target->loadObject();

    for(size_t i = 0; i < this->sources.size(); ++i)
    {
        this->sources[i]->loadObject();
    }
}

void TestingSet::runTests()
{
    // Load the objects before going in parallel sections. Workaround for crash when loading polygonfile in parallel
    this->loadSet();

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

    // Then they are registered
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

    registration.applyTransform(M);

    mat4 realign_M = registration.refineAlignment();

    registration.applyTransform(realign_M);

    mat4 ICP_M = registration.finalICP();

    registration.applyTransform(ICP_M);

    // Saving the final transformation because we need it to compute alignment errors when writing results
    this->results[source_id].transform = ICP_M * realign_M * M;

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
    source_aligned->transform(this->results[source_id].transform);

    // store the transformed object
    this->sources_aligned[source_id] = source_aligned;

    //TODO: save alignment statistics in results[i]
    // TEST: compare distance between selected pairs, and nearest centers to see if they correspond
    auto pair_list = registration.getSelectedPlanes();
    auto distances = registration.computeDistanceErrors();

    // Create a cloud with target planes centers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto i: this->target_planes)
    {
        cloud->push_back(i.plane.getCenterPCL());
    }
    pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>);
    kdtree->setInputCloud(cloud);

    // Now for each point in source list, get nearest center and compute distance
    for(size_t i = 0; i < pair_list.size(); ++i)
    {
        auto s_plane = get<0>(pair_list[i]);
        auto t_plane = get<1>(pair_list[i]);

        vector<int> indices;
        vector<float> dists;
        kdtree->nearestKSearch(s_plane.plane.getCenterPCL(), 1, indices, dists);

        cout << "Before ICP: source id " << s_plane.id << " with target id " << t_plane.id << ": " << distances[i]
                << ". After ICP: found target id " << this->target_planes[indices[0]].id << ": " << sqrt(dists[0]) << endl;
    }
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

void TestingSet::applyRandomTransforms()
{
    // Apply a different random rotation and translation to every source
    for(size_t i = 0; i < this->sources.size(); ++i)
    {
        mat4 R = mat4::Identity();
        R.block(0,0,3,3) << getRandomRotation();
        mat4 T = getRandomTranslation();
        mat4 M = T * R;

        this->sources[i]->setOriginalTransform(M);
        this->sources[i]->transform(M);
    }
}

void TestingSet::preprocessClouds()
{
    this->p_target->preprocess();

    for(size_t i = 0; i < this->sources.size(); ++i)
    {
        this->sources[i]->preprocess();
    }
}

void TestingSet::saveObjectsPLY(int set_id)
{
    this->p_target->saveObject("_target", set_id);

    for (size_t i = 0; i < this->sources.size(); ++i)
    {
        stringstream ss;
        ss << "_source_" << i;
        this->sources[i]->saveObject(ss.str(), set_id);
    }
}

void TestingSet::writeTestSet(ofstream &output)
{
    output << "group" << endl;
    output << this->p_target->getFilename() << " " << (this->p_target->isCloud() ? "c" : "m") << endl;

    for(size_t i = 0; i < this->sources.size(); ++i)
    {
        mat4 m = this->sources[i]->getOriginalTransform();
        output << this->sources[i]->getFilename() << " " << (this->sources[i]->isCloud() ? "c " : "m ") << this->getMatStr(m) << endl;
    }
}

string TestingSet::getMatStr(mat4 &m)
{
    stringstream ss;
    ss << m.row(0) << " " << m.row(1) << " " << m.row(2) << " " << m.row(3);
    return ss.str();
}

void TestingSet::writeResults()
{
    // Name of the file in which to write the stats

    // Rotation error

    // Sum of distances between each points
}
