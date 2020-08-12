 /**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   07.08.2017
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
* @Project: ImmersiveTripReports 2017-2018
* AdobePatentID="P7840-US"
*/
/*
# Copyright 2020 CPhoto@FIT, Brno University of Technology,
# Faculty of Information Technology,
# Božetěchova 2, 612 00, Brno, Czech Republic
#
# Redistribution and use in source code form, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# 3. Redistributions must be pursued only for non-commercial research
#    collaboration and demonstration purposes.
#
# 4. Where separate files retain their original licence terms
#    (e.g. MPL 2.0, Apache licence), these licence terms are announced, prevail
#    these terms and must be complied.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.
*/


#include "sfmloader.h"

using namespace itr;
using namespace openMVG::geometry;
using namespace openMVG;
using namespace nanoflann;
using std::size_t;

SfMLoader::SfMLoader(string component_dir, ref_ptr<MapNode> mapNode,
                     double center_lat, double center_lon):
    data_dir(component_dir),
    mapNode(mapNode),
    center_lat(center_lat), center_lon(center_lon),
    view_state(SfMLoader::ViewState::SHOW_NONE),
    loaded2D3D(false)
{
    aligned_T = Eigen::Matrix4d::Identity();

    try
    {
        string sfm_data_path = findSfMDataICP(component_dir);
        if (!openMVG::sfm::Load(data, sfm_data_path,
                                (ESfM_Data)(VIEWS | EXTRINSICS |
                                            INTRINSICS | STRUCTURE)))
        {
            throw std::runtime_error("Unable to load sfm_data.");
        }
        std::cout << "ICP SfmData Loaded: " << sfm_data_path << std::endl;
    }
    catch (std::runtime_error &re)
    {
        try
        {
            std::cout << "trying to convert nvm data..." << std::endl;
            //we did not find sfm data, try nvm
            string nvm_path = findNVMData(component_dir);
            string sfm_data_path = (fs::path(nvm_path).parent_path() / "sfm_data.bin").string();

            int res = 0;
            if (!fs::exists(sfm_data_path))
            {
                fs::path full_path = fs::system_complete(fs::path( "." ));
                fs::path current_dir = full_path.parent_path();
                fs::path nvm_to_openmvg_path = current_dir / "../applications/nvm_to_openmvg";
                string nvm_to_openmvg_exe = nvm_to_openmvg_path.string();
                if (!boost::filesystem::exists(nvm_to_openmvg_exe))
                {
                    //if the exe is not located in build directory, fallback to the system-wide.
                    //if we don't have system-wide, we are screwed.
                    nvm_to_openmvg_exe = "nvm_to_openmvg";
                }
#ifdef __APPLE__
    #include "TargetConditionals.h"
    #ifdef TARGET_OS_MAC
                const char* env = getenv("DYLD_LIBRARY_PATH");
                string env_str = "DYLD_LIBRARY_PATH=" + string(env) + " ";
    #endif
#else
                const char* env = getenv("LD_LIBRARY_PATH");
                string env_str = "LD_LIBRARY_PATH=" + string(env) + " ";
#endif
                std::cout << "env: " << env_str << std::endl;
                string command = env_str + nvm_to_openmvg_exe + " " + nvm_path + " " + sfm_data_path;
                std::cout << "command: " << command << std::endl;
                res = system(command.c_str());
                std::cout << "conversion result " << res << std::endl;
            }
            if (res == 0)
            {
                //load converted openmvg data
                if (!openMVG::sfm::Load(data, sfm_data_path,
                                        (ESfM_Data)(VIEWS | EXTRINSICS |
                                                    INTRINSICS | STRUCTURE)))
                {
                    throw std::runtime_error("Unable to load sfm_data.");
                }

            }
            else
            {
                std::cerr << "Could not convert nvm file to OpenMVG" << std::endl;
                throw std::runtime_error("Unable to convert nvm file to OpenMVG.");
            }
        }
        catch (std::runtime_error &re)
        {
            string sfm_data_path = findSfMData(component_dir);
            if (!openMVG::sfm::Load(data, sfm_data_path,
                                    (ESfM_Data)(VIEWS | EXTRINSICS |
                                                INTRINSICS | STRUCTURE)))
            {
                throw std::runtime_error("Unable to load sfm_data.");
            }
            std::cout << "ICP SfmData Loaded: " << sfm_data_path << std::endl;
        }
    }
}

void SfMLoader::rotationAngles(Eigen::Matrix3d m, double &yaw,
                    double &pitch, double &roll)
{
    Eigen::Matrix4d W2C = Eigen::Matrix4d::Identity();
    W2C.block(0, 0, 3, 3) = m;

    Eigen::Matrix4d R2W;
    R2W <<  0, 1, 0, 0,
            0, 0, 1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;

    Eigen::Matrix4d R2C = W2C * R2W;
    Eigen::Vector4d A = R2C.inverse() * Eigen::Vector4d(0, 0, 1, 1);
    yaw = atan2(A.x(), A.z());

    double c = cos(yaw);
    double s = sin(yaw);
    Eigen::Matrix4d rotY;
    rotY << c , 0 , s ,  0,
            0 , 1 , 0 ,  0,
           -s , 0 , c ,  0,
            0 , 0 , 0 ,  1;

    R2C = R2C * rotY;
    A = R2C.inverse() * Eigen::Vector4d(0, 0, 1, 1);
    pitch = -atan2(A.y(), A.z());

    c = cos(pitch);
    s = sin(pitch);
    Eigen::Matrix4d rotX;
    rotX << 1 , 0 , 0 ,  0,
            0 , c ,-s ,  0,
            0 , s , c ,  0,
            0 , 0 , 0 ,  1;

    A = R2C.inverse() * Eigen::Vector4d(1, 0, 0, 1);
    roll = atan2(A.y(), A.x());
}

osg::Node *SfMLoader::loadPointCloud(MapNode *mapNode)
{

    if (orig_pointcloud)
    {
        mapNode->removeChild(orig_pointcloud);
    }

    Geode *pointCloud = new Geode;
    Geometry *pointGeom = new Geometry;
    pointCloud->addDrawable(pointGeom);

    Vec3Array *vert = new Vec3Array;
    Vec4Array *colors = new osg::Vec4Array;

    Hash_Map<IndexT, Landmark>::const_iterator it;
    for (it = data.structure.begin(); it != data.structure.end(); ++it)
    {
        Eigen::Vector3d point = it->second.X;
        vert->push_back(osg::Vec3(point.x(), point.y(), point.z()));
        //std::cout << "point: " << point.x() << ", " << point.y() << ", " << point.z() << std::endl;
        //all points red for now
        colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
    }
    pointGeom->setVertexArray(vert);
    pointGeom->setColorArray(colors);
    pointGeom->setColorBinding(Geometry::BIND_PER_VERTEX);
    pointGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,
                                                   0, vert->size()));

    pointGeom->getOrCreateStateSet()->setMode(GL_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);

    osgEarth::Registry::shaderGenerator().run(pointCloud);
    const char * point_size_vertex = "\n"
        "#version 410\n"
        "void point_size(inout vec4 vertex){\n"
        "   //gl_PointSize(2.0f);\n"
        "   vertex = vertex;\n"
        "}";

    VirtualProgram* vp = VirtualProgram::getOrCreate(pointGeom->getOrCreateStateSet());
    vp->setFunction("point_size", point_size_vertex, ShaderComp::LOCATION_VERTEX_MODEL);

    mapNode->addChild(pointCloud);
    orig_pointcloud = pointCloud;

    //alignPointCloudICP(mapNode);
    return pointCloud;
}

osg::Node *SfMLoader::addPointCloud(MapNode *mapNode, vector<osg::Vec3d> points,
                                    osg::Vec4 color, float elevated)
{
    ref_ptr<SpatialReference> srs = SpatialReference::get("epsg:4326");
    Geode *pointCloud = new Geode;
    Geometry *pointGeom = new Geometry;
    pointCloud->addDrawable(pointGeom);

    Vec3Array *vert = new Vec3Array;
    Vec4Array *colors = new osg::Vec4Array;

    for (osg::Vec3d &point : points)
    {
        //elevate each point
        GeoPoint posp(srs->getGeocentricSRS(), point.x(), point.y(), point.z(), ALTMODE_ABSOLUTE);
        GeoPoint wgs84pos = posp.transform(srs);
        wgs84pos.z() = wgs84pos.z() + elevated;
        GeoPoint ecefElevated = wgs84pos.transform(srs->getGeocentricSRS());

        vert->push_back(osg::Vec3(ecefElevated.x(), ecefElevated.y(), ecefElevated.z()));
        //vert->push_back(point);
        colors->push_back(color);
    }
    pointGeom->setVertexArray(vert);
    pointGeom->setColorArray(colors);
    pointGeom->setColorBinding(Geometry::BIND_PER_VERTEX);
    pointGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,
                                                   0, vert->size()));
    pointGeom->getOrCreateStateSet()->setMode(GL_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
    pointGeom->getOrCreateStateSet()->setRenderBinDetails(INT_MAX, "RenderBin");

    osgEarth::Registry::shaderGenerator().run(pointCloud);
    const char * point_size_vertex = "\n"
        "#version 410\n"
        "void point_size(inout vec4 vertex){\n"
        "   //gl_PointSize(2.0f);\n"
        "   vertex = vertex;\n"
        "}";

    VirtualProgram* vp = VirtualProgram::getOrCreate(pointGeom->getOrCreateStateSet());
    vp->setFunction("point_size", point_size_vertex, ShaderComp::LOCATION_VERTEX_MODEL);

    osgEarth::Registry::shaderGenerator().run(pointCloud);
    mapNode->addChild(pointCloud);
    return pointCloud;
}

BBox3D SfMLoader::findPointCloudBBox(const vector<IndexT> &cluster,
                                     Vector3d &centroid)
{
    BBox3D b;

    vector<IndexT>::const_iterator it;
    centroid.x() = 0.0; centroid.y() = 0.0; centroid.z() = 0.0;
    int t = 1;
    for (it = cluster.begin(); it != cluster.end(); ++it)
    {
        Eigen::Vector3d p = data.structure.find(*it)->second.X;

        for (int i = 0; i < 3; ++i)
        {
            if (p[i] > b.max[i])
            {
                b.max[i] = p[i];
            }
            if (p[i] < b.min[i])
            {
                b.min[i] = p[i];
            }
        }
        centroid += (p - centroid) / t;
        ++t;
    }
    return b;
}

BBox3D SfMLoader::findPointCloudBBox(const vector<Eigen::Vector3d> &cluster,
                                     Vector3d &centroid)
{
    BBox3D b;

    vector<Eigen::Vector3d>::const_iterator it;
    centroid.x() = 0.0; centroid.y() = 0.0; centroid.z() = 0.0;
    int t = 1;
    for (it = cluster.begin(); it != cluster.end(); ++it)
    {
        Eigen::Vector3d p = *it;

        for (int i = 0; i < 3; ++i)
        {
            if (p[i] > b.max[i])
            {
                b.max[i] = p[i];
            }
            if (p[i] < b.min[i])
            {
                b.min[i] = p[i];
            }
        }
        centroid += (p - centroid) / t;
        ++t;
    }
    return b;
}


void SfMLoader::filterOutliers()
{
    std::cout << "Filtering outliers" << std::endl;
    Hash_Map<IndexT, Landmark>::const_iterator it;
    Hash_Map<IndexT, double> means;

    //for each point calculate minimum distance to all photos and if it is
    //larger than threshold, discard it
    /*vector<IndexT> to_remove;
    for (it = data.structure.begin(); it != data.structure.end(); ++it)
    {
        Eigen::Vector3d point = it->second.X;
        sfm::Views::const_iterator pit;
        double mindist = std::numeric_limits<double>::max();
        for (pit = data.views.begin(); pit != data.views.end(); ++pit)
        {
            const sfm::View &v = (*pit->second);
            Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v.id_pose);
            if (poses_it != data.poses.end())
            {
                //load photo view
                Eigen::Vector3d photo_position = poses_it->second.center();
                double dist = (point - photo_position).norm();
                mindist = fmin(dist, mindist);
            }
        }
        if (mindist > 2500) // if shortest distance is larger than 5km, remove
        {
            to_remove.push_back(it->first);
        }
    }
    for (IndexT idx : to_remove)
    {
        data.structure.erase(idx);
    }
    to_remove.clear();*/

    //calculate mean for each point and mean from all point means
    double mean_total = 0;
    double tt = 1;
    for (it = data.structure.begin(); it != data.structure.end(); ++it)
    {
        Eigen::Vector3d p = it->second.X;

        double mean = 0;
        int t = 1;
        Hash_Map<IndexT, Landmark>::const_iterator sit;
        for (sit = data.structure.begin(); sit != data.structure.end(); ++sit)
        {
            Eigen::Vector3d p1 = sit->second.X;
            mean += ((p - p1).norm() - mean) / t;
            ++t;
        }
        if (!std::isnan(mean))
        {
            mean_total += (mean - mean_total) / tt;
            means.insert(pair<IndexT, double>(it->first, mean));
            ++tt;
        }

    }

    //calculate stdev of from the point means
    double sum_diff = 0;
    Hash_Map<IndexT, double>::const_iterator mit;
    for (mit = means.begin(); mit != means.end(); ++mit)
    {
        double diff = (mean_total - mit->second);
        sum_diff += diff * diff;
    }
    double std_total = sqrt(sum_diff / (means.size() - 1));

    //filter points
    std::cout << "mean_total: " << mean_total << ", std total: " << std_total << "sum_diff: " << sum_diff << std::endl;
    double min = mean_total - std_total;
    double max = mean_total + std_total;
    for (mit = means.begin(); mit != means.end(); ++mit)
    {
        //std::cout << "mean: " << mit->second << "min: " << min << ", max: " << max << std::endl;
        if (mit->second < min || mit->second > max)
        {
            //reject the point, since mean of distances to other points
            //exeeds the permitted interval
            data.structure.erase(mit->first);
        }
    }


}


Eigen::MatrixXd SfMLoader::runICP(vector<osg::Vec3d> points, //sampled terrain points
                                  vector<Eigen::Vector3d> cloud, //sfm point cloud
                                  vector<osg::Vec3d> &result)
{
    typedef PointMatcher<double> PM;
    typedef PM::DataPoints DP;
    typedef PM::Parameters Parameters;

    typedef PM::DataPoints::Labels Labels;
    typedef PM::DataPoints::Label Label;
    //create point cloud to be aligned
    Labels point_lbl;
    point_lbl.push_back(Label("x", 1));
    point_lbl.push_back(Label("y", 1));
    point_lbl.push_back(Label("z", 1));
    point_lbl.push_back(Label("w", 1));
    Eigen::MatrixXd point_feat(4, points.size());
    int idx = 0;
    for (osg::Vec3d &p : points)
    {
        point_feat.col(idx) << p.x(), p.y(), p.z(), 1.0;
        ++idx;
    }
    DP points_cloud(point_feat, point_lbl);
    //std::cout << point_feat << std::endl;

    //create reference point cloud
    Labels ref_lbl;
    ref_lbl.push_back(Label("x", 1));
    ref_lbl.push_back(Label("y", 1));
    ref_lbl.push_back(Label("z", 1));
    ref_lbl.push_back(Label("w", 1));
    Eigen::MatrixXd ref_feat(4, cloud.size());
    idx = 0;
    std::vector<Eigen::Vector3d>::iterator it;
    for (it = cloud.begin(); it != cloud.end(); ++it)
    {
        Eigen::Vector3d point = *it;
        ref_feat.col(idx) << point.x(), point.y(), point.z(), 1.0;
        ++idx;
    }
    //std::cout << ref_feat << std::endl;
    DP ref_cloud(ref_feat, ref_lbl);

    PM::ICP icp;

    icp.setDefault();


    std::cout << "running ICP" << std::endl;
    PM::TransformationParameters T = icp(ref_cloud, points_cloud);
    DP data_out(ref_cloud);
    icp.transformations.apply(data_out, T);
    std::cout << "obtaining ICP result" << std::endl;
    for (int i = 0; i < data_out.features.cols(); ++i)
    {
        double w = data_out.features.col(i)[3];
        double x = data_out.features.col(i)[0] / w;
        double y = data_out.features.col(i)[1] / w;
        double z = data_out.features.col(i)[2] / w;

        result.push_back(osg::Vec3d(x, y, z));
    }

    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(T))
    {
        std::cout << "WARNING: T does not represent a valid rigid "
                     "transformation\nProjecting onto an orthogonal basis"
                  << std::endl;
        T = rigidTrans->correctParameters(T);
    }
    std::cout << "Final transformation: " << T << std::endl;

    return T;
}

vector< vector<IndexT> > SfMLoader::findEuclideanClustersPhotoviews(double search_radius)
{
    size_t point_count = photoviews.size();
    Eigen::MatrixXd cloud(point_count, 3);

    Hash_Map<IndexT, shared_ptr<AbstractPhotoView>>::const_iterator it;
    Hash_Map<size_t, IndexT> indexMap;
    size_t idx = 0;

    for (it = photoviews.begin(); it != photoviews.end(); ++it)
    {
        osg::Vec3d p = it->second->getECEFPosition();
        Eigen::Vector3d point = Eigen::Vector3d(p.x(), p.y(), p.z());
        cloud.row(idx) = point;
        indexMap.insert(pair<size_t, IndexT>(idx, it->first));
        ++idx;
    }

    typedef KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree_t;
    kd_tree_t kd_tree(cloud, 10); //10 max leaf size
    kd_tree.index->buildIndex();

    vector< vector<IndexT> > clusters;
    set<size_t> processed;

    for (size_t i = 0; i < point_count; ++i)
    {
        //std::cout << "done: " << (double)i/point_count << "%" << std::endl;
        vector<IndexT> cluster;
        queue<size_t> q;
        if (processed.find(i) == processed.end())
        {
            q.push(i);
        }
        while (!q.empty())
        {
            //std::cout << "queue size: " << q.size() << std::endl;
            size_t point_idx = q.front();
            processed.insert(point_idx);
            cluster.push_back(indexMap.find(point_idx)->second);
            q.pop();
            vector< pair<kd_tree_t::IndexType, double> > result;
            Eigen::Vector3d point = cloud.row(point_idx);
            vector<double> query;
            query.push_back(point.x());
            query.push_back(point.y());
            query.push_back(point.z());
            SearchParams sp(32, 0, true);
            //as L2 is default, search_radius is distance squared! 1km = 1000m => 1000*1000 = 1000000
            kd_tree.index->radiusSearch(&query[0], search_radius, result, sp);
            vector< pair<long, double> >::reverse_iterator it;
            //iterating in descening distance order
            for (it = result.rbegin(); it != result.rend(); ++it)
            {
                if (processed.find(it->first) != processed.end())
                {
                    break;
                }
                processed.insert(it->first);
                q.push(it->first);
            }
        }
        if (cluster.size() >= 1)
        {
            clusters.push_back(cluster);
        }
    }
    std::cout << "Done." << std::endl;
    return clusters;
}

vector< vector<IndexT> > SfMLoader::findEuclideanClusters(const Hash_Map<IndexT, Eigen::Vector3d> &_cloud)
{
    size_t point_count = _cloud.size();
    std::cout << "Original point count: " << point_count << std::endl;
    Eigen::MatrixXd cloud(point_count, 3);
    Hash_Map<IndexT, Eigen::Vector3d>::const_iterator it;
    Hash_Map<size_t, IndexT> indexMap;
    size_t idx = 0;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distrib(0.0,1.0);


    double ratio = 20000.0 / point_count;
    ratio = ratio >= 1.0 ? 1.0 : ratio;
    ratio = ratio < 0.01 ? 0.01 : ratio;
    std::cout << "Threshold ratio: " << ratio << std::endl;

    vector<osg::Vec3d> new_cloud_pts;
    int point_idx = 0;
    for (it = _cloud.cbegin(); it != _cloud.cend(); ++it)
    {
        //double number = distrib(generator);
        //if (number < ratio)
        if (point_idx % (int)(1.0 / ratio) == 0)
        {
            //std::cout << "sampled point idx: " << point_idx << std::endl;
            Eigen::Vector3d point = it->second;
            cloud.row(idx) = point;
            new_cloud_pts.push_back(osg::Vec3d(point.x(), point.y(), point.z()));
            indexMap.insert(pair<size_t, IndexT>(idx, it->first));
            ++idx;
            point_idx = 0;
        }
        ++point_idx;
    }
    //addPointCloud(mapNode, new_cloud_pts, osg::Vec4(1.0, 1.0, 0.0, 1.0));
    point_count = idx;
    cloud.resize(point_count, 3);
    std::cout << "Reduced point cout: " << point_count << std::endl;

    typedef KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree_t;
    kd_tree_t kd_tree(cloud, 10); //10 max leaf size
    kd_tree.index->buildIndex();

    vector< vector<IndexT> > clusters;
    set<size_t> processed;

    for (size_t i = 0; i < point_count; ++i)
    {
        if (i % 1000 == 0)
        {
            std::cout << "done: " << (double)i/point_count << "%" << std::endl;
        }
        vector<IndexT> cluster;
        queue<size_t> q;
        if (processed.find(i) == processed.end())
        {
            q.push(i);
        }
        while (!q.empty())
        {
            //std::cout << "queue size: " << q.size() << std::endl;
            size_t point_idx = q.front();
            processed.insert(point_idx);
            cluster.push_back(indexMap.find(point_idx)->second);
            q.pop();
            vector< pair<kd_tree_t::IndexType, double> > result;
            Eigen::Vector3d point = cloud.row(point_idx);
            vector<double> query;
            query.push_back(point.x());
            query.push_back(point.y());
            query.push_back(point.z());
            SearchParams sp(32, 0, true);
            kd_tree.index->radiusSearch(&query[0], 1000000, result, sp);
            vector< pair<long, double> >::reverse_iterator it;
            //iterating in descening distance order
            for (it = result.rbegin(); it != result.rend(); ++it)
            {
                if (processed.find(it->first) != processed.end())
                {
                    continue;
                }
                processed.insert(it->first);
                q.push(it->first);
            }
        }
        if (cluster.size() > 1)
        {
            clusters.push_back(cluster);
        }
    }
    std::cout << "Done." << std::endl;
    return clusters;
}

vector<Eigen::Vector3d> SfMLoader::createCloudFromData()
{
    Hash_Map<IndexT, Landmark>::const_iterator it;
    vector<Eigen::Vector3d> cloud;
    for (it = data.structure.begin(); it != data.structure.end(); ++it)
    {
        Eigen::Vector3d point = it->second.X;
        cloud.push_back(point);
    }

    return cloud;
}


Hash_Map<IndexT, Eigen::Vector3d> SfMLoader::createCloudMapFromDataForViewId(openMVG::IndexT viewId)
{
    init2D3D();

    /*Hash_Map<IndexT, Landmark>::const_iterator it;
    Hash_Map<IndexT, Eigen::Vector3d> cloud;
    for (it = data.structure.begin(); it != data.structure.end(); ++it)
    {
        Observations::const_iterator oit = it->second.obs.find(viewId);
        if (oit != it->second.obs.cend())
        {
            Eigen::Vector3d point = it->second.X;
            cloud.insert(std::make_pair(it->first, point));
        }
    }*/

    Hash_Map<IndexT, Hash_Map<IndexT, Eigen::Vector3d> >::iterator it = view_3D_track.find(viewId);

    return it->second;
}

Hash_Map<IndexT, Eigen::Vector3d> SfMLoader::createCloudMapFromData()
{
    Hash_Map<IndexT, Landmark>::const_iterator it;
    Hash_Map<IndexT, Eigen::Vector3d> cloud;
    for (it = data.structure.begin(); it != data.structure.end(); ++it)
    {
        Eigen::Vector3d point = it->second.X;
        cloud.insert(std::make_pair(it->first, point));
    }

    return cloud;
}



vector<Eigen::Vector3d> SfMLoader::createCloud(const Hash_Map<IndexT, Eigen::Vector3d> &_cloud)
{
    Hash_Map<IndexT, Eigen::Vector3d>::const_iterator it;
    vector<Eigen::Vector3d> cloud;
    for (it = _cloud.begin(); it != _cloud.end(); ++it)
    {
        Eigen::Vector3d point = it->second;
        cloud.push_back(point);
    }

    return cloud;
}

vector<osg::Vec3d> SfMLoader::sampleTerrain(const Hash_Map<IndexT, Eigen::Vector3d> &cloud)
{
    Eigen::Vector3d centroid;
    vector<Eigen::Vector3d> pcloud = createCloud(cloud);
    BBox3D whole_bbox = findPointCloudBBox(pcloud, centroid);
    double whole_side = (whole_bbox.max - whole_bbox.min).norm();
    SpatialReference *srs = SpatialReference::get("epsg:4326");
    vector<osg::Vec3d> points;
    std::cout << "whole side: " << whole_side << std::endl;
    if (whole_side > 30000)
    {
        std::cout << "Searching clusters..." << std::endl;
        vector< vector<IndexT> > clusters = findEuclideanClusters(cloud);
        vector< vector<IndexT> >::const_iterator it;
        vector< vector<IndexT> >::const_iterator cbegin = clusters.begin();
        std::cout << "Cluster count: " << clusters.size() << std::endl;
        for (it = clusters.begin(); it != clusters.end(); ++it)
        {
            Vector3d centroid;
            BBox3D bbox = findPointCloudBBox(*it, centroid);
            double side = fmin(((bbox.max - bbox.min).norm()), 5000.0);
            //std::cout << "side: " << side << std::endl;
            Vector3d center_bbox = ((bbox.max - bbox.min) / 2.0) + bbox.min;
            GeoPoint center_ecef(srs->getGeocentricSRS(),
                                 center_bbox.x(), center_bbox.y(), center_bbox.z(),
                                 ALTMODE_ABSOLUTE);
            GeoPoint center;
            center_ecef.transform(srs, center);

            double d = std::distance(cbegin, it);
            std::cout << "Done: "
                      << (d / (double)clusters.size()) * 100 << "%, "
                      << "bbox center: "
                      << center.y() << ", "<< center.x() << std::endl;

            if (center_lat >= -90 && center_lat <= 90 &&
                center_lon >= -180 && center_lon <= 180)
            {
                 /*std::cout << "Testing center: " << center_lat << ", "
                          << center_lon << std::endl;*/

                GeoPoint our_center(srs, center_lon, center_lat, 1.8, ALTMODE_RELATIVE);
                our_center.transformZ(ALTMODE_ABSOLUTE,
                                      mapNode->getTerrainEngine()->getTerrain());
                GeoPoint our_center_ecef;
                our_center.transform(srs->getGeocentricSRS(), our_center_ecef);

                osg::Vec3d center_xyz(center_ecef.x(), center_ecef.y(), center_ecef.z());
                osg::Vec3d our_center_xyz(our_center_ecef.x(),
                                          our_center_ecef.y(),
                                          our_center_ecef.z());

                double dist = (center_xyz - our_center_xyz).length();
                if (dist > 100000)
                {
                    std::cout << "Discarded due to big dist: " << dist << std::endl;
                    continue;
                }
            }
            //side + 1000: enlarge bbox by some margin
            GridSampler gs(mapNode, center, side, side, 10, 10);
            gs.sample(points);
        }
    }
    else
    {
        //bbox of the whole point cloud is smaller than 5km, we can sample it
        Vector3d center_bbox = ((whole_bbox.max - whole_bbox.min) / 2.0) + whole_bbox.min;
        GeoPoint center_ecef(srs->getGeocentricSRS(),
                             center_bbox.x(), center_bbox.y(), center_bbox.z(),
                             ALTMODE_ABSOLUTE);
        GeoPoint center;
        center_ecef.transform(srs, center);
        GridSampler gs(mapNode, center, whole_side, whole_side, 10, 10);
        gs.sample(points);
    }
    //create bbox around photo positions and sample it too
    sfm::Views::const_iterator vit;
    std::cout << "Views count: " << data.views.size() << std::endl;
    /*for (vit = data.views.begin(); vit != data.views.end(); ++vit)
    {
        const sfm::View &v = (*vit->second);
        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v.id_pose);
        if (poses_it != data.poses.end())
        {
            Eigen::Vector3d center_bbox = poses_it->second.center();

            GeoPoint center_ecef(srs->getGeocentricSRS(),
                                 center_bbox.x(), center_bbox.y(), center_bbox.z(),
                                 ALTMODE_ABSOLUTE);
            GeoPoint center;
            center_ecef.transform(srs, center);
            std::cout << "bbox center: " << center.y() << ", " << center.x() << std::endl;
            GridSampler gs(mapNode, center, 1000, 1000, 100, 100);
            gs.sample(points);
        }
    }*/
    return points;
}

void SfMLoader::alignPointCloudParts(MapNode *mapNode)
{
    //cluster photoviews by position
    vector< vector<IndexT> > clusters = findEuclideanClustersPhotoviews();
    std::cout << "Num Clusters Photoviews: " << clusters.size() << std::endl;
    using PLM = Hash_Map<IndexT, vector<IndexT> >;
    PLM photoviewsLandmarksMap; //key is photoview key, value is vector of tack ids.

    std::cout << "processing landmarks..." << std::endl;
    for (Landmarks::iterator lit = data.structure.begin(); lit != data.structure.end(); ++lit)
    {
        for (Observations::iterator iit = lit->second.obs.begin(); iit != lit->second.obs.end(); ++iit)
        {
            IndexT viewID = iit->first;
            Hash_Map<IndexT, std::shared_ptr<openMVG::sfm::View> >::iterator vit = data.views.find(viewID);
            PLM::iterator plmit = photoviewsLandmarksMap.find(vit->second->id_pose);
            if (plmit != photoviewsLandmarksMap.end())
            {
                plmit->second.push_back(lit->first);
            }
            else
            {
                vector<IndexT> v;
                v.push_back(lit->first);
                photoviewsLandmarksMap.insert(std::make_pair(vit->second->id_pose, v));
            }
        }
    }
    std::cout << "Done." << std::endl;
    /*
    }
    /**/
    for (vector<IndexT> &cluster : clusters)
    {
        //assign part of the original point cloud to each cluster
        vector<Eigen::Vector3d> cloud;
        Hash_Map<IndexT, Eigen::Vector3d> mapCloud;
        for(IndexT &i : cluster)
        {
            vector<IndexT> landmarkIds = photoviewsLandmarksMap.find(i)->second;
            for (IndexT &lid : landmarkIds)
            {
                Landmarks::iterator lit = data.structure.find(lid);
                cloud.push_back(lit->second.X);
                mapCloud.insert(std::make_pair(lit->first, lit->second.X));
            }
        }

        //we have corresponding point cloud, sample points, run icp, align
        vector<osg::Vec3d> terrain = sampleTerrain(mapCloud);
        //addPointCloud(mapNode, terrain);
        vector<osg::Vec3d> aligned_points;
        if (terrain.size() > 0)
        {
            aligned_T = runICP(terrain, cloud, aligned_points);
            addPointCloud(mapNode, aligned_points, osg::Vec4(0.0, 0.0, 1.0, 1.0));
            //update landmarks
            for(IndexT &i : cluster)
            {
                vector<IndexT> landmarkIds = photoviewsLandmarksMap.find(i)->second;
                for (IndexT &lid : landmarkIds)
                {
                    Landmarks::iterator lit = data.structure.find(lid);
                    Eigen::Vector3d point = lit->second.X;
                    Eigen::Vector4d ap = aligned_T * Eigen::Vector4d(point.x(), point.y(),
                                                                  point.z(), 1.0);
                    ap /= ap.w();
                    lit->second.X = Eigen::Vector3d(ap.x(), ap.y(), ap.z());
                }
            }


            //update camera poses
            sfm::Views::const_iterator vit;
            for(IndexT &i : cluster)
            {
                Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(i);
                if (poses_it != data.poses.end())
                {
                    Pose3 &pose = poses_it->second;
                    Eigen::Matrix3d rot = pose.rotation();
                    Eigen::Vector3d pos = pose.center();

                    Eigen::Matrix4d RT = Eigen::Matrix4d::Identity();
                    RT.block(0, 0, 3, 3) = rot;
                    RT.block(0, 3, 3, 1) = pos;
                    Eigen::Matrix4d nRT = aligned_T * RT;
                    pose.rotation() = nRT.block(0, 0, 3, 3);
                    pose.center() = nRT.block(0, 3, 3, 1);

                    rot = pose.rotation();
                    pos = pose.center();


                    Eigen::Quaternion<double> rotq(rot);
                    Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());

                    std::shared_ptr<AbstractPhotoView> pv = photoviews.find(i)->second;
                    pv->setOrientation(quat);
                    pv->setECEFPosition(pos[0], pos[1], pos[2]);
                }
            }
        }
    }

}

void SfMLoader::alignGPS()
{
    srand (time(NULL));
    std::vector<Eigen::Vector3d> X_SfM;
    std::vector<Eigen::Vector3d> X_GPS;
    std::vector<IndexT> has_gps;

    sfm::Views::iterator it;
    int gidx = 0;
    for (it = data.views.begin(); it != data.views.end(); ++it)
    {
        std::shared_ptr<sfm::View> v = it->second;
        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v->id_pose);
        if (poses_it != data.poses.end())
        {
            PhotoviewMap::iterator pv_it = photoviews.find(poses_it->first);
            std::shared_ptr<AbstractPhotoView> pv = pv_it->second;

            //FIXME: for debug, select random gps
            bool use = (rand() % 10 + 1) > 5;
            if (pv->hasGeoExif() && use)
            {
                osg::Vec3d p = pv->getECEFPosition();
                X_SfM.push_back(Eigen::Vector3d(p.x(), p.y(), p.z()));

                osg::Vec3d pe = pv->getGeoExif();
                X_GPS.push_back(Eigen::Vector3d(pe.x(), pe.y(), pe.z()));
                has_gps.push_back(pv_it->first);
                ++gidx;
            }
        }
    }
    /*
    if (X_GPS.size() > 0 && X_GPS.size() < 3)
    {
        std::cout << "Translating cluster to the GT GPS." << std::endl;
        //just translate according to the first gps
        osg::Vec3d gps = osg::Vec3d(X_GPS[0].x(), X_GPS[0].y(), X_GPS[0].z());
        shared_ptr<AbstractPhotoView> pv = photoviews.find(has_gps[0])->second;
        osg::Vec3d pos = pv->getECEFPosition();
        osg::Vec3d trans = gps - pos;
        openMVG::sfm::Pose3 pose(Mat3::Identity(), openMVG::Vec3(trans.x(), trans.y(), trans.z()));
        openMVG::geometry::Similarity3 sim(pose, 10000.0);
        openMVG::sfm::ApplySimilarity(sim, data);
        //update photoviews
        for (it = data.views.begin(); it != data.views.end(); ++it)
        {
            std::shared_ptr<sfm::View> v = it->second;
            Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v->id_pose);
            if (poses_it != data.poses.end())
            {
                PhotoviewMap::iterator pv_it = photoviews.find(poses_it->first);
                std::shared_ptr<AbstractPhotoView> pv = pv_it->second;

                osg::Vec3d pos = pv->getECEFPosition();
                Eigen::Vector3d epos(pos.x(), pos.y(), pos.z());
                epos = sim(epos);
                pv->setECEFPosition(epos.x(), epos.y(), epos.z());

                Pose3 pose = poses_it->second;
                Eigen::Matrix3d rot = pose.rotation().transpose();
                Eigen::Quaternion<double> rotq(rot);
                Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());
                pv->setOrientation(quat);
            }
        }
    }
    else */
    if (X_GPS.size() >= 3)
    {
        std::cout << "Calculating robust transform." << std::endl;

        const Mat X_SfM_Mat = Eigen::Map<Mat>(X_SfM[0].data(),3, X_SfM.size());
        const Mat X_GPS_Mat = Eigen::Map<Mat>(X_GPS[0].data(),3, X_GPS.size());

        std::cout << "xsfm: " << X_SfM_Mat.cols() << ", " << X_GPS_Mat.cols() << std::endl;

        //estimate registration
        geometry::kernel::Similarity3_Kernel kernel(X_SfM_Mat, X_GPS_Mat);
        openMVG::geometry::Similarity3 sim;

        /*
        //RIGID, all correspondences
        openMVG::Vec3 t;
        openMVG::Mat3 R;
        double S;
        if (!openMVG::geometry::FindRTS(X_SfM_Mat, X_GPS_Mat, &S, &t, &R))
        {
          std::cerr << "Failed to comute the registration" << std::endl;
          return;
        }

        std::cout
          << "Found transform:\n"
          << " scale: " << S << "\n"
          << " rotation:\n" << R << "\n"
          << " translation: " << std::fixed << std::setprecision(9)
          << t.transpose() << std::endl;

        // Encode the transformation as a 3D Similarity transformation matrix // S * R * X + t
        sim = openMVG::geometry::Similarity3(geometry::Pose3(R, -R.transpose()* t/S), S);

        // Compute & display fitting errors
        {
          const Vec vec_fitting_errors_eigen(
            geometry::kernel::Similarity3ErrorSquaredMetric::ErrorVec(sim, X_SfM_Mat, X_GPS_Mat).array().sqrt());
          std::cout << "\n3D Similarity fitting error (in target coordinate system units):";
          minMaxMeanMedian<float>(
            vec_fitting_errors_eigen.data(),
            vec_fitting_errors_eigen.data() + vec_fitting_errors_eigen.rows() );
        }*/

        //ROBUST
        const double lmeds_median = openMVG::robust::LeastMedianOfSquares(kernel, &sim);
        if (lmeds_median != std::numeric_limits<double>::max())
        {
            const Vec vec_fitting_errors_eigen(
              geometry::kernel::Similarity3ErrorSquaredMetric::ErrorVec(sim, X_SfM_Mat, X_GPS_Mat).array().sqrt());
            std::cout << "\n3D Similarity fitting error (in target coordinate system units):";
            minMaxMeanMedian<float>(
              vec_fitting_errors_eigen.data(),
              vec_fitting_errors_eigen.data() + vec_fitting_errors_eigen.rows() );

            //solution is usable
            // Apply the found transformation to the SfM Data Scene
            openMVG::sfm::ApplySimilarity(sim, data);
            updatePhotoviewsFromSfMData();
            this->loadPointCloud(mapNode);
        }
    }
    else
    {
        std::cout << "No valid GT GPS." << std::endl;
    }
}

void SfMLoader::updatePhotoviewsFromSfMData()
{
    sfm::Views::iterator it;
    for (it = data.views.begin(); it != data.views.end(); ++it)
    {
        std::shared_ptr<sfm::View> v = it->second;
        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v->id_pose);
        if (poses_it != data.poses.end())
        {
            PhotoviewMap::iterator pv_it = photoviews.find(poses_it->first);
            std::shared_ptr<AbstractPhotoView> pv = pv_it->second;

            Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase> >::iterator intr_it;
            intr_it = data.intrinsics.find(v->id_intrinsic);
            std::shared_ptr<cameras::IntrinsicBase> intr = intr_it->second;
            vector<double> intr_params = intr->getParams(); //focal length at 0
            double focal_pix = intr_params[0];
            double side = intr->w() > intr->h() ? intr->w() : intr->h();
            double fov = 2.0 * atan2((0.5 * side), focal_pix);
            pv->setFOV(fov);

            Pose3 pose = poses_it->second;
            Eigen::Matrix3d rot = pose.rotation().transpose();
            Eigen::Quaternion<double> rotq(rot);
            Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());
            pv->setOrientation(quat);

            Eigen::Vector3d position = pose.center();
            pv->setECEFPosition(position[0], position[1], position[2]);
        }
    }
}

void SfMLoader::alignPointCloudICP(MapNode *mapNode)
{
    Hash_Map<IndexT, Eigen::Vector3d> cloud = createCloudMapFromData();
    if (cloud.size() > 0)
    {
        vector<osg::Vec3d> points = sampleTerrain(cloud);
        alignPointCloudICP(mapNode, points);
    }
}

void SfMLoader::alignPointCloudICP(MapNode *mapNode, vector<osg::Vec3d> points)
{
    //TODO get bbox of the point cloud, measure the largest two dimensions
    //and center, create a geo rectangle covering the bbox, sample points in
    //this geo-rectangle, get elevations for sampled points.
    //filterOutliers();
    //vector<osg::Vec3d> points = sampleTerrain(mapNode);
    //show the sampled points
    if (sampled_pointcloud)
    {
        mapNode->removeChild(icp_aligned_pointcloud);
    }
    sampled_pointcloud = addPointCloud(mapNode, points, osg::Vec4(0, 1, 0, 1),
                                       5.0f); //elevate by 5 m so that it does
                                              //not clash with the terrain

    vector<osg::Vec3d> aligned_points;
    if (points.size() > 1)//1 needed for some reason so that ICP won't crash
    {
        std::vector<Eigen::Vector3d> cloud = createCloudFromData();
        if (cloud.size() > 0)
        {
            aligned_T = runICP(points, cloud, aligned_points);
            applyAlignmentToSfMDataStructure();
            updatePhotoviewsFromSfMData();
            //show the aligned points
            if (icp_aligned_pointcloud)
            {
                mapNode->removeChild(icp_aligned_pointcloud);
            }
            icp_aligned_pointcloud = addPointCloud(mapNode, aligned_points,
                                                   osg::Vec4(0.0, 0.0, 1.0, 1.0));
        }
        else
        {
            std::cerr << "SfM point cloud has no points." << std::endl;
        }
    }
}

void SfMLoader::applyAlignmentToSfMDataStructure()
{
    //update structure
    Hash_Map<IndexT, Landmark>::iterator it;
    for (it = data.structure.begin(); it != data.structure.end(); ++it)
    {
        Eigen::Vector3d point = it->second.X;
        Eigen::Vector4d ap = aligned_T * Eigen::Vector4d(point.x(), point.y(),
                                                      point.z(), 1.0);
        ap /= ap.w();
        it->second.X = Eigen::Vector3d(ap.x(), ap.y(), ap.z());
    }

    //update camera poses
    sfm::Views::const_iterator vit;
    for (vit = data.views.begin(); vit != data.views.end(); ++vit)
    {
        const sfm::View &v = (*vit->second);
        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v.id_pose);
        if (poses_it != data.poses.end())
        {
            Pose3 &pose = poses_it->second;
            Eigen::Matrix3d rot = pose.rotation();
            Eigen::Vector3d pos = pose.center();

            Eigen::Matrix4d RT = Eigen::Matrix4d::Identity();
            RT.block(0, 0, 3, 3) = rot;
            RT.block(0, 3, 3, 1) = pos;
            //OpenMVG stores center of the camera on world coords
            Eigen::Matrix4d nRT = aligned_T * RT;
            pose.center() = nRT.block(0, 3, 3, 1);

            //OpenMVG rotation matrix is NOT camera pose, but EXTRINSIC!
            //Therefore we need to multiply with inverse transform
            //in reversed order!
            nRT = RT * aligned_T.inverse();
            pose.rotation() = nRT.block(0, 0, 3, 3);
        }
    }
}


bool SfMLoader::isPano(const string &imageName)
{
    fs::path p(imageName);
    string basename = fs::basename(p);
    boost::regex pattern(".*_[0-9]+_[0-9]+_[0-9]+$");
    return boost::regex_match(basename, pattern);
}

string SfMLoader::getPanoFilePath(const string &imageName)
{
    fs::path component_dir(data_dir);
    fs::path root = component_dir.parent_path().parent_path()
                                 .parent_path().parent_path();
    fs::path img_name(imageName);
    string img_basename = fs::basename(img_name);
    boost::regex pattern("_[0-9]+_[0-9]+_[0-9]+$");
    string pano_basename = boost::regex_replace(img_basename, pattern, "");
    fs::path pano_path = root / fs::path("pano") /
                         fs::path(pano_basename + ".jpg");
    return pano_path.string();
}

int SfMLoader::getPanoYaw(const string &imageName)
{
    boost::regex re("_[0-9]+_[0-9]+_[0-9]+$");
    boost::sregex_token_iterator i(imageName.begin(), imageName.end(), re, -1);
    boost::sregex_token_iterator j;

    int yaw = 0;
    if (i != j)
    {
        string matched = *i;
        vector<string> strs;
        boost::split(strs, matched, boost::is_any_of("_"));
        try
        {
            yaw = stoi(strs[1]);
        }
        catch (invalid_argument &ia)
        {
            std::cerr << "Unable to parse int yaw value from: " << strs[1]
                      << ", for image: " << imageName
                      << ", fallback to yaw = 0." << std::endl;
            yaw = 0;
        }
    }
    else
    {
        std::cerr << "Unable to parse int yaw value for image: " << imageName
                  << ", fallback to yaw = 0." << std::endl;
        yaw = 0;
    }
    return yaw;
}

set< shared_ptr<AbstractPhotoView> > SfMLoader::loadPhotoparams(MapNode *mapNode)
{
    //set<string> loaded_pano;
    Hash_Map<string, shared_ptr<AbstractPhotoView> > map_name_pv;
    typedef Hash_Map<string, shared_ptr<AbstractPhotoView> >::iterator map_name_pv_iterator;
    set< shared_ptr<AbstractPhotoView> > photoviews_set;
    sfm::Views::const_iterator it;
    int idx = 0;
    std::cout << "loading photoviews: " << data.views.size() << std::endl;
    for (it = data.views.begin(); it != data.views.end(); ++it)
    {
        /*if (idx > 100)
        {
            break;
        }*/
        const sfm::View &v = (*it->second);

        //load position and orientation from sfm data
        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v.id_pose);
        if (poses_it != data.poses.end())
        {
            //load photo view
            shared_ptr<AbstractPhotoView> pv(nullptr);
            fs::path img_path = fs::path(data_dir) / fs::path(v.s_Img_path);
            if (false && isPano(v.s_Img_path))
            {
                string pano_path = getPanoFilePath(v.s_Img_path);
                map_name_pv_iterator mpvit = map_name_pv.find(pano_path);
                if (mpvit == map_name_pv.end())
                {
                    int pano_yaw = getPanoYaw(v.s_Img_path);
                    pv = make_shared<PanoPhotoView>(mapNode, pano_path, pano_yaw);
                    map_name_pv.insert(std::make_pair(pano_path, pv));
                    photoviews_set.insert(pv);
                }
                else
                {
                    pv = mpvit->second;
                }
            }
            else
            {
                try{
                    pv = make_shared<PhotoView>(mapNode, img_path.string());
                    photoviews_set.insert(pv);
                }
                catch (std::runtime_error &re)
                {
                    std::cout << "Skipping photoview: " << img_path.string() << ", unable to load." << std::endl;
                }

            }
            if (pv)
            {
                std::cout << "Loading photoview: " << pv->getPhotoName() << std::endl;
                Pose3 pose = poses_it->second;
                Eigen::Matrix3d rot = pose.rotation().transpose();
                Eigen::Vector3d position = pose.center();

                Eigen::Quaternion<double> rotq(rot);
                Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());

                Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase> >::iterator intr_it;
                intr_it = data.intrinsics.find(v.id_intrinsic);
                std::shared_ptr<cameras::IntrinsicBase> intr = intr_it->second;
                vector<double> intr_params = intr->getParams(); //focal length at 0
                double focal_pix = intr_params[0];
                double side = intr->w() > intr->h() ? intr->w() : intr->h();
                double fov = 2.0 * atan2((0.5 * side), focal_pix);

                pv->setFOV(fov);
                pv->setOrientation(quat);
                pv->setECEFPosition(position[0], position[1], position[2]);
                photoviews.insert(std::make_pair(poses_it->first, pv));
                photoviews_views.insert(std::make_pair(it->first, pv));
                ++idx;
            }
        }
    }
    return photoviews_set;
}

Eigen::Matrix3d SfMLoader::KabschAlgorithm(const Eigen::MatrixXd &P,
                                           const Eigen::MatrixXd &Q,
                                           double &out_scale)
{
    assert(P.cols() == 3);
    assert(Q.cols() == 3);
    assert(P.rows() == Q.rows());

    //calculate std of P
    double P_std_2 = 0;
    for (int i = 0; i < P.rows(); ++i)
    {
        P_std_2 += P.row(i).squaredNorm();
    }

    Eigen::MatrixXd A = P.transpose() * Q;
    Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    Eigen::Vector3d e(1, 1, 1);
    if (R.determinant() < 0)
    {
        e = Eigen::Vector3d(1, 1, -1);
        //multiply 3rd column by -1
        R.block(0, 2, 3, 1) = -R.block(0, 2, 3, 1);
    }
    out_scale = svd.singularValues().dot(e) / P_std_2;
    return R;
}

int randomize(const int &i)
{
    return rand() % i;
}

vector<int> SfMLoader::random_vector(int n, int k)
{
    vector<int> sample(n);
    // generate numbers 0 .. n-1
    iota(sample.begin(), sample.end(), 0);
    // shuffle elements
    random_shuffle(sample.begin(), sample.end(), randomize);
    // grab the first five elements after shuffling
    vector<int> chosen(sample.begin(), sample.begin() + k);
    return chosen;
}

std::pair<double, std::shared_ptr<AbstractPhotoView>> SfMLoader::findNearestPhotoviewWithExifGPS(osg::Vec3d pos)
{
    double min_dist = std::numeric_limits<double>::max();
    std::shared_ptr<AbstractPhotoView> currentPhotoview;
    PhotoviewMap::iterator pv_it;
    for (pv_it = photoviews.begin(); pv_it != photoviews.end(); ++pv_it)
    {
        std::shared_ptr<AbstractPhotoView> photoview = pv_it->second;
        if (photoview->hasGeoExif())
        {
            osg::Vec3d p = photoview->getGeoExif();
            double dist = (pos - p).length();
            if (dist < min_dist)
            {
                min_dist = dist;
                currentPhotoview = photoview;
            }
        }
    }
    return std::make_pair(min_dist, currentPhotoview);
}


void SfMLoader::init2D3D()
{
    // FIXME: commented out lazy initialization due to a problem when ICP is
    // calculated. If lazy init is turned on, the variables are initialized
    // before ICP transform is applied to the data. This causes that
    // reprojectionRMSE returns wrong values. Therefore, this has to be
    // commented out until this problem is resolved.

    /*if (loaded2D3D)
    {
        return;
    }*/

    view_3D.clear();
    view_3D_track.clear();
    view_2D.clear();
    viewLandmarksMap.clear();

    Landmarks::const_iterator lit;
    for (lit = data.structure.begin(); lit != data.structure.end(); ++lit)
    {
        Observations::const_iterator obs_it;
        for (obs_it = lit->second.obs.begin(); obs_it != lit->second.obs.end(); ++obs_it)
        {
            IndexT viewID = obs_it->first;
            Eigen::Vector2d p_2d = obs_it->second.x;
            Eigen::Vector3d p_3d = lit->second.X;
            Hash_Map<IndexT, vector<Eigen::Vector3d> >::iterator it_3d = view_3D.find(viewID);
            Hash_Map<IndexT, Hash_Map<IndexT, Eigen::Vector3d> >::iterator it_3d_track = view_3D_track.find(viewID);
            Hash_Map<IndexT, vector<Eigen::Vector2d> >::iterator it_2d = view_2D.find(viewID);

            if (it_3d != view_3D.end())
            {
                vector<Eigen::Vector3d> &m3d = it_3d->second;
                m3d.push_back(p_3d);
            }
            else
            {
                vector<Eigen::Vector3d> m3d;
                m3d.push_back(p_3d);
                view_3D.insert(std::make_pair(viewID, m3d));
            }

            if (it_3d_track != view_3D_track.end())
            {
                Hash_Map<IndexT, Eigen::Vector3d> &m3dt = it_3d_track->second;
                m3dt.insert(std::make_pair(lit->first, p_3d));
            }
            else
            {
                Hash_Map<IndexT, Eigen::Vector3d> m3dt;
                m3dt.insert(std::make_pair(lit->first, p_3d));
                view_3D_track.insert(std::make_pair(viewID, m3dt));
            }

            if (it_2d != view_2D.end())
            {
                vector<Eigen::Vector2d> &m2d = it_2d->second;
                m2d.push_back(p_2d);
            }
            else
            {
                vector<Eigen::Vector2d> m2d;
                m2d.push_back(p_2d);
                view_2D.insert(std::make_pair(viewID, m2d));
            }

            VLM::iterator vlmit = viewLandmarksMap.find(viewID);
            if (vlmit != viewLandmarksMap.end())
            {
                vlmit->second.push_back(lit->first);
            }
            else
            {
                vector<IndexT> v;
                v.push_back(lit->first);
                viewLandmarksMap.insert(std::make_pair(viewID, v));
            }
        }

    }

    loaded2D3D = true;
}

void SfMLoader::adjustRotationScale(MapNode *mapNode)
{
    //reproject 2D points with unit rotation and known distance from
    //camera center to corresponding 3D point and then estimate
    //rotation using Kabsch algorithm.
    init2D3D();

    sfm::Views::const_iterator it;
    int cnt = 0;
    //intrinsics indexed by pose_id
    Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase> > intrinsics;
    for (it = data.views.begin(); it != data.views.end(); ++it)
    {
        const sfm::View &v = (*it->second);
        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v.id_pose);
        if (poses_it != data.poses.end())
        {
            Hash_Map<IndexT, shared_ptr<AbstractPhotoView> >::iterator pv_it =
                    photoviews.find(poses_it->first);
            shared_ptr<AbstractPhotoView> pv = pv_it->second;

            Hash_Map<IndexT, vector<Eigen::Vector3d> >::iterator it_3d = view_3D.find(v.id_view);
            Hash_Map<IndexT, vector<Eigen::Vector2d> >::iterator it_2d = view_2D.find(v.id_view);
            Pose3 &pose = poses_it->second;
            //load intrinsic
            std::shared_ptr<cameras::IntrinsicBase> intr = data.intrinsics.find(v.id_intrinsic)->second;
            intrinsics.insert(std::make_pair(v.id_pose, intr));
            vector<double> intr_params = intr->getParams(); //focal length at 0
            double focal_pix = intr_params[0];
            double plane_width = intr->w() / focal_pix;
            double plane_height = intr->h() / focal_pix;
            /*double aspect = (intr->h() / intr->w());
            double plane_height = plane_width * aspect;*/

            if (it_3d != view_3D.end())
            {
                vector<Eigen::Vector3d> &m3d = it_3d->second;
                vector<Eigen::Vector2d> &m2d = it_2d->second;

                vector<Eigen::Vector3d> sel_m3d;
                vector<Eigen::Vector3d> proj_m2d;
                if (m3d.size() < 3)
                {
                    pv->hide();
                    std::cout << "Lower than 3 point correspondences for view id: " << v.id_view << std::endl;
                    continue;
                }

                assert(m3d.size() == m2d.size());
                vector<int> randvec = random_vector(m3d.size() - 1, std::min((int)m3d.size() - 1, (int)m3d.size() - 1));

                Eigen::MatrixXd m3d_mat(randvec.size(), 3);
                Eigen::MatrixXd proj_2d_mat(randvec.size(), 3);

                //std::cout << "dists: ";
                for (size_t i = 0; i < randvec.size(); ++i)
                {
                    Eigen::Vector3d &p3d = m3d[randvec[i]];
                    Eigen::Vector2d &p2d = m2d[randvec[i]];
                    //normalize absolute 2d point to world coords
                    Eigen::Vector2d np2d = p2d / focal_pix;
                    Eigen::Vector3d np2d_w(np2d.x() - (plane_width / 2.0), np2d.y() - (plane_height / 2.0), 1.0);
                    double dist = (p3d - pose.center()).norm();
                    if (dist > 10000)
                    {
                        std::cout << "detected too big dist: " << dist << ", view id: " << v.id_view << std::endl;;
                    }
                    //
                    Eigen::Vector3d proj_2d = (np2d_w.normalized() * dist);
                    proj_m2d.push_back(proj_2d);
                    sel_m3d.push_back(p3d);

                    //visualize point correspondences
                    /*osg::Vec3d pc(pose.center().x(), pose.center().y(), pose.center().z());
                    Eigen::Vector3d tar = proj_2d + pose.center();
                    osg::Vec3d t(tar.x(), tar.y(), tar.z());
                    Geode *line = PhotoView::createLine(osg::Vec4(0.0, 0.0, 1.0, 1.0),
                                          pc,
                                          t);
                    mapNode->addChild(line);*/
                }
                //find centroids
                Eigen::Vector3d c1 = Eigen::Vector3d(0.0, 0.0, 0.0);
                Eigen::Vector3d c2 = Eigen::Vector3d(0.0, 0.0, 0.0);
                for (size_t i = 0; i < sel_m3d.size(); ++i)
                {
                    Eigen::Vector3d &p3d = sel_m3d[i];
                    Eigen::Vector3d &p2d = proj_m2d[i];
                    c1 += p3d / (double)sel_m3d.size();
                    c2 += p2d / (double)sel_m3d.size();
                }
                //translate to centroid
                for (size_t i = 0; i < sel_m3d.size(); ++i)
                {
                    m3d_mat.row(i) = sel_m3d[i] - c1;
                    proj_2d_mat.row(i) = proj_m2d[i] - c2;
                }

                //now we have 2 3D point clouds, original m3d_mat and
                //projected proj_2d_mat centered at world origin.
                //Now apply Kabsch algorithm to estimate the orientation
                double out_scale = 1.0;
                Eigen::Matrix3d R = KabschAlgorithm(proj_2d_mat, m3d_mat, out_scale);
                poses_it->second.rotation() = R.transpose();
                std::cout << "out scale: " << out_scale << std::endl;
                intr_params[0] = intr_params[0] * out_scale;

                Eigen::Matrix3d rot = poses_it->second.rotation();
                Eigen::Vector3d pos = poses_it->second.center();
                Eigen::Matrix4d RT = Eigen::Matrix4d::Identity();
                RT.block(0, 0, 3, 3) = rot;
                RT.block(0, 3, 3, 1) = pos;

                /*
                vector<double> focals;
                //calculate focal length using least squares
                int num_trials = 20;
                for (int j = 0; j < num_trials; ++j)
                {
                    vector<int> randvec = random_vector(m3d.size() - 1, std::min((int)m3d.size() - 1, 5));
                    MatrixXd A = MatrixXd::Zero(m3d.size(), 1);
                    MatrixXd b = MatrixXd::Zero(m3d.size(), 1);
                    for (size_t i = 0; i < randvec.size(); ++i)
                    {
                        Eigen::Vector4d m3dh;
                        m3dh << m3d[randvec[i]].x(), m3d[randvec[i]].y(), m3d[randvec[i]].z(), 1.0;
                        Eigen::Vector4d m3dhp = RT * m3dh;
                        A(i) = m3dhp[0];
                        b(i) = m2d[randvec[i]].x() - 0.5 * m3dhp.z();
                        //std::cout << "m2d:" << m2d[randvec[i]].x() << ", " << m2d[randvec[i]].y() << std::endl;
                    }
                    MatrixXd res = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
                    double w = intr->w() > intr->h() ? intr->w() : intr->h();
                    double f =  res(0) * w;
                    focals.push_back(f);
                    //std::cout << "iter: " << j <<  ", old focal: " << focal_pix <<  ", estimated focal: " << f << ", w: " << intr->w() << ", h: " << intr->h() << std::endl;
                }
                //median of all focals
                std::sort(focals.begin(), focals.end());
                double f = focals[num_trials/2];
                if (f > 0)
                {
                    //otherwise not viable
                    //intr_params[0] = f;
                    //intr->updateFromParams(intr_params);
                    std::cout << "USED FOCAL: " << ", old focal: " << focal_pix <<  ", estimated focal: " << f << ", pose id:" << poses_it->first << std::endl;
                }
                else
                {
                    std::cout << "WRONGLY ESTIMATED FOCAL LENGTH: " << f << std::endl;
                }*/

                //visualize projected points with found rotation

                vector<osg::Vec3d> cloud;
                vector<osg::Vec3d> rotated_cloud;
                std::cout << "size: " << sel_m3d.size() << ", c1: " << c1.x() << ", " << c1.y() << ", " << c1.z() << std::endl;
                std::cout << "c2: " << c2.x() << ", " << c2.y() << ", " << c2.z() << std::endl;
                for (int i = 0; i < sel_m3d.size(); ++i)
                {
                    Eigen::Vector3d p = proj_2d_mat.row(i).transpose() + c2 + pose.center();
                    Eigen::Vector3d pr = (R * proj_2d_mat.row(i).transpose()) + c1;
                    //std::cout << pr.x() << ", " << pr.y() << ", " << pr.z() << std::endl;
                    if (isnan(pr.x()) || isnan(pr.y()) || isnan(pr.z()))
                    {
                        std::cout << "Nan detected! View id: " << v.id_view << std::endl;
                    }
                    else
                    {
                        cloud.push_back(osg::Vec3d(p.x(), p.y(), p.z()));
                        rotated_cloud.push_back(osg::Vec3d(pr.x(), pr.y(), pr.z()));
                    }
                }
                addPointCloud(mapNode, cloud, osg::Vec4(0.0, 1.0, 0.0, 1.0));
                addPointCloud(mapNode, rotated_cloud, osg::Vec4(0.0, 0.0, 1.0, 1.0));
                ++cnt;
            }
        }
    }

    //update rotations of the photoviews
    Hash_Map<IndexT, shared_ptr<AbstractPhotoView> >::iterator pit;
    for (pit = photoviews.begin(); pit != photoviews.end(); ++pit)
    {
        shared_ptr<AbstractPhotoView> pv = pit->second;
        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(pit->first);
        if (poses_it != data.poses.end())
        {
            std::cout << "updating orientation of photoview with pose_id: " << pit->first << std::endl;
            Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase> >::iterator intr_it;
            intr_it = intrinsics.find(pit->first);
            std::shared_ptr<cameras::IntrinsicBase> intr = intr_it->second;
            vector<double> intr_params = intr->getParams(); //focal length at 0
            double focal_pix = intr_params[0];
            double side = intr->w() > intr->h() ? intr->w() : intr->h();
            double fov = 2.0 * atan2((0.5 * side), focal_pix);
            std::cout << "retrieved focal pix: " << focal_pix << ", FOV:" << fov << std::endl;
            Pose3 pose = poses_it->second;
            Eigen::Matrix3d rot = pose.rotation().transpose();
            //Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
            Eigen::Quaternion<double> rotq(rot);
            Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());
            pv->setOrientation(quat);
            pv->setFOV(fov);
        }
    }
}

void SfMLoader::fitPhotoViewsToTerrain()
{
    init2D3D();

    Hash_Map<IndexT, shared_ptr<AbstractPhotoView> >::iterator it;

    //fit photoviews and corresponding poses in SfM_data to the terrain
    for (it = photoviews_views.begin(); it != photoviews_views.end(); ++it)
    {
        shared_ptr<AbstractPhotoView> pv = it->second;
        float above_ground = pv->distanceAboveGround();
        std::cout << "above ground: " << above_ground << std::endl;
        if ((above_ground > 0 || above_ground < -100))
        {
            //do not fit to terrain iff the photoview is above ground or
            //more than 100m below ground.
            std::cout << "Skipping fit to terrain due to elevation." << std::endl;
            continue;
        }

        osg::Vec3d op = pv->getECEFPosition();
        Eigen::Vector3d orig_pos = Eigen::Vector3d(op.x(), op.y(), op.z());
        pv->fitPositionToTerrain();
        IndexT viewId = it->first;
        Hash_Map<IndexT, std::shared_ptr<openMVG::sfm::View>>::iterator vit = data.views.find(viewId);
        if (vit != data.views.end())
        {
            IndexT poseId = vit->second->id_pose;
            Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(poseId);
            if (poses_it != data.poses.end())
            {
                Hash_Map<IndexT, vector<Eigen::Vector3d> >::iterator it_3d = view_3D.find(viewId);
                if (it_3d != view_3D.end())
                {
                    vector<Eigen::Vector3d> m3d = it_3d->second;
                    //find centroids
                    Eigen::Vector3d c1 = Eigen::Vector3d(0.0, 0.0, 0.0);
                    for (size_t i = 0; i < m3d.size(); ++i)
                    {
                        Eigen::Vector3d &p3d = m3d[i];
                        c1 += p3d / (double)m3d.size();
                    }
                    osg::Vec3d fp = pv->getECEFPosition();
                    Eigen::Vector3d fit_pos = Eigen::Vector3d(fp.x(), fp.y(), fp.z());

                    // vector looking from the original camera position
                    // to the centroid of the 3D point cloud
                    Eigen::Vector3d v_orig = (c1 - orig_pos).normalized();
                    // vector looking from the new camera position
                    // to the centroid of the 3D point cloud.
                    Eigen::Vector3d v_fit = (c1 - fit_pos).normalized();
                    // find the rotation matrix between the old and new vector
                    // (taken from Rik's answer https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/180436#180436)
                    Eigen::Vector3d v = v_orig.cross(v_fit);
                    Eigen::Matrix3d v_x;
                    v_x << 0,       -v[2],      v[1],
                           v[2],    0,          -v[0],
                           -v[1],    v[0],       0;
                    double c = v_orig.dot(v_fit);
                    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + v_x + (v_x.array().pow(2).matrix() * 1.0 / (1 + c));
                    poses_it->second.rotation() *= R.transpose();

                    Eigen::Matrix3d rot = poses_it->second.rotation().transpose();
                    Eigen::Quaternion<double> rotq(rot);
                    Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());
                    pv->setOrientation(quat);

                    std::cout << "updating position of pose: " << it->first << std::endl;
                    poses_it->second.center().x() = fp.x();
                    poses_it->second.center().y() = fp.y();
                    poses_it->second.center().z() = fp.z();
                }
            }
        }
    }
}


int SfMLoader::getViewsCount()
{
    return data.views.size();
}

string SfMLoader::findSfMDataICP(const string &path)
{
    vector<string> sfm_data_vec = Util::findFileRecursively(path,
                                                            "sfm_data_icp", ".bin");
    if (sfm_data_vec.size() > 0)
    {
        return sfm_data_vec[0];
    }
    else
    {
        throw std::runtime_error("Unable to find sfm_data.bin at path: " +path);
    }
}

string SfMLoader::findSfMData(const string &path)
{
    vector<string> sfm_data_vec = Util::findFileRecursively(path,
                                                            "sfm_data", ".bin");
    if (sfm_data_vec.size() > 0)
    {
        return sfm_data_vec[0];
    }
    else
    {
        throw std::runtime_error("Unable to find sfm_data.bin at path: " +path);
    }
}

string SfMLoader::findNVMData(const string &path)
{
    vector<string> nvm_vec = Util::findFileRecursively(path, "sfm_data", ".nvm");
    if (nvm_vec.size() > 0)
    {
        return nvm_vec[0];
    }
    else
    {
        throw std::runtime_error("Unable to find sfm_data.nvm at path: " +path);
    }
}


void SfMLoader::toggleViewState(MapNode *mapNode)
{

    view_state = static_cast<SfMLoader::ViewState>((view_state + 1) %
                                                   VIEW_STATE_LAST);
    updateViewState(mapNode);
}

void SfMLoader::updateViewState(MapNode *mapNode)
{
    if (orig_pointcloud.valid())
        mapNode->removeChild(orig_pointcloud);
    if (sampled_pointcloud.valid())
        mapNode->removeChild(sampled_pointcloud);
    if (icp_aligned_pointcloud.valid())
        mapNode->removeChild(icp_aligned_pointcloud);

    switch (view_state) {
    case SHOW_ALL:
    {
        if (orig_pointcloud.valid())
            mapNode->addChild(orig_pointcloud);
        if (sampled_pointcloud.valid())
            mapNode->addChild(sampled_pointcloud);
        //if (icp_aligned_pointcloud.valid())
        //    mapNode->addChild(icp_aligned_pointcloud);
        break;
    }
    case SHOW_ORIG:
    {
        if (orig_pointcloud.valid())
            mapNode->addChild(orig_pointcloud);
        break;
    }
    case SHOW_ALIGNED_ICP:
    {
        if (icp_aligned_pointcloud.valid())
            mapNode->addChild(icp_aligned_pointcloud);
        if (sampled_pointcloud.valid())
            mapNode->addChild(sampled_pointcloud);
        break;
    }
    case SHOW_SAMPLED:
    {
        if (sampled_pointcloud.valid())
            mapNode->addChild(sampled_pointcloud);
        break;
    }
    case SHOW_NONE:
        break;
    default:
        break;
    }
}

string SfMLoader::getComponentDir()
{
    return data_dir;
}

void SfMLoader::saveSfmData(string filename)
{
    fs::path compdir(data_dir);
    string sfm_data_icp_path = (compdir / fs::path(filename)).string();
    openMVG::sfm::Save(data, sfm_data_icp_path,
                       (ESfM_Data)(VIEWS | EXTRINSICS |
                        INTRINSICS | STRUCTURE));
}


vector<openMVG::IndexT> SfMLoader::getPhotoviewKeys(double search_radius_sq)
{
    vector< vector<IndexT> >::iterator cit;
    vector<openMVG::IndexT> keys;

    vector< vector<IndexT> > clusters = findEuclideanClustersPhotoviews(search_radius_sq);
    std::cout << "Number of photoview clusters: " << clusters.size() << std::endl;

    for (cit = clusters.begin(); cit != clusters.end(); ++cit)
    {
        std::cout << "Current photoview cluster size: " << cit->size() << std::endl;
        for (vector<IndexT>::iterator it = cit->begin(); it != cit->end(); ++it)
        {
            keys.push_back(*it);
        }
    }
    return keys;
}


shared_ptr<AbstractPhotoView> SfMLoader::getPhotoView(openMVG::IndexT viewId)
{
    return photoviews_views[viewId];
}


std::tuple<int, double, double> SfMLoader::getPhotoViewError(openMVG::IndexT viewId,
                                                             const Hash_Map<IndexT, Eigen::Vector3d> &cld,
                                                             const vector<osg::Vec3d> &points)
{
    std::tuple<int, double, double, vector<double> > st = calculate3DStats(cld, points);

    return std::make_tuple(std::get<0>(st), std::get<1>(st), std::get<2>(st));
}

std::tuple<int, double, double, vector<double> > SfMLoader::calculate3DStatsForData()
{
    Hash_Map<IndexT, Eigen::Vector3d> cld = createCloudMapFromData();
    vector<osg::Vec3d> points = sampleTerrain(cld);

    if (cld.size() > 0)
    {
        return calculate3DStats(cld, points);
    }
    else
    {
        vector<double> empty;
        return std::make_tuple(0, 0, 0, empty);
    }
}

std::tuple<int, double, double, vector<double> > SfMLoader::calculate3DStats(const Hash_Map<IndexT, Eigen::Vector3d> &cld,
                                                                             const vector<osg::Vec3d> &points)
{
    Hash_Map<IndexT, Eigen::Vector3d> _cld(cld);
    Eigen::Vector3d c1 = Eigen::Vector3d(0.0, 0.0, 0.0);
    //Eigen::Vector3d c2 = Eigen::Vector3d(0.0, 0.0, 0.0);
    Hash_Map<IndexT, Eigen::Vector3d>::iterator cit;
    for (cit = _cld.begin(); cit != _cld.end(); ++ cit)
    {
        c1 += cit->second / (double)cld.size();
    }
    //move the point cloud to origin
    for (cit = _cld.begin(); cit != _cld.end(); ++ cit)
    {
        cit->second -= c1;
    }


    //build KD tree for fast search
    Eigen::MatrixXd cloud(points.size(), 3);
    int idx = 0;
    std::vector<osg::Vec3d>::const_iterator it;
    for (it = points.cbegin(); it != points.cend(); ++it)
    {
        Eigen::Vector3d point = Eigen::Vector3d(it->x(), it->y(), it->z()) - c1;
        //c2 += point / (double)points.size();
        cloud.row(idx) = point;
        ++idx;
    }
    //move the sampled poitns to origin
    //already moved to origin c1.
    /*
    for (int i = 0; i < points.size(); ++i)
    {
        cloud.row(i) -= c2;
    }*/

    typedef KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree_t;
    kd_tree_t kd_tree(cloud, 10); //10 max leaf size
    kd_tree.index->buildIndex();

    Hash_Map<IndexT, Eigen::Vector3d>::iterator cid;
    double mean_dist = 0;
    vector<double> dists;
    for (cid = _cld.begin(); cid != _cld.end(); ++cid)
    {
        long out_idx[1];
        double out_dist[1];
        Eigen::Vector3d curr_pt = cid->second;
        kd_tree.index->knnSearch(&curr_pt[0], 1, out_idx, out_dist);
        dists.push_back(out_dist[0]);
        mean_dist += out_dist[0] / cld.size();
        //Vec3 nearest_point = cloud.row(out_idx[0]);
    }
    std::sort(dists.begin(), dists.end());
    return std::tuple<int, double, double, vector<double> >(cld.size(),
                                   mean_dist,
                                   dists[dists.size() / 2], dists);
}

int SfMLoader::pointCount()
{
    return data.structure.size();
}

vector<double> SfMLoader::reprojectionRMSE(IndexT viewID)
{
    vector<double> res;
    init2D3D();
    std::shared_ptr<sfm::View> v = data.views[viewID];
    IndexT id_intr = v->id_intrinsic;
    Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase> >::iterator intr_it;
    intr_it = data.intrinsics.find(id_intr);
    std::shared_ptr<cameras::IntrinsicBase> intr = intr_it->second;

    Hash_Map<IndexT, vector<Eigen::Vector3d> >::iterator it_3d = view_3D.find(viewID);
    Hash_Map<IndexT, vector<Eigen::Vector2d> >::iterator it_2d = view_2D.find(viewID);

    if (it_3d != view_3D.end())
    {
        vector<Eigen::Vector3d> &m3d = it_3d->second;
        vector<Eigen::Vector2d> &m2d = it_2d->second;

        Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v->id_pose);
        if (poses_it != data.poses.end())
        {
            double sqnorm = 0;
            int cnt = 0;
            for (int i = 0; i < m3d.size(); ++i)
            {
                //Pose3 pose = poses_it->second;
                //const Eigen::Vector3d X = pose(m3d[i]);
                //std::cout << "projected point: " << X.hnormalized() << std::endl;
                Eigen::Vector2d p2d = intr->project(poses_it->second(m3d[i]));
                if (p2d.x() >= 0 && p2d.y() >= 0 && p2d.x() <= intr->w() && p2d.y() <= intr->h())
                {
                    //projection makes sense
                    sqnorm += (p2d - m2d[i]).squaredNorm();
                    ++cnt;
                    /*std::cout << "proj: " << p2d.x() << ", " << p2d.y()
                              << ", obs: " << m2d[i].x() << ", "
                              << m2d[i].y() << std::endl;

                    std::cout << "sqnorm: " << (p2d - m2d[i]).squaredNorm()
                              << ", sum: " << sqnorm << std::endl;*/
                }

            }
            if (sqnorm > 0)
            {
                res.push_back(std::sqrt(sqnorm / cnt));
            }
        }
    }

    return res;
}

vector<double> SfMLoader::reprojectionRMSE()
{
    vector<double> res;
    init2D3D();
    sfm::Views::iterator it;
    for (it = data.views.begin(); it != data.views.end(); ++it)
    {
        vector<double> view_res = reprojectionRMSE(it->first);
        std::copy(view_res.begin(), view_res.end(),
                      std::back_inserter(res));
    }
    return res;
}


bool SfMLoader::savePhotoview3DProjections(openMVG::IndexT viewID,
                                           const Hash_Map<IndexT, Eigen::Vector3d> &cld,
                                           const vector<osg::Vec3d> &points,
                                           string sfm_projections_filename,
                                           string sfm_observations_filename,
                                           string terrain_projections_filename)
{
    init2D3D();

    Hash_Map<IndexT, vector<Eigen::Vector2d> >::iterator it_2d = view_2D.find(viewID);
    Hash_Map<IndexT, vector<Eigen::Vector3d> >::iterator it_3d = view_3D.find(viewID);

    std::shared_ptr<sfm::View> v = data.views[viewID];
    Hash_Map<IndexT, Pose3>::iterator poses_it = data.poses.find(v->id_pose);
    if (poses_it == data.poses.end())
    {
        std::cerr << "Pose for view ID: " << viewID << " NOT FOUND!" << std::endl;
        return false;
    }
    Pose3 &pose = poses_it->second;

    std::shared_ptr<cameras::IntrinsicBase> intr = data.intrinsics.find(v->id_intrinsic)->second;
    vector<double> intr_params = intr->getParams(); //focal length at 0
    double focal_pix = intr_params[0];
    double plane_width = intr->w() / focal_pix;
    double plane_height = intr->h() / focal_pix;
    float width = intr->w();
    float height = intr->h();


    //project 3D points to 2D and save
    if (it_3d != view_3D.end())
    {
        vector<Eigen::Vector3d> &m3d = it_3d->second;

        ofstream sfm_projections;
        sfm_projections.open(sfm_projections_filename);
        sfm_projections << width << ", " << height << std::endl;
        vector<osg::Vec3d> proj_obs;
        for (size_t i = 0; i < m3d.size(); ++i)
        {
            //Eigen::Vector3d &p3d = m3d[randvec[i]];
            Eigen::Vector3d &p3d = m3d[i];
            //normalize absolute 2d point to world coords
            Vector3d p2d = pose(p3d);
            p2d /= p2d.z();
            p2d.x() += plane_width / 2.0;
            p2d.y() += plane_height / 2.0;
            p2d *= focal_pix;

            sfm_projections << p2d.x() << ", " << p2d.y() << std::endl;
        }
        sfm_projections.close();
    }

    //save 2D observations
    if (it_2d != view_2D.end())
    {
        vector<Eigen::Vector2d> &m2d = it_2d->second;

        //vector<osg::Vec3d> proj_obs;
        ofstream sfm_observations;
        sfm_observations.open(sfm_observations_filename);
        sfm_observations << intr->w() << ", " << intr->h() << std::endl;
        for (size_t i = 0; i < m2d.size(); ++i)
        {
            //Eigen::Vector3d &p3d = m3d[randvec[i]];
            Eigen::Vector2d &p2d = m2d[i];
            sfm_observations << p2d.x() << ", " << p2d.y() << std::endl;
        }
        sfm_observations.close();
    }
    else
    {
        std::cerr << "view for view ID: " << viewID << " NOT FOUND!" << std::endl;
    }

    //build KD tree for fast search
    Eigen::MatrixXd cloud(points.size(), 3);
    int idx = 0;
    std::vector<osg::Vec3d>::const_iterator it;
    for (it = points.cbegin(); it != points.cend(); ++it)
    {
        Eigen::Vector3d point = Eigen::Vector3d(it->x(), it->y(), it->z());
        cloud.row(idx) = point;
        ++idx;
    }

    typedef KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree_t;
    kd_tree_t kd_tree(cloud, 10); //10 max leaf size
    kd_tree.index->buildIndex();

    ofstream terrain_projections;
    terrain_projections.open(terrain_projections_filename);
    terrain_projections << width << ", " << height << std::endl;

    Hash_Map<IndexT, Eigen::Vector3d>::const_iterator cid;
    for (cid = cld.cbegin(); cid != cld.cend(); ++cid)
    {
        long out_idx[1];
        double out_dist[1];
        Eigen::Vector3d curr_pt = cid->second;
        kd_tree.index->knnSearch(&curr_pt[0], 1, out_idx, out_dist);

        Vector3d p2dn(0, 0, 0);
        if (cloud.rows() > 0)
        {
            Eigen::Vector3d nearest_point = cloud.row(out_idx[0]);
            p2dn = pose(nearest_point);
            p2dn /= p2dn.z();
            p2dn.x() += plane_width / 2.0;
            p2dn.y() += plane_height / 2.0;
            p2dn *= focal_pix;
        }

        Vector3d p2dc = pose(curr_pt);
        p2dc /= p2dc.z();
        p2dc.x() += plane_width / 2.0;
        p2dc.y() += plane_height / 2.0;
        p2dc *= focal_pix;

        terrain_projections << p2dc.x() << ", " << p2dc.y() << ", " << p2dn.x() << ", " << p2dn.y() << std::endl;
    }
    terrain_projections.close();

    return true;
}

void SfMLoader::printPoseNetPoses(string output_path)
{
    std::vector<Eigen::Vector3d> cld;
    Hash_Map<IndexT, shared_ptr<AbstractPhotoView>>::const_iterator it;
    for (it = photoviews.begin(); it != photoviews.end(); ++it)
    {
        osg::Vec3d p = it->second->getECEFPosition();
        Eigen::Vector3d pos = Eigen::Vector3d(p.x(), p.y(), p.z());
        cld.push_back(pos);
    }
    Vector3d centroid;
    BBox3D box = findPointCloudBBox(cld, centroid);
    std::cout << "bbox size: " << box.max - box.min << std::endl;
    if (cld.size() > 0)
    {
        fstream f(output_path, f.out | f.app);
        f << "Visual Landmark Dataset V1 generated by itr";
        f << std::endl;
        f << "ImageFile, Camera Position [X Y Z W P Q R]";
        f << std::endl;
        f << std::endl;

        for (it = photoviews.begin(); it != photoviews.end(); ++it)
        {
            osg::Vec3d p = it->second->getECEFPosition();
            Eigen::Vector3d pos = Eigen::Vector3d(p.x(), p.y(), p.z()) - centroid;
            osg::Quat quat = it->second->orientation();
            string photoname = fs::path(it->second->getPhotoName()).filename().string();
            f << photoname;
            f << " ";
            f << pos.x();
            f << " ";
            f << pos.y();
            f << " ";
            f << pos.z();
            f << " ";
            f << quat.w();
            f << " ";
            f << quat.x();
            f << " ";
            f << quat.y();
            f << " ";
            f << quat.z();
            f << std::endl;
        }
        f.close();
    }
}

int SfMLoader::numberOfSharedElements(vector<IndexT> tracks, vector<IndexT> curr_tracks, int upper_bound)
{
    std::sort(tracks.begin(), tracks.end());
    std::sort(curr_tracks.begin(), curr_tracks.end());

    vector<IndexT> *ref_smaller = &tracks;
    vector<IndexT> *ref_larger = &curr_tracks;
    if (tracks.size() > curr_tracks.size())
    {
        ref_smaller = &curr_tracks;
        ref_larger = &tracks;
    }

    size_t i_l = 0;
    size_t i_s = 0;
    int num_shared = 0;
    while (i_s < ref_smaller->size())
    {
        if (num_shared >= upper_bound)
        {
            //at least given number of 3D points is shared
            //no need to continue
            return num_shared;
        }
        if (i_l >= ref_larger->size() || i_s >= ref_smaller->size())
        {
            //if we are out of any of the two arrays, stop
            return num_shared;
        }
        if ((*ref_smaller)[i_s] == (*ref_larger)[i_l])
        {
            ++num_shared;
            ++i_s;
            ++i_l;
        }
        else if ((*ref_smaller)[i_s] < (*ref_larger)[i_l])
        {
            ++i_s;
        }
        else
        {
            //ref_smaller[i_s] is larger than ref_larger[i_l]
            ++i_l;
        }
    }
    return num_shared;
}

std::vector< std::tuple<openMVG::IndexT, std::string, int> > SfMLoader::getViewsSharing3DPointsWithViewId(openMVG::IndexT curr_viewID)
{
    init2D3D();
    std::vector< std::tuple<openMVG::IndexT, string, int>> res;

    sfm::Views::iterator it;
    for (it = data.views.begin(); it != data.views.end(); ++it)
    {
        std::shared_ptr<sfm::View> v = it->second;
        if (v->id_view != curr_viewID)
        {
            //check the number of shared 3D points
            VLM::iterator vlmit = viewLandmarksMap.find(v->id_view);
            VLM::iterator curr_vlmit = viewLandmarksMap.find(curr_viewID);
            if (vlmit != viewLandmarksMap.end() && curr_vlmit != viewLandmarksMap.end())
            {
                int num_shared = numberOfSharedElements(vlmit->second, curr_vlmit->second);
                string photo_base = "";
                shared_ptr<AbstractPhotoView> pv = photoviews_views[v->id_view];
                if (pv)
                {
                    photo_base = fs::path(pv->getPhotoName()).stem().string();
                }
                res.push_back(std::make_tuple(v->id_view, photo_base, num_shared));
            }
        }
    }
    std::sort(res.begin(), res.end(),
              [](std::tuple<openMVG::IndexT, std::string, int> &a,
                 std::tuple<openMVG::IndexT, std::string, int> &b)
                {return std::get<2>(a) > std::get<2>(b);});
    return res;

}


sfm::Pose3 SfMLoader::getPose(openMVG::IndexT viewID) const
{
    sfm::Views::const_iterator vit = data.views.find(viewID);

    if (vit != data.views.end())
    {
        sfm::Poses::const_iterator pit = data.poses.find(vit->second->id_pose);
        return pit->second;
    }
    sfm::Pose3 id;
    return id;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SfMLoader::calculateCentroidAndExtentForView3DPoints(IndexT viewID)
{
    init2D3D();

    Hash_Map<IndexT, vector<Eigen::Vector3d> >::iterator it_3d = view_3D.find(viewID);

    Eigen::Vector3d centroid = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d extent = Eigen::Vector3d(1.0, 1.0, 1.0);
    if (it_3d != view_3D.end())
    {
        vector<Eigen::Vector3d> &m3d = it_3d->second;
        BBox3D bbox = findPointCloudBBox(m3d, centroid);
        extent = bbox.max - bbox.min;
    }
    return make_pair(centroid, extent);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SfMLoader::calculateSceneCentroidAndExtent()
{
    Eigen::Vector3d centroid = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d extent = Eigen::Vector3d(1.0, 1.0, 1.0);

    vector<Eigen::Vector3d> m3d = createCloudFromData();
    BBox3D bbox = findPointCloudBBox(m3d, centroid);
    extent = bbox.max - bbox.min;
    return make_pair(centroid, extent);
}
