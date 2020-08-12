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

#ifndef SFMLOADER_H
#define SFMLOADER_H

//OpenMVG
#include "openMVG/types.hpp"
#include "openMVG/numeric/numeric.h"
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/multiview/projection.hpp"
#include "openMVG/multiview/solver_resection_kernel.hpp"

#include "openMVG/geometry/Similarity3.hpp"
#include "openMVG/geometry/Similarity3_Kernel.hpp"
#include "openMVG/robust_estimation/robust_estimator_LMeds.hpp"
#include "openMVG/geometry/rigid_transformation3D_srt.hpp"
#include "openMVG/multiview/projection.hpp"

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/sfm_data_filters_frustum.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_transform.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"

#include "openMVG/sfm/sfm_filters.hpp"

#include <openMVG/geometry/pose3.hpp>
#include <openMVG/types.hpp>


//local headers
#include "util.h"
#include "photoview.h"
#include "bbox3d.h"
#include "gridsampler.h"
#include "abstractphotoview.h"
#include "panophotoview.h"

//STL headers
#include <string>
#include <vector>
#include <stdexcept>
#include <set>
#include <memory>
#include <limits>
#include <cmath>
#include <cstdlib> //std::size_t
#include <queue>
#include <cassert>
#include <algorithm>
#include <random>
#include <utility>
#include <iterator>
#include <iostream>
#include <tuple>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <tuple>


//OSG Earth headers
#include <osgEarth/MapNode>
#include <osgEarth/ECEF>
#include <osgEarth/ElevationQuery>
#include <osgEarth/GeoData>
#include <osgEarth/GeoMath>


//OpenSceneGraph
#include <osg/Point>

//Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>

//Boost
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/regex/pattern_except.hpp>

//Libpointmatcher
#include "pointmatcher/PointMatcher.h"

//Nanoflann header only library
#include "nanoflann.hpp"

using namespace std;
using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;

using namespace osgEarth;
namespace fs = boost::filesystem;

namespace itr{

class SfMLoader
{
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum ViewState
    {
        SHOW_ALL = 0,
        SHOW_ORIG = 1,
        SHOW_ALIGNED_ICP = 2,
        SHOW_SAMPLED = 3,
        SHOW_NONE = 4,
        VIEW_STATE_LAST = 5
    };

    /**
     * @brief SfmLoader
     * Loads Structure from motion data from component directory.
     * @param component_dir Component directory contains images and
     * sfm_data.bin containing computed SfM information.
     */
    SfMLoader(string component_dir, ref_ptr<MapNode> mapNode,
              double center_lat = 360, double center_lon = 360);

    /**
     * @brief loadCameras
     * Loads photo views from sfm data and inserts them into the map.
     * @param mapNode osgEarth map node to insert the loaded photoparams into.
     */
    set< shared_ptr<AbstractPhotoView> > loadPhotoparams(MapNode *mapNode);


    osg::Node *loadPointCloud(MapNode *mapNode);

    /**
     * @brief addPointCloud
     * Adds arbitrary point cloud into the scene.
     * @param points the points of the point cloud
     * @param color color of the points, default green.
     * @return new point cloud node
     */
    osg::Node *addPointCloud(MapNode *mapNode, vector<osg::Vec3d> points,
                             osg::Vec4 color = osg::Vec4(0, 1, 0, 1),
                             float elevated = 0);


    /**
     * @brief getViewsCount
     * @return number of views in loaded sfm data.
     */
    int getViewsCount();

    static BBox3D findPointCloudBBox(const vector<Eigen::Vector3d> &cluster,
                                         Vector3d &centroid);

    void alignPointCloudParts(MapNode *mapNode);

    /**
     * @brief alignPointCloudICP
     * Convenient method to run ICP directly, without the need to sample
     * terrain manually.
     * @param mapNode mapNode to get the underlying terrain from.
     */
    void alignPointCloudICP(MapNode *mapNode);

    /**
     * @brief saveSfmData
     * Saves SfMData in OpenMVG format in current state.
     * Usable when ICP is calculated so that we don't need to recalculate it
     * over and over.
     */
    void saveSfmData(string filename);

    /**
     * @brief alignGPS
     * Calculates robust alignment of the sfm scene with GPS priors loaded from
     * EXIF.
     */
    void alignGPS();

    /**
     * @brief fitPhotoViewsToTerrain
     * Fits the photoview positions to the terrain.
     */
    void fitPhotoViewsToTerrain();

    void adjustRotationScale(MapNode *mapNode);

    void toggleViewState(MapNode *mapNode);

    void updateViewState(MapNode *mapNode);

    vector<Eigen::Vector3d> createCloudFromData();

    openMVG::Hash_Map<openMVG::IndexT, Eigen::Vector3d> createCloudMapFromData();

    vector<Eigen::Vector3d> createCloud(const openMVG::Hash_Map<openMVG::IndexT, Eigen::Vector3d> &_cloud);

    /**
     * @brief createCloudMapFromDataForViewId
     * Creates cloud map which corresponds to the view
     * @param viewId
     * @return
     */
    openMVG::Hash_Map<IndexT, Eigen::Vector3d> createCloudMapFromDataForViewId(openMVG::IndexT viewId);

    string getComponentDir();

    /**
     * @brief getPhotoviewKeys
     * Returns the keys of photoviews. First the photoviews are clustered based
     * on their position, so that the keys are grouped by cluster. This way, when
     * iterating the output keys, the photoviews which keys are close in the vector
     * should be also close spatially. How close the photoviews are in the cluster is
     * driven by search_radius_sq parameter.
     * @param search_radius_sq radius squared (in meters) is used to search nearest neighbors to the
     * current pivot point. If a photoview which should be included in the cluster
     * shall be at most 1km distant from any otehr photoview in the cluster,
     * set the search_radius = 1km * 1km = 1000m * 1000m = 1000000m (default).
     * @return vector of sfm::ViewId indexing the photoviews stored
     * by this SfMLoader. The calues can be used to iterate photoviews_views
     * using getPhotoview() method.
     */
    vector<openMVG::IndexT> getPhotoviewKeys(double search_radius_sq = 1000000);

    /**
     * @brief getPhotoView
     * @param viewId sfm::ViewId index of the photoview within this
     * SfMLoader (stored in photoview_view)
     * @return AbstractPhotoView at view)d.
     */
    shared_ptr<AbstractPhotoView> getPhotoView(openMVG::IndexT viewId);

    /**
     * @brief getPhotoViewMeanError
     * @param idx sfm::ViewId index of the photoview within this
     * SfMLoader
     * @param cld   sampled points from the terrain model
     *              near the SfM 3D points corresponding to the viewID
     *              (may be created by calling createCloudMapFromDataForViewId()).
     * @param points    the points sampled from the terrain near the 3D SfM points from cld
     *                  (may be created by calling sampleTerrain(cld, mapNode).
     * @return Tuple of:
     * - number of poitns in the point cloud corresponding to the photoview
     * - mean distance between the terrain and 3D points corresponding
     * to this photoview.
     * - median distance between the terrain and 3D points corresponding
     * to this photoview.
     */
    std::tuple<int, double, double> getPhotoViewError(openMVG::IndexT idx,
                                                      const Hash_Map<IndexT, Eigen::Vector3d> &cld,
                                                      const vector<osg::Vec3d> &points);

    /**
     * @brief calculate3DStats
     * Calculates distance statistics between the 3D points passed in cld and
     * the sampled terrain. The for each input 3D point the closest sampled
     * terrain point is used.
     * @param cld   sampled points from the terrain model
     *              near the SfM 3D points corresponding to the viewID
     *              (may be created by calling createCloudMapFromDataForViewId()).
     * @param points    the points sampled from the terrain near the 3D SfM points from cld
     *                  (may be created by calling sampleTerrain(cld, mapNode).
     * @return
     * Tuple of:
     * - number of poitns in the point cloud corresponding to the photoview
     * - mean distance between the terrain and 3D points
     * - median distance between the terrain and 3D points
     * - vector of distances between the terrain and 3D points
     * (to calculate median for larger datasets consolidated from more scenes).
     */
    std::tuple<int, double, double, vector<double> > calculate3DStats(
            const Hash_Map<IndexT, Eigen::Vector3d> &cld,
            const vector<osg::Vec3d> &points);

    /**
     * @brief calculateMean3DDistance
     * Calculates distance statistics of all 3D points in this scene
     * to the nearest point on the terrain.
     * @return
     * Tuple of:
     * - number of poitns in the point cloud corresponding to the photoview
     * - mean distance between the terrain and 3D points
     * - median distance between the terrain and 3D points
     * - vector of distances between the terrain and 3D points
     * (to calculate median for larger datasets consolidated from more scenes).
     */
    std::tuple<int, double, double, vector<double> > calculate3DStatsForData();


    /**
     * @brief pointCount
     * @return Number of 3D points for this scene.
     */
    int pointCount();

    /**
     * @brief reprojectionSquaredNorm
     * @return Calculates squared norm of the reprojection for all views
     * in this scene.
     */
    vector<double> reprojectionRMSE();

    vector<double> reprojectionRMSE(IndexT viewID);

    /**
     * @brief savePhotoview3DProjections
     * - (1) Saves 3D points projected into the image coordinates into a file.
     * @viewID is id of the respective view to show observations for.
     * - (2) Saves 2D observations (keypoints)
     * - (3) TODO: For each 3D point find closest point on the terrain, project it
     * to image coordinates and save them to a file.
     *
     * @param viewID id of the view as stored in SfM_Data structure
     * @param cld   sampled points from the terrain model
     *              near the SfM 3D points corresponding to the viewID
     *              (may be created by calling createCloudMapFromDataForViewId()).
     * @param points    the points sampled from the terrain near the 3D SfM points from cld
     *                  (may be created by calling sampleTerrain(cld, mapNode).
     * @param sfm_projections_filename output filename of the projected 3D SfM points (1)
     * @param sfm_observations_filename output filename of the 2D observations (keypoints) (2)
     * @param terrain_projections_filename output filename of the projected closest point on terrain (3)
     * @return true when the view has corresponding 3D points, false otherwise.
     */
    bool savePhotoview3DProjections(openMVG::IndexT viewID,
                                    const Hash_Map<IndexT, Eigen::Vector3d> &cld,
                                    const vector<osg::Vec3d> &points,
                                    string sfm_projections_filename,
                                    string sfm_observations_filename,
                                    string terrain_projections_filename);

    /**
     * @brief getPhotoviewsSharing3DPoints
     * Finds photoviews which share 3D sfm points with given view.
     * @param viewID view defining the 3D points which shall be shared in returned views.
     * @return vector of tuples containing viewIds, photoview names, and number of
     * 3D sfm points shared with view defined with viewID.
     */
    std::vector< std::tuple<openMVG::IndexT, std::string, int> > getViewsSharing3DPointsWithViewId(openMVG::IndexT viewID);


    /**
     * @brief getPose
     * gets the pose for viewID.
     * @param viewID
     * @return if found, returns pose for given viewID,
     * identity pose otherwise (identity rotation, zero translation).
     */
    sfm::Pose3 getPose(openMVG::IndexT viewID) const;

    /**
     * @brief calculateCentroidForView3DPoints
     * Calculates centroid from 3D sfm points which are observed
     * from the view with given viewID.
     * @param viewID id of the view for which to calculate the centroid.
     * @return centroid of all 3D sfm points which are observed from the
     * view with givem viewID.
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> calculateCentroidAndExtentForView3DPoints(IndexT viewID);

    /**
     * @brief calculateCentroidForView3DPoints
     * Calculates centroid and extent for the whole scene.
     * @return centroid and extent of all 3D sfm points in the scene.
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> calculateSceneCentroidAndExtent();


    /**
     * @brief sampleTerrain
     * Samples the terrain into a vector of points.
     * @param mapNode
     * @return vector of sampled terrain points
     */
     vector<osg::Vec3d> sampleTerrain(const openMVG::Hash_Map<openMVG::IndexT, Eigen::Vector3d> &cloud);

     /**
      * @brief SfMLoader::printPoseNetPoses
      */
     void printPoseNetPoses(string output_path);

private:

    ref_ptr<MapNode> mapNode;

    double center_lat, center_lon;

    /// SfM data structure from OpenMVG
    SfM_Data data;

    /// parent directory where the images and sfm data are located
    string data_dir;

    /// Estimated SfM point cloud to terrain model alignment transformation
    Eigen::MatrixXd aligned_T;

    /// Collection of photoviews (indexed by sfm::View::id_pose)
    typedef openMVG::Hash_Map<openMVG::IndexT,
                              shared_ptr<AbstractPhotoView> > PhotoviewMap;
    PhotoviewMap photoviews;

    /// Collection of photoviews (indexed by sfm::ViewId)
    typedef openMVG::Hash_Map<openMVG::IndexT,
                              shared_ptr<AbstractPhotoView> > PhotoviewMapView;
    PhotoviewMapView photoviews_views;

    SfMLoader::ViewState view_state;


    ref_ptr<osg::Node> orig_pointcloud;
    ref_ptr<osg::Node> icp_aligned_pointcloud;
    ref_ptr<osg::Node> sampled_pointcloud;

    //3D points indexed by sfm::view_id
    Hash_Map<IndexT, vector<Eigen::Vector3d> > view_3D;
    //3D points indexed by sfm::view_id containing
    //hash map indexing the 3D point with trackID
    Hash_Map<IndexT, Hash_Map<IndexT, Eigen::Vector3d> > view_3D_track;
    //2D points indexed by sfm::view_id
    Hash_Map<IndexT, vector<Eigen::Vector2d> > view_2D;
    //trackIds indexed by sfm::view_id
    using VLM = Hash_Map<IndexT, vector<IndexT> >;
    VLM viewLandmarksMap;
    bool loaded2D3D;

    /**
      Initializes 2D and 3D points indexed by viewId (view_3D, view_2D)
     * @brief init3D2D
     */
    void init2D3D();

    /**
     * @brief findSfMData
     * Recursively searches for sfm_data.bin file in component dir.
     * @param component_dir component dir containing images and reconstructed
     * sfm information stored in sfm_data.bin.
     * @return path to the sfm_data including the filename.
     */
    static string findSfMData(const string &component_dir);

    /**
     * @brief findSfMDataICP
     * Recursively searches for sfm_data_icp.bin file in component dir.
     * These are data aligned with ICP method.
     * @param component_dir component dir containing images and reconstructed
     * sfm information stored in sfm_data.bin.
     * @return path to the sfm_data including the filename.
     */
    static string findSfMDataICP(const string &component_dir);

    /**
     * @brief findNVMData
     * Recursively searches for *.nvm files in component dir.
     * @param component_dir dir containing images and reconstructed
     * sfm information stored in nvm format.
     * (see http://ccwu.me/vsfm/doc.html#nvm)
     * @return path to the first nvm file found.
     */
    static string findNVMData(const string &component_dir);

    static void rotationAngles(Eigen::Matrix3d m, double &yaw,
                        double &pitch, double &roll);

    /**
     * @brief alignPointCloudICP
     * Aligns the point cloud with the underlying terrain.
     * Needs to be called prior loadPointCloud, which loads the data
     * from SfM_Data into the scene. Shall be called directly in the
     * beginning of loadPointCloud. The found transformation is stored in
     * alignM matrix.
     * @param mapNode mapNode to get the underlying terrain from.
     * @param points sampled terrain points used for alignment
     */
    void alignPointCloudICP(MapNode *mapNode, vector<osg::Vec3d> points);


    /**
     * @brief runICP
     * Runs ICP algorithm to align points to the point cloud from SfM_Data.
     * @param points reference point cloud into which the point cloud from
     * SfM_data will be aligned.
     * @param cloud point cloud to be aligned (from SfM_data).
     * @return (4x4) transformation matrix defining the point cloud
     * transformation.
     */
    Eigen::MatrixXd runICP(vector<osg::Vec3d> points,
                           vector<Eigen::Vector3d> cloud,
                           vector<osg::Vec3d> &aligned_points);

    /**
     * @brief findPointCloudBBox
     * Calculates bounding box of the point cloud structure inside SfM_Data.
     * @param cluster stores indices to points in cluster for which calculate
     * the bounding box.
     * @param centroid reference into which the centroid of the
     * point is stored.
     * @return bounding box of the point cloud stored in SfM_Data.
     */
     BBox3D findPointCloudBBox(const vector<openMVG::IndexT> &cluster,
                               Vector3d &centroid);



     /**
      * @brief createBBoxPhotoPositions
      * Creates bounding box around photo positions.
      * @return bounding box around photo positions.
      */
     BBox3D createBBoxPhotoPositions();

     /**
      * @brief applyAlignmentToSfMData
      * Applies transformation matrix obtained from ICP to structure
      * stored in SfM_data.
      */
     void applyAlignmentToSfMDataStructure();

     /**
      * @brief updatePhotoviewsFromSfMData
      * Updates photoviews position, orientation, and FOV according to
      * SfM data.
      */
     void updatePhotoviewsFromSfMData();


     void filterOutliers();

     /**
      * @brief findEuclideanClusters
      * Finds clusters of 3D points in euclidean space. Points to be
      * processed are loaded from SfM_Data.
      * @return vector of clusters. Each cluster contains indices to
      * 3D points stored in SfM_Data.structure.
      */
     vector< vector<openMVG::IndexT> > findEuclideanClusters(const openMVG::Hash_Map<openMVG::IndexT, Eigen::Vector3d> &_cloud);

     /**
      * @brief findEuclideanClusters
      * Finds clusters of photoviews positions in euclidean space.
      * @param search_radius_sq radius squared (in meters) is used to search nearest neighbors to the
      * current pivot point. If a photoview which should be included in the cluster
      * shall be at most 1km distant from any photoview in the cluster,
      * the search_radius = 1km * 1km = 1000m * 1000m = 1000000m (default).
      * @return vector of clusters. Each cluster contains indices to
      * 3D points stored in SfM_Data.structure.
      */
     vector< vector<openMVG::IndexT> > findEuclideanClustersPhotoviews(double search_radius_sq = 1000000);

     /**
      * @brief KabschAlgorithm
      * https://en.wikipedia.org/wiki/Kabsch_algorithm
      * Finds rotation of P to Q.
      * @param P first point cloud for which rotation should be estimated
      * @param Q reference point cloud
      * @return rotation matrix which rotates P to Q in the sense
      * of minimal error.
      */
     Eigen::Matrix3d KabschAlgorithm(const Eigen::MatrixXd &P,
                                     const Eigen::MatrixXd &Q,
                                     double &out_scale);

     /**
      * @brief random_vector
      * Generates vector of k random numbers without repetition in the range
      * from 0 to n.
      * @param n the maximum number
      * @param k number of elements to return, must be lower or equal to n.
      * @return random vector of k numbers from 0 to n without repetition.
      */
     vector<int> random_vector(int n, int k);

     /**
      * @brief isPano
      * Decides whether the image is part of 360 panorama, or not.
      * @param imageName name of the image.
      * @return true when the image is a part of 360 degree panorama.
      */
     static bool isPano(const string &imageName);

     /**
      * @brief getPanoFilePath
      * Generates the panorama file path given an image name which defines
      * image reprojected to rectilinear from original equirectangular pano.
      * @param imageName name of the part of the pano (represents image
      * sliced from pano and reprojected to rectilinear).
      * @return panorama file path.
      */
     string getPanoFilePath(const string &imageName);

     /**
      * @brief getPanoYaw
      * Extracts yaw angle from image name of the reprojected rectilinear
      * image from original equirectangular pano.
      * @param imageName name of the part of the original equirectangular pano
      * (reprojected to rectilinear).
      * @return yaw angle extracted from the image name.
      */
     int getPanoYaw(const string &imageName);


     /**
      * @brief findNearestPhotoviewWithExifGPS
      * Finds nearest photoview based on exif GPS.
      * @param pos  query position in ECEF coordinate system.
      * @return pair<distance, photoview> having nearest exif GPS position
      * to the query position.
      */
     std::pair<double, std::shared_ptr<AbstractPhotoView>> findNearestPhotoviewWithExifGPS(osg::Vec3d pos);


     /**
      * @brief numberOfSharedElements
      * Calculates number of same elements in both arrays
      * @param first the first array
      * @param second the second array
      * @param upper_bound the search will be ended if there is
      * more than upper_bound of shared elements in both arrays.
      * Default value: INT_MAX.
      * @return number of same (shared) elements in both arrays.
      * If the number of shared elements is larger than upper_bound,
      * upper_bound is returned and the computation is terminated earlier
      * to save time.
      */
     static int numberOfSharedElements(vector<IndexT> tracks, vector<IndexT> curr_tracks, const int upper_bound = INT_MAX);
};

}

#endif // SFMLOADER_H
