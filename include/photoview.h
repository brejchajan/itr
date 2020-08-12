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



#ifndef PHOTOVIEW_H
#define PHOTOVIEW_H

#include "abstractphotoview.h"

#include <boost/filesystem.hpp>

#include "exif.h"
#include <time.h>

using namespace osg;
using namespace osgEarth;
using namespace std;
using namespace openMVG::exif;

namespace fs = boost::filesystem;

namespace itr{

class PhotoView : public AbstractPhotoView
{
public:

    static constexpr double PHOTOPLANE_OFFSET = 50.0;
    const int IMAGE_WIDTH=128;
    const int IMAGE_WIDTH_LARGE=2048;

    /**
     * @brief PhotoView
     * Constructor for usage with osgearth
     * @param mapNode
     * @param photoName
     */
    PhotoView(MapNode *mapNode, string photoName);


    Geode * getGeode();

    /**
     * @brief setOrientation
     * Sets orientation of the photo view using three angles.
     * @param yaw
     * @param pitch
     * @param roll
     */
    void setOrientation(double yaw, double pitch, double roll);


    void setOrientation(Quat orientation);

    /**
     * @brief setECEFPosition
     * Sets position of the photo view using ECEF coordinate system.
     * (https://cs.wikipedia.org/wiki/ECEF)
     * @param x
     * @param y
     * @param z
     */
    void setECEFPosition(double x, double y, double z);

    /**
     * @brief setWGS84Position
     * Sets position of the photo view using WGS84 (GPS) position
     * 1.8 meters above the terrain.
     * @param latitude
     * @param longitude
     */
    void setWGS84Position(double latitude, double longitude);

    /**
     * @brief setViewState
     * Sets desired view state and updates the graph.
     * @param state
     */
    void setViewState(PhotoView::ViewState state);

    /**
     * @brief toggleViewState
     * Moves to the next view state and updates the graph.
     */
    void toggleViewState();

    //FIXME: not used?
    //void setMatrixTransform(const osg::Matrix &tr);

    /**
     * @brief getTransf
     * @return transformation node that transforms the whole
     * photoview (image plane, position pointers, and other stuff).
     */
    ref_ptr<PositionAttitudeTransform> getTransf();

    ref_ptr<GeoTransform> getGeoTransf();


    time_t getCreateDate();

    osg::Vec3d getECEFPosition();

    virtual osg::Quat getOrientationLocal();

    virtual osg::Quat getOrientation();

    /**
     * @brief getCenterInWorldCoords
     * Calculates center of the photoview in world coordinates
     * @return center of the photoview in world coordinates.
     */
    osg::Vec3d getCenterInWorldCoords();

    /**
     * @brief getECEFPositionElevated
     * Returns position of this photoview elevated by some height.
     * @param height in meters to elevate the original position.
     * @return elevated position of the photoview by height.
     */
    osg::Vec3d getECEFPositionElevated(double height);

    /**
     * @brief orientation
     * @return quaternion defining the orientation of this photoview.
     */
    osg::Quat orientation();

    /**
     * @brief getAspect
     * @return aspect ratio defined as width / height of the photo
     */
    double getAspect();

    /**
     * @brief getFOVDeg
     * @return field of view in Y direction in degrees
     */
    double getFOVYDeg();

    /**
     * @brief getFOVDeg
     * @return field of view in degrees
     */
    double getFOVDeg();

    /**
     * @brief getFOV
     * in radians
     * @return
     */
    double getFOV();

    int getImageWidth();

    int getImageHeight();

    /**
     * @brief isPortrait
     * @return true if height is larger than width
     */
    bool isPortrait();

    /**
     * @brief setFOV
     * Sets horizontal field-of-view of this photoview and
     * automatically updates the transformation.
     * @param fov horizontal field of view in radians
     */
    void setFOV(double fov);

    /**
     * @brief setFOVYDeg
     * Sets horizontal field-of-view of this photoview from vertical FOV
     * and automatically updates the transformation.
     * @param foyv vertical field of view in degrees.
     */
    void setFOVYDeg(double fovy);


    /**
     * @brief setOpacity
     * Sets the opacity of the photo plane.
     * @param opacity 0 transparent, 1 opaque.
     */
    void setOpacity(float opacity);


    virtual bool isAboveGround();

    virtual float distanceAboveGround();


    void fitPositionToTerrain();

    void hide()
    {
        setOpacity(0);
    }

    static Geode *createLine(Vec4 color, Vec3 start, Vec3 end);

    virtual bool operator==(const string &rhs) const
    {
        return photoName == rhs;
    }

    virtual bool operator==(const AbstractPhotoView &rhs) const
    {
        return photoName == rhs.getPhotoName();
    }

    /**
     * @brief remove
     * Removes the photoview from scene graph.
     */
    virtual void remove();

    virtual string getPhotoName() const;

    /**
     * @brief hasGeoExif
     * @return True iff the photoview has valid GPS location loaded from EXIF
     * and getGeoExif is usable.
     */
    virtual bool hasGeoExif();

    /**
     * @brief getGeoPosition
     * Returns geo position loaded from exif in format:
     * Vec3(longitude, latitude, altitude). Latitude, longitude are values
     * loaded from EXIF, altitude is calculated based on elevation model at
     * the position.
     * @return  geo position loaded from exif (altitude is calculated based on
     * the elevation model) in ECEF coordinate system.
     */
    virtual Vec3d getGeoExif();

    /**
     * @brief updatePhotoTexture
     * Updates the texture. Call after loadFullResolutionPhoto or
     * loadSmallResolutionPhoto.
     */
    void updatePhotoTexture(bool overlay = false);


    void loadSmallResolutionPhoto();
    void loadFullResolutionPhoto();



private:
    ref_ptr<MapNode> mapNode;

    /// reference to the osg plane node
    ref_ptr<Geode> photoplane;

    /// reference to the osg plane node for the masked part of photo
    ref_ptr<Geode> photoplane_masked;

    ref_ptr<Geode> photo_pointer;

    ref_ptr<Geode> photo_pointer_exif;

    /// line illustrating direction of the view (perpendicular to image
    /// plane).
    ref_ptr<Geode> perp_line;

    /// reference to the geometry of the plane
    ref_ptr<Geometry> geom;

    /// reference to the geometry of the plane for the masked part of photo
    ref_ptr<Geometry> geom_masked;

    /// reference to the actual vertices of the plane
    ref_ptr<Vec3Array> vert;

    /// reference to the transform osg node of this photo view
    ref_ptr<PositionAttitudeTransform> transf;

    ref_ptr<PositionAttitudeTransform> glob_transf;

    /// reference to the geo transform osg node in WGS84
    ref_ptr<GeoTransform> geo_transf;

    /// reference to the goe transform osg node which denotes the
    /// position loaded from exif.
    ref_ptr<GeoTransform> geo_transf_ex;

    /// name of the photo file
    string photoName;

    /// photo image
    ref_ptr<Image> photoImg;

    /// photo image mask
    ref_ptr<Image> photoMask;

    /// spatial reference which is set to WGS84 in the constructor
    const SpatialReference *srs;

    /// position of the photo in geo coordinates
    GeoPoint position;

    /// position of the photo given by exif in geo coordinates
    GeoPoint position_exif;

    /// horizontal field-of-view of the photo in radians
    double fov;

    double photo_yaw, photo_pitch, photo_roll;

    PhotoView::ViewState view_state;

    std::time_t createDate;

    /// True iff geolocation was found in exif and position_exif is usable.
    bool has_geo;

    osg::ref_ptr<ElevationPool> ep;
    osg::ref_ptr<ElevationEnvelope> ee;

    void loadPhoto(string photoName);

    /**
     * @brief createLocationPointer
     * Creates vertical line which shall be used for position
     * visualization. After it is created, it needs to be added to the
     * transform node defining its position.
     * @param red red channel of the line color
     * @param green green channel of the line color
     * @param blue blue channel of the line color
     * @return
     */
    Geode *createLocationPointer(float red, float green, float blue);

    void updateViewState();

    void loadPhotoMask(int w, int h);



};

struct PhotoViewEarlier
{
    inline bool operator() (std::shared_ptr<AbstractPhotoView> p1,
                            std::shared_ptr<AbstractPhotoView> p2) const
    {
        return (p1->getCreateDate() < p2->getCreateDate());
    }
};


} //NAMESPACE itr



#endif // PHOTOVIEW_H
