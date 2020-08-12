/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   13.09.2017
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



#ifndef PANOPHOTOVIEW_H
#define PANOPHOTOVIEW_H

// stl headers
#include <string>
#include <cstdlib>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include <sstream>
#include <memory>
#include <iomanip>

// OpenSceneGraph headers
#include "osg/Geode"
#include "osg/Geometry"
#include "osg/Group"
#include "osg/Array"
#include "osg/AlphaFunc"
#include "osg/Drawable"
#include "osg/PositionAttitudeTransform"
#include "osg/Image"
#include "osg/Texture2D"
#include "osg/Material"
#include "osg/ShapeDrawable"

// osgEarth headers
#include "osgEarth/Registry"
#include "osgEarth/Terrain"
#include "osgEarth/MapNode"
#include "osgEarth/GeoTransform"
#include "osgEarth/VirtualProgram"
#include "osgEarth/TerrainEngineNode"

// local headers
#include "abstractphotoview.h"
#include "exif.h"



namespace itr{

using namespace osg;
using namespace osgEarth;
using namespace std;


class PanoPhotoView : public AbstractPhotoView {

public:

    static constexpr double SPHERE_SIZE = 10.0;

    /**
     * @brief PanoPhotoView
     * Constructor of the PanoPhotoView
     * @param mapNode root node into which this PanoPhotoView will be appended.
     * @param photoName path of the equirectangular image file.
     * @param yaw initial rotation of the pano in degrees.
     */
    PanoPhotoView(MapNode *mapNode, const string &photoName, int yaw);

    virtual Geode * getGeode();

    /**
     * @brief setOrientation
     * Sets orientation of the photo view using three angles.
     * @param yaw
     * @param pitch
     * @param roll
     */
    virtual void setOrientation(double yaw, double pitch, double roll);


    virtual void setOrientation(Quat orientation);

    /**
     * @brief setECEFPosition
     * Sets position of the photo view using ECEF coordinate system.
     * (https://cs.wikipedia.org/wiki/ECEF)
     * @param x
     * @param y
     * @param z
     */
    virtual void setECEFPosition(double x, double y, double z);

    /**
     * @brief setWGS84Position
     * Sets position of the photo view using WGS84 (GPS) position
     * 1.8 meters above the terrain.
     * @param latitude
     * @param longitude
     */
    virtual void setWGS84Position(double latitude, double longitude);

    /**
     * @brief setViewState
     * Sets desired view state and updates the graph.
     * @param state
     */
    virtual void setViewState(AbstractPhotoView::ViewState state);

    /**
     * @brief toggleViewState
     * Moves to the next view state and updates the graph.
     */
    virtual void toggleViewState();

    /**
     * @brief getTransf
     * @return transformation node that transforms the whole
     * photoview (image plane, position pointers, and other stuff).
     */
    virtual ref_ptr<PositionAttitudeTransform> getTransf();

    virtual ref_ptr<GeoTransform> getGeoTransf();

    virtual time_t getCreateDate();

    virtual osg::Vec3d getECEFPosition();

    virtual osg::Quat getOrientationLocal();

    /**
     * @brief getCenterInWorldCoords
     * Calculates center of the photoview in world coordinates
     * @return center of the photoview in world coordinates.
     */
    virtual osg::Vec3d getCenterInWorldCoords();

    /**
     * @brief getECEFPositionElevated
     * Returns position of this photoview elevated by some height.
     * @param height in meters to elevate the original position.
     * @return elevated position of the photoview by height.
     */
    virtual osg::Vec3d getECEFPositionElevated(double height);

    /**
     * @brief orientation
     * @return quaternion defining the orientation of this photoview.
     */
    virtual osg::Quat orientation();

    /**
     * @brief getAspect
     * @return aspect ratio defined as width / height of the photo
     */
    virtual double getAspect();

    /**
     * @brief getFOVDeg
     * @return field of view in degrees
     */
    virtual double getFOVDeg();

    /**
     * @brief getFOVYDeg
     * @return field of view in degrees
     */
    virtual double getFOVYDeg();

    /**
     * @brief getFOV
     * @return field of view in radians
     */
    virtual double getFOV();

    /**
     * @brief setFOV
     * Sets horizontal field-of-view of this photoview and
     * automatically updates the transformation.
     * @param fov horizontal field of view in radians
     */
    virtual void setFOV(double fov);


    /**
     * @brief setOpacity
     * Sets the opacity of the photo plane.
     * @param opacity 0 transparent, 1 opaque.
     */
    virtual void setOpacity(float opacity);


    virtual void fitPositionToTerrain();

    virtual bool isAboveGround();

    virtual float distanceAboveGround();

    virtual void hide();

    virtual void remove();

    virtual string getPhotoName() const;

    /**
     * @brief operator ==
     * Compares whether two pano photo views are the same, eg., whether
     * they represent the same panorama image (based on name of the image,
     * not the path).
     * @param rhs
     * @return true if both panoramas represent the same panorama image.
     */
    virtual bool operator==(const AbstractPhotoView &rhs) const
    {
        return photoName == rhs.getPhotoName();
    }

    virtual bool operator==(const string &rhs) const
    {
        return photoName == rhs;
    }

    virtual osg::Quat getInitRot();

    virtual double getYaw();

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
     * @brief isPortrait
     * @return true if height is larger than width
     */
    bool isPortrait();


private:
    MapNode *mapNode;

    /// name of the photo file
    string photoName;

    /// sphere onto which is mapped the pano
    ref_ptr<Geode> sphere;

    /// geometry of the sphere
    ref_ptr<ShapeDrawable> geom;

    /// local transformation node - relative rotation and position
    /// to the global position
    ref_ptr<PositionAttitudeTransform> transf;

    /// global transformation node - holds global
    ref_ptr<PositionAttitudeTransform> glob_transf;

    /// photo image
    ref_ptr<Image> photoImg;

    /// spatial reference which is set to WGS84 in the constructor
    const SpatialReference *srs;

    /// initial yaw rotation in degrees.
    int yaw;

    Quat initRotQuat;

    //FIXME: initialize!
    std::time_t createDate;

    void loadPhoto(string photoName);

};

}
#endif // PANOPHOTOVIEW_H
