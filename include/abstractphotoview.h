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



#ifndef ABSTRACTPHOTOVIEW_H
#define ABSTRACTPHOTOVIEW_H

// stl headers
#include <string>
#include <cstdlib>
#include <stdexcept>
#include <cmath>
#include <ctime>
//#include <time.h>
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

// osgEarth headers
#include "osgEarth/Registry"
#include "osgEarth/Terrain"
#include "osgEarth/MapNode"
#include "osgEarth/GeoTransform"
#include "osgEarth/VirtualProgram"
#include "osgEarth/TerrainEngineNode"

namespace itr{


using namespace osg;
using namespace osgEarth;
using namespace std;

class AbstractPhotoView{

public:

    enum ViewState
    {
        SHOW_ALL = 0,
        SHOW_EXIF_POS = 1,
        SHOW_SFM_POS = 2,
        SHOW_SFM_POS_AND_DIR = 3,
        SHOW_NONE = 4,
        VIEW_STATE_LAST = 5
    };

    virtual Geode * getGeode() = 0;

    /**
     * @brief setOrientation
     * Sets orientation of the photo view using three angles.
     * @param yaw
     * @param pitch
     * @param roll
     */
    virtual void setOrientation(double yaw, double pitch, double roll) = 0;


    virtual void setOrientation(Quat orientation) = 0;

    /**
     * @brief setECEFPosition
     * Sets position of the photo view using ECEF coordinate system.
     * (https://cs.wikipedia.org/wiki/ECEF)
     * @param x
     * @param y
     * @param z
     */
    virtual void setECEFPosition(double x, double y, double z) = 0;

    /**
     * @brief setWGS84Position
     * Sets position of the photo view using WGS84 (GPS) position
     * 1.8 meters above the terrain.
     * @param latitude
     * @param longitude
     */
    virtual void setWGS84Position(double latitude, double longitude) = 0;

    /**
     * @brief setViewState
     * Sets desired view state and updates the graph.
     * @param state
     */
    virtual void setViewState(AbstractPhotoView::ViewState state) = 0;

    /**
     * @brief toggleViewState
     * Moves to the next view state and updates the graph.
     */
    virtual void toggleViewState() = 0;

    /**
     * @brief getTransf
     * @return transformation node that transforms the whole
     * photoview (image plane, position pointers, and other stuff).
     */
    virtual ref_ptr<PositionAttitudeTransform> getTransf() = 0;

    virtual ref_ptr<GeoTransform> getGeoTransf() = 0;


    virtual time_t getCreateDate() = 0;

    virtual osg::Vec3d getECEFPosition() = 0;

    /**
     * @brief getCenterInWorldCoords
     * Calculates center of the photoview in world coordinates
     * @return center of the photoview in world coordinates.
     */
    virtual osg::Vec3d getCenterInWorldCoords() = 0;

    /**
     * @brief getECEFPositionElevated
     * Returns position of this photoview elevated by some height.
     * @param height in meters to elevate the original position.
     * @return elevated position of the photoview by height.
     */
    virtual osg::Vec3d getECEFPositionElevated(double height) = 0;

    /**
     * @brief orientation
     * @return quaternion defining the orientation of this photoview.
     */
    virtual osg::Quat orientation() = 0;

    virtual osg::Quat getOrientationLocal() = 0;

    /**
     * @brief getAspect
     * @return aspect ratio defined as width / height of the photo
     */
    virtual double getAspect() = 0;

    /**
     * @brief getFOVDeg
     * @return field of view in degrees
     */
    virtual double getFOVDeg() = 0;

    /**
     * @brief getFOVYDeg
     * @return field of view in degrees
     */
    virtual double getFOVYDeg() = 0;

    /**
     * @brief getFOV
     * @return  field of view in radians
     */
    virtual double getFOV() = 0;

    /**
     * @brief setFOV
     * Sets horizontal field-of-view of this photoview and
     * automatically updates the transformation.
     * @param fov horizontal field of view in radians
     */
    virtual void setFOV(double fov) = 0;

    /**
     * @brief isPortrait
     * @return true if height is larger than width
     */
    virtual bool isPortrait() = 0;


    /**
     * @brief setOpacity
     * Sets the opacity of the photo plane.
     * @param opacity 0 transparent, 1 opaque.
     */
    virtual void setOpacity(float opacity) = 0;


    virtual void fitPositionToTerrain() = 0;

    virtual bool isAboveGround() = 0;

    virtual float distanceAboveGround() = 0;

    virtual void hide() = 0;

    virtual string getPhotoName() const = 0;

    /**
     * @brief remove
     * Removes the photoview from scene graph.
     */
    virtual void remove() = 0;

    virtual bool operator==(const AbstractPhotoView &rhs) const = 0;

    virtual bool operator==(const string &rhs) const = 0;

    /**
     * @brief hasGeoExif
     * @return True iff the photoview has valid GPS location loaded from EXIF
     * and getGeoExif is usable.
     */
    virtual bool hasGeoExif() = 0;

    /**
     * @brief getGeoPosition
     * Returns geo position loaded from exif in format:
     * Vec3(longitude, latitude, altitude). Latitude, longitude are values
     * loaded from EXIF, altitude is calculated based on elevation model at
     * the position.
     * @return  geo position loaded from exif (altitude is calculated based on
     * the elevation model) in ECEF coordinate system.
     */
    virtual Vec3d getGeoExif() = 0;

};

}
#endif // ABSTRACTPHOTOVIEW_H
