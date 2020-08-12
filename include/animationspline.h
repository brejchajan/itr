/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   31.08.2017
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



#ifndef ANIMATIONSPLINE_H
#define ANIMATIONSPLINE_H

// local headers
#include "sfmloader.h"
#include "photoview.h"
#include "controlpoint.h"
#include "bbox3d.h"

// STL headers
#include <vector>
#include <memory>
#include <limits>

// osgearth headers
#include "osgEarth/SpatialReference"
#include "osgEarth/Config"
#include "osgEarth/MapNode"
#include "osgEarth/TerrainEngineNode"

using namespace osg;

namespace itr{

class AnimationSpline{
public:
    AnimationSpline(vector< shared_ptr<AbstractPhotoView> > photoviews,
                    MapNode *mapNode,
                    ref_ptr<SpatialReference> srs);

    AnimationSpline(vector< shared_ptr<AbstractPhotoView> > picked_photoviews,
                    vector< shared_ptr<AbstractPhotoView> > all_photoviews,
                    MapNode *_mapNode,
                    ref_ptr<SpatialReference> _srs);


    /**
     * @brief remove
     * Removes animation spline geometry from the scene graph.
     */
    void remove();

    /**
     * @brief reset
     * Resets the animation spline to the beginning.
     */
    void reset();

    /**
     * @brief nextFrame
     * Advances the animation to the next frame
     * @return true if we hit the stop point, false otherwise
     */
    bool nextFrame();


    /**
     * @brief position
     * @return position of the camera on the spline
     */
    osg::Vec3d position();

    /**
     * @brief center
     * @return point at which the camera is looking at
     */
    osg::Vec3d center();

    /**
     * @brief up
     * @return up vector of the camera
     */
    osg::Vec3d up();

    /**
     * @brief finished
     * @return true if there the animation finished
     */
    bool finished();

    /**
     * @brief fovy
     * @return interpolated camera field of view
     */
    double fovy();

    /**
     * @brief aspect
     * @return interpolated aspect ratio of the camera
     */
    double aspect();

    /**
     * @brief setFovy
     * Sets field-of-view of the camera to interpolate from.
     * @param fov field-of-view of the camera in the y-direction according to
     * gluPerspective().
     */
    void setFovy(double fov);

    /**
     * @brief setAspect
     * Sets aspect of the camera to interpolate from.
     * @param aspect of the camera = width/height as defined in gluPerspective()
     */
    void setAspect(double aspect);

    /**
     * @brief slerp
     * Interpolation between two points on sphere.
     * @param p0
     * @param p1
     * @param t
     * @return
     */
    static osg::Vec3d slerp(osg::Vec3d p0, osg::Vec3d p1, double t);

    /**
     * @brief setFramerate
     * Sets current framerate so that the animation can compute correct
     * animation updates.
     * @param f
     */
    void setFramerate(double f);


    /**
     * @brief getControlPoint
     * Return current control point towards which the camera is moving.
     * @return
     */
    ControlPoint getControlPoint();


    /**
     * @brief distanceToNextControlPoint
     * Calculates the distance to the next control point from current position
     * @return
     */
    double distanceToNextControlPoint();

    double distanceToPreviousControlPoint();

    /**
     * @brief isPathShown
     * @return true when the animation path is shown
     */
    bool isPathShown();

    /**
     * @brief togglePathShown
     * Toggles the animation path on and off
     */
    void togglePathShown();



private:
    /// control points used for presentation - the points the user has picked
    vector<ControlPoint> control_points;

    /// control points used for calculating the spline, might be richer than
    /// user-selected control_points, eg., might be all the user-generated
    /// photographs
    vector<ControlPoint> all_control_points;

    /// points of the spline used for the animation
    vector<osg::Vec3d> animation_spline; //FIXME: formerly animation_path

    /// transform node of the spline to be placed in world coordinates
    ref_ptr<PositionAttitudeTransform> transf; //FIXME formerly curve_transf

    /// osg node holding the spline for rendering
    ref_ptr<Geode> spline_geode; //FIXME: formerly line

    /// size of the box kernel for smoothing the spline
    const double BOX_KERNEL_SIZE = 20.0;

    /// parent scene graph node.
    ref_ptr<MapNode> mapNode;

    ref_ptr<SpatialReference> srs;

    /// index of current position on animation_spline
    size_t pos_idx;

    /// index of following control point along the animation spline
    size_t cp_idx;

    /// True when the animation is moving along the spline.
    bool on_spline;

    /// True when arriving to the next photoview position (while not on spline),
    /// false when departing from the photoview to the nearest position
    /// on the spline.
    bool arriving;

    /// Position of the camera when animating arrival or departure.
    osg::Vec3d pos_outside;

    /// Up vector of the camera when animating arrival or departure.
    osg::Vec3d up_outside;

    /// Animation parameter t for animating outside of the spline, when
    /// arriving and departing to and from the PhotoView.
    double t_outside;

    /// Animation parameter for animating image transition from transparent
    /// to opaque and back.
    double t_transition;

    /// Animation step length;
    const double L = 0.001;

    /// FOV of the perspective camera to interpolate from
    double cam_fovy;

    /// Aspect ratio of the perspective camera to interpolate to
    double cam_aspect;

    /// Camera field-of-view interpolated when animating arrival or departure
    double cam_fovy_outside;

    /// Camera aspect ratio interpolated when animating arrival or departure
    double cam_aspect_outside;

    osg::ref_ptr<ElevationPool> ep;
    osg::ref_ptr<ElevationEnvelope> ee;

    double framerate;

    /// true when animation path is shown
    bool path_shown;


    /**
     * @brief generateSpline
     * Initializes and generates the animation spline and spline
     * for rendering from control points.
     */
    void generateSpline();

    /**
     * @brief smoothenAnimationSpline
     * Makes the animation spline smooth by applying low-pas box kernel.
     * @param box_size
     */
    void smoothenAnimationSpline(size_t box_size);

    /**
     * @brief checkPointAboveTerrain
     * @param pos position to check, if it is below terrain, it is moved
     * to be 2m above the terrain.
     * @return true if the point was above the terrain.
     */
    bool checkPointAboveTerrain(Eigen::Vector3d &pos);

    /**
     * @brief initStopPoints
     * Must be run AFTER generateSpline(). For each control point the
     * stop point (which is the point on the spline nearest to the following
     * photoview in the animation) is calculated.
     */
    void initStopPoints();

    /**
     * @brief nextFrameApproachingPhoto
     * Calculates next frame as a linear interpolation between current
     * position and target position of the photoview we are approaching to.
     * @return true when the photo has been approached.
     */
    bool nextFrameApproachingPhoto();

    /**
     * @brief nextFrameDepartingPhoto
     * Opposite of nextFrameApproachingPhoto().
     * @return true when the position of camera returned to the spline.
     */
    bool nextFrameDepartingPhoto();

    /**
     * @brief nextFrameOnSpline
     * Calculates next frame on spline.
     * @return true when the position is nearest to the following photoview,
     * false otherwise.
     */
    bool nextFrameOnSpline();

    /**
     * @brief interpolateOutsidePosition
     * Updates pos_outside according to current parameter t_outside.
     * @return distance between last position on spline and the position
     * of the photoview.
     */
    double interpolateOutsidePosition();


    /**
     * @brief interpolateOutsideUp
     * Updates up_outside according to current parameter t_outside.
     */
    void interpolateOutsideUp();

    /**
     * @brief calculateUp
     * Calculates up vector perpendicular to the tangent plane on given
     * world position.
     * @param pos world position.
     * @return up vector on given world position.
     */
    osg::Vec3d calculateUp(Vec3d pos);

    /**
     * @brief interpolateOutsideIntrinsics
     * Updates cam_fovy_outside and cam_aspect_outside according to the
     * current parameter t_outside.
     */
    void interpolateOutsideIntrinsics();

    /**
     * @brief computeInterpParam
     * Computes interpolation parameter t between two photoviews.
     * Starts at 0 at first photoview, and as the second photoview approaches,
     * the parameter t linearly increases towards 1.
     * @return interpolation parameter t in range 0, 1.
     */
    double computeInterpParam();

    /**
     * @brief imageTransition
     * Animates the opacity of the image.
     */
    void imageTransition();

    /**
     * @brief isTooClose
     * Checks whether the PhotoView is not too close (< 100m) to any other
     * control point stored in all_control_points collection. If it is closer,
     * it is discouraged to be added to the all_control_points collection,
     * since the curvature of the generated spline would be too heavy.
     * @param pv
     * @return
     */
    bool isTooClose(shared_ptr<AbstractPhotoView> pv);

};

}

#endif // ANIMATIONSPLINE_H
