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


#include "animationspline.h"
#include <iostream>

using namespace itr;
using namespace std;

AnimationSpline::AnimationSpline(vector< shared_ptr<AbstractPhotoView> > photoviews,
                                 MapNode *_mapNode,
                                 ref_ptr<SpatialReference> _srs)
    :mapNode(_mapNode), srs(_srs), on_spline(true), t_outside(0),
     t_transition(0), framerate(30.0), path_shown(true)
{
    ep = mapNode->getMap()->getElevationPool();
    ee = ep->createEnvelope(srs, 15);

    transf = new PositionAttitudeTransform;
    for (const shared_ptr<AbstractPhotoView> &pv : photoviews)
    {
        control_points.push_back(ControlPoint(pv));
        all_control_points.push_back(ControlPoint(pv));
    }

    generateSpline();
    //now we have valid spline and control points
    reset();
}

AnimationSpline::AnimationSpline(vector< shared_ptr<AbstractPhotoView> > picked_photoviews,
                                 vector< shared_ptr<AbstractPhotoView> > all_photoviews,
                                 MapNode *_mapNode,
                                 ref_ptr<SpatialReference> _srs)
    :mapNode(_mapNode), srs(_srs), on_spline(true), t_outside(0),
     t_transition(0), framerate(30.0), path_shown(true)
{
    ep = mapNode->getMap()->getElevationPool();
    ee = ep->createEnvelope(srs, 15);

    transf = new PositionAttitudeTransform;

    for (const shared_ptr<AbstractPhotoView> &pv : picked_photoviews)
    {
        control_points.push_back(ControlPoint(pv));
    }
    size_t picked_cnt = 0;
    size_t more_cnt = 0;
    for (const shared_ptr<AbstractPhotoView> &pv : all_photoviews)
    {

        vector< shared_ptr<AbstractPhotoView> >::iterator picked_it;
        picked_it = find(picked_photoviews.begin(), picked_photoviews.end(), pv);
        if (picked_it != picked_photoviews.end())
        {
            //control point must be placed in all cases
            all_control_points.push_back(ControlPoint(pv));
            ++picked_cnt;
        }
        else
        {
            bool isTooCloseToPicked = false;
            for (const shared_ptr<AbstractPhotoView> &ppv : picked_photoviews)
            {
                double dist = (ppv->getECEFPosition() - pv->getECEFPosition()).length();
                if (dist < 100)
                {
                    isTooCloseToPicked = true;
                }
            }
            if (!isTooClose(pv) && !isTooCloseToPicked)
            {
                if (picked_cnt == picked_photoviews.size())
                {
                    ++more_cnt; //adding further control point behind the picked one.
                }
                all_control_points.push_back(ControlPoint(pv));
            }
        }
        if (picked_cnt == picked_photoviews.size() && more_cnt >= 3)
        {
            //we added all needed control points so far.
            break;
        }
    }
    generateSpline();
    //now we have valid spline and control points
    reset();
}

bool AnimationSpline::isTooClose(shared_ptr<AbstractPhotoView> pv)
{
    for (ControlPoint &cp : all_control_points)
    {
        double dist = (cp.getPhotoViewPosition() - pv->getECEFPosition()).length();
        if (dist < 100)
        {
            return true;
        }
    }
    return false;
}

void AnimationSpline::reset()
{
    std::cout << "resetting animation..." << std::endl;
    std::cout << "cp size: " << control_points.size() << ", all cp size: " << all_control_points.size() << std::endl;
    on_spline = true;
    if (control_points.size() >= all_control_points.size() - 2)
    {
        pos_idx = animation_spline.size();
        cp_idx = control_points.size() - 3; // ommit the first photo along the path
    }
    else
    {
        //we want to start from the first photo user selected
        pos_idx = control_points[control_points.size() - 1].getStopPointIndex() + 1;
        cp_idx = control_points.size() - 1;
    }
    std::cout << "pos idx: " << pos_idx << std::endl;
}

void AnimationSpline::generateSpline()
{
    spline_geode = new Geode();
    Geometry *pp_geom = new Geometry();
    spline_geode->addDrawable(pp_geom);

    Vec3Array *pp_vert = new Vec3Array;
    DrawElementsUInt *pp_line = new DrawElementsUInt(PrimitiveSet::LINE_STRIP, 0);
    Vec4dArray *pp_colors = new Vec4dArray;

    //calculate bbox to be able to calculate the spline in normalized
    //coordinates
    vector<Eigen::Vector3d> eigen_loc;
    for (const ControlPoint &cp : all_control_points)
    {
        eigen_loc.push_back(cp.getControlPointEigen());
    }
    Eigen::Vector3d centroid;
    BBox3D box = SfMLoader::findPointCloudBBox(eigen_loc, centroid);
    Eigen::Vector3d base = box.max - box.min;
    double base_s = fmax(base.x(), fmax(base.y(), base.z()));
    osg::Vec3d cen(centroid.x(), centroid.y(), centroid.z());

    vector<Eigen::Vector3d>::const_iterator vit;
    //interpolate linearly between first and second point
    /*for (vit = eigen_loc.begin(); vit != (eigen_loc.begin() + 1); ++vit)
    {
        Eigen::Vector3d p1 = (*vit - centroid) / base_s;
        Eigen::Vector3d p2 = (*(vit+1) - centroid) / base_s;

        double delta = L;
        for (double t = 0; t < 1; t += delta)
        {
            Eigen::Vector3d pos = p1 * t + p2 * (1-t);
            Eigen::Vector3d dpos_dt = p1 - p2;
            delta = L / sqrt(dpos_dt.norm());
            Eigen::Vector3d wp = (pos * base_s) + centroid;
            checkPointAboveTerrain(wp);
            osg::Vec3d p = osg::Vec3d(wp.x(), wp.y(), wp.z());
            animation_spline.push_back(p);
        }
    }*/
    //now generate catmull-rom
    for (vit = (eigen_loc.begin() + 1); vit != (eigen_loc.end() - 2);
         ++vit)
    {
        Eigen::Vector3d p0 = (*(vit-1) - centroid) / base_s;
        Eigen::Vector3d p1 = (*vit - centroid) / base_s;
        Eigen::Vector3d p2 = (*(vit+1) - centroid) / base_s;
        Eigen::Vector3d p3 = (*(vit+2) - centroid) / base_s;

        double delta = L;
        //double delta = 0.005 / (p2 - p1).norm();
        //delta = fmin(delta, 0.005); //at least three poits per segment
        for (double t = 0; t < 1; t += delta)
        {
            //add points to the line according to the catmull-rom formula
            //https://www.mvps.org/directx/articles/catmull/
            double t2 = t * t;
            double t3 = t2 * t;
            Eigen::Vector3d pos = ((p1 * 2.0) +
                                   (-p0 + p2) * t +
                                   (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2 +
                                   (-p0 + p1 * 3.0 - p2 * 3.0 + p3) * t3) * 0.5;
            Eigen::Vector3d dpos_dt = ((-p0 + p2) +
                                       (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * 2*t +
                                       (-p0 + p1 * 3.0 - p2 * 3.0 + p3) * 3 * t2) * 0.5;
            delta = L / sqrt(dpos_dt.norm());
            Eigen::Vector3d wp = (pos * base_s) + centroid;
            checkPointAboveTerrain(wp);

            osg::Vec3d p = osg::Vec3d(wp.x(), wp.y(), wp.z());
            animation_spline.push_back(p);
        }
    }
    //smoothen animation path using low-pass box filter
    smoothenAnimationSpline(BOX_KERNEL_SIZE);

    //put points from animation path to path geometry
    int idx = 0;
    for (const osg::Vec3d &pos : animation_spline)
    {
        osg::Vec3d scaled_pos = (pos - cen) / base_s;
        pp_vert->push_back(scaled_pos);
        pp_line->push_back(idx);
        pp_colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
        ++idx;
    }

    mapNode->addChild(transf); //add the spline to the graph
    transf->setScale(osg::Vec3d(base_s, base_s, base_s));
    transf->setPosition(cen);
    transf->addChild(spline_geode);

    pp_geom->setVertexArray(pp_vert);
    pp_geom->addPrimitiveSet(pp_line);
    pp_geom->setColorArray(pp_colors);
    pp_geom->setColorBinding(Geometry::BIND_PER_VERTEX);
    osgEarth::Registry::shaderGenerator().run(spline_geode);

    initStopPoints();
}

void AnimationSpline::initStopPoints()
{
    int num = 0;
    size_t last_point_idx = 0;
    for (ControlPoint &cp : control_points)
    {
        //std::cout << "stop point number: " << num << std::endl;
        osg::Vec3d cp_pos = cp.getPhotoViewPosition();
        double mindist = numeric_limits<double>::max();
        double dist;
        osg::Vec3d stop_point = cp_pos;
        size_t stop_point_idx = 0;
        size_t idx = 0;
        for (const osg::Vec3d &pos : animation_spline)
        {
            dist = (cp_pos - pos).length();
            if (dist < mindist)
            {
                mindist = dist;
                stop_point = pos;
                stop_point_idx = idx;
            }
            //std::cout << "found mindist: " << mindist << std::endl;
            ++idx;
        }
        cp.setStopPoint(stop_point);
        cp.setStopPointIndex(stop_point_idx);
        last_point_idx = stop_point_idx;
        std::cout << "stop point idx: " << stop_point_idx << std::endl;
        ++num;
    }

    //sort according to set stop point index
    std::sort(control_points.begin(), control_points.end(), ControlPointIndexPredicate());
}


void AnimationSpline::smoothenAnimationSpline(size_t box_size)
{
    //use box kernel
    double coef = 1.0 / box_size;
    if (animation_spline.size() > box_size)
    {
        vector<osg::Vec3d>::iterator it;
        for (it = animation_spline.begin();
             it != (animation_spline.end() - box_size);
             ++it)
        {
            osg::Vec3d res;
            for (int i = 0; i < box_size; ++i)
            {
                res += *(it+i) * coef;
            }
            *it = res;
        }
    }
}


bool AnimationSpline::checkPointAboveTerrain(Eigen::Vector3d &pos)
{
    GeoPoint gp(srs->getGeocentricSRS(), pos.x(), pos.y(), pos.z(), ALTMODE_ABSOLUTE);
    GeoPoint wp = gp.transform(srs);
    float elev = ee->getElevation(wp.x(), wp.y());
    //GeoPoint relp(srs, wp.x(), wp.y(), 0, ALTMODE_RELATIVE);
    //relp.transformZ(ALTMODE_ABSOLUTE, mapNode->getTerrainEngine()->getTerrain());
    if (wp.z() - 35.0 < elev)
    {
        //point on the path is under the ground
        elev += 35.0; //add two meters to the elevation
        GeoPoint relp(srs, wp.x(), wp.y(), elev, ALTMODE_ABSOLUTE);
        GeoPoint ecef_fix = relp.transform(srs->getGeocentricSRS()); //to ECEF
        pos.x() = ecef_fix.x();
        pos.y() = ecef_fix.y();
        pos.z() = ecef_fix.z();
        return false;
    }
    return true;
}

void AnimationSpline::remove()
{
    mapNode->removeChild(transf);
}


bool AnimationSpline::nextFrameOnSpline()
{
    if (pos_idx > 0)
    {
        --pos_idx;
    }
    pos_outside = animation_spline[pos_idx]; // security
    size_t stop_pt_idx = control_points[cp_idx].getStopPointIndex();
    std::cout << "pos_idx: " << pos_idx << ", stop_pt_idx: " << stop_pt_idx << "cp_idx: " << cp_idx << std::endl;
    if (pos_idx == stop_pt_idx && cp_idx >= 0)
    {
        return true;
    }
    return false;
}

double AnimationSpline::interpolateOutsidePosition()
{
    osg::Vec3d pos_spline = animation_spline[pos_idx]; //last position on spline
    osg::Vec3d pos_photoview = control_points[cp_idx].getPhotoViewPosition();
    pos_outside = pos_spline * (1 - t_outside) + pos_photoview * t_outside;
    return (pos_spline - pos_photoview).length();
}


void AnimationSpline::interpolateOutsideUp()
{
    double t = computeInterpParam();
    if (!on_spline) // && !arriving
    {
        //we are outside the spline and we are departing
        t = t_outside;
    }
    osg::Vec3d up_spline = calculateUp(animation_spline[pos_idx]);
    osg::Quat pv_orientation = control_points[cp_idx].getPhotoViewOrientation();
    osg::Vec3d up_photoview = pv_orientation * osg::Vec3d(0, -1, 0);
    up_outside = up_spline * (1 - t) + up_photoview * t;
}


double AnimationSpline::computeInterpParam()
{
    if (cp_idx >= control_points.size() - 1)
    {
        //cannot retrieve previous control point
        return 0;
    }
    size_t startIdx = control_points[cp_idx].getStopPointIndex();
    size_t nextStopIdx = control_points[cp_idx + 1].getStopPointIndex();
    return 1 - (double)(pos_idx - startIdx) / (nextStopIdx - startIdx);
}

void AnimationSpline::interpolateOutsideIntrinsics()
{
    double t = computeInterpParam();
    if (!on_spline) // && !arriving
    {
        //we are outside the spline and we are departing
        t = t_outside;
    }
    std::cout << "Interp outside intrinsics: " << t << std::endl;
    //double t = 1.0 - t_outside;
    shared_ptr<AbstractPhotoView> pv = control_points[cp_idx].getPhotoView();
    /*shared_ptr<AbstractPhotoView> pv_prev;
    if (cp_idx >= control_points.size() - 1)
    {
        pv_prev = pv;
    }
    else
    {
        pv_prev = control_points[cp_idx + 1].getPhotoView();
    }*/
    double aspect_photoview = pv->getAspect();
    double fov_photoview = pv->getFOVYDeg();
    //cam_aspect_outside = pv_prev->getAspect() * (1 - t) + aspect_photoview * t;
    //cam_fovy_outside = pv_prev->getFOVYDeg() * (1 - t) + fov_photoview * t;
    cam_aspect_outside = cam_aspect * (1 - t) + aspect_photoview * t;
    cam_fovy_outside = cam_fovy * (1 - t) + fov_photoview * t;
}

void AnimationSpline::imageTransition()
{
    shared_ptr<AbstractPhotoView> pv = control_points[cp_idx].getPhotoView();
    double t;
    t = t_transition > 1.0 ? 1.0 : t_transition;
    t = t < 0.0 ? 0.0 : t;
    pv->setOpacity(t);
}

bool AnimationSpline::nextFrameApproachingPhoto()
{
    std::cout << "approaching t_outside: " << t_outside << std::endl;
    if (t_outside <= 1)
    {
        double dist = interpolateOutsidePosition();
        interpolateOutsideUp();
        interpolateOutsideIntrinsics();
        //dist = 5;
        //minimum 4 meters so that animation is long enough
        dist = dist < 4 ? 4 : dist;
        //dist /= 2.0;
        t_outside += 1.0 / (dist * framerate);
        return false;
    }
    else if (t_transition <= 1)
    {
        imageTransition();
        t_transition += 1.0 / (3.0 * framerate);
        return false;
    }
    t_outside = 1.0;
    t_transition = 1.0;
    return true;
}

bool AnimationSpline::nextFrameDepartingPhoto()
{
    std::cout << "departing t_outside: " << t_outside << std::endl;
    if (t_transition >= 0)
    {
        imageTransition();
        t_transition -= 1.0 / (3.0 * framerate);
        return false;
    }
    else if (t_outside >= 0)
    {
        double dist = interpolateOutsidePosition();
        interpolateOutsideUp();
        interpolateOutsideIntrinsics();

        //osg::Vec3d old_center = control_points[cp_idx].getPhotoView()->getCenterInWorldCoords();
        //osg::Vec3d new_center = control_points[cp_idx - 1].getPhotoView()->getCenterInWorldCoords();
        //double alpha = (old_center * new_center) / (old_center.length() * new_center.length());
        //alpha = (alpha * 180.0) / M_PI;
        //minimum 4 meters so that animation is long enough
        dist = dist < 4 ? 4 : dist;
        //dist = 5;
        //std::cout << "departing, dist: " << dist << ", angle: " << alpha << std::endl;
        t_outside -= 1.0 / ((dist) * framerate); //40 degrees per second
        return false;
    }
    t_outside = 0.0;
    t_transition = 0.0;
    return true;
}

bool AnimationSpline::nextFrame()
{
    std::cout << "on spline: " << on_spline << std::endl;
    if (on_spline)
    {
        if (nextFrameOnSpline())
        {
            on_spline = false;
            arriving = true;
            t_outside = 0;
        }
        else {
            interpolateOutsideUp();
        }
    }
    else if (arriving)
    {
        if (nextFrameApproachingPhoto())
        {
            arriving = false;
            return true; //we need to stop the animation now
        }
    }
    else
    {
        if (nextFrameDepartingPhoto())
        {
            --cp_idx; // move to next photoview
            on_spline = true;
        }
    }
    return false;
}

osg::Vec3d AnimationSpline::position()
{
    if (on_spline)
    {
        return animation_spline[pos_idx];
    }
    return pos_outside;
}

osg::Vec3d AnimationSpline::slerp(osg::Vec3d p0, osg::Vec3d p1, double t)
{
    //shift to origin for computational stability
    osg::Vec3d p0_n = osg::Vec3d(0, 0, 0);
    osg::Vec3d p1_n = p1 - p0;

    double omega = acos(p0_n * p1_n);
    double term1 = sin((1 - t) * omega);
    double term2 = sin(t * omega);
    double sin_omega = sin(omega);
    osg::Vec3d res = p0_n * (term1/sin_omega) + p1_n * (term2/sin_omega);

    //shift back
    return res + p0;
}

osg::Vec3d AnimationSpline::center()
{
    if (!on_spline && !arriving)
    {

        //departing, interpolate center from current to the next photoview
        osg::Vec3d old_center = control_points[cp_idx].getPhotoView()->getCenterInWorldCoords();
        osg::Vec3d view_dir = old_center - control_points[cp_idx].getPhotoViewPosition();
        view_dir.normalize();
        if (cp_idx > 0)
        {
            osg::Vec3d new_center = control_points[cp_idx - 1].getPhotoView()->getCenterInWorldCoords();
            osg::Vec3d new_dir = new_center - position();
            double dist_to_new = new_dir.length();
            osg::Vec3d view_dir_equal = view_dir * dist_to_new;
            //move the old center so that the position is in the center of the slerp
            old_center = view_dir_equal + position();

            double st = t_outside;//sqrt(t_outside);
            return slerp(old_center, new_center, 1.0 - st);
        }
        return old_center;
    }
    //enlarge the view dir vector to look on more distant point so that camera
    //rotates smoothly when arriving
    osg::Vec3d view_dir = (control_points[cp_idx].getPhotoView()->getCenterInWorldCoords() - position()) * 10.0;

    return (view_dir + position());
}

osg::Vec3d AnimationSpline::calculateUp(Vec3d pos)
{
    GeoPoint gp(srs->getGeocentricSRS(),
                pos.x(), pos.y(), pos.z(), ALTMODE_ABSOLUTE);
    osg::Vec3d up;
    gp.createWorldUpVector(up);

    osg::Vec3d forward = center() - position();
    forward.normalize();
    up.normalize();
    osg::Vec3d side = forward ^ up;
    side.normalize();
    up = forward ^ side;
    up.normalize();
    return -up; //so that up is not upside down
}

osg::Vec3d AnimationSpline::up()
{
    //FIXME: problem here, up is nan with spherical panos
    if (on_spline)
    {
        osg::Vec3d up = calculateUp(position());
        up_outside = up; //security
        return up;
    }
    interpolateOutsideUp();
    return up_outside;
}


bool AnimationSpline::finished()
{
    if (pos_idx == 0 || cp_idx == 0)
    {
        return true;
    }
    return false;
}

double AnimationSpline::fovy()
{
    if (on_spline)
    {
        return cam_fovy;
    }
    return cam_fovy_outside;
}

double AnimationSpline::aspect()
{
    /*if (on_spline)
    {
        return cam_aspect;
    }*/
    return cam_aspect;
    //return cam_aspect_outside;
}

void AnimationSpline::setFovy(double fov)
{
    cam_fovy = fov;
    cam_fovy_outside = fov; // security
}

void AnimationSpline::setAspect(double aspect)
{
    cam_aspect = aspect;
    cam_aspect_outside = aspect; // security
}

void AnimationSpline::setFramerate(double f)
{
    framerate = f;
}

ControlPoint AnimationSpline::getControlPoint()
{
    return control_points[cp_idx];
}

double AnimationSpline::distanceToNextControlPoint()
{
    Vec3d cp_pos = control_points[cp_idx].getControlPoint();
    return (position() - cp_pos).length();
}

double AnimationSpline::distanceToPreviousControlPoint()
{
    if (cp_idx < control_points.size() - 1)
    {
        Vec3d cp_pos = control_points[cp_idx + 1].getControlPoint();
        return (position() - cp_pos).length();
    }
    return 0;
}

bool AnimationSpline::isPathShown()
{
    return path_shown;
}

void AnimationSpline::togglePathShown()
{
    if (path_shown)
    {
        transf->removeChild(spline_geode);
        path_shown = false;
    }
    else
    {
        transf->addChild(spline_geode);
        path_shown = true;
    }

}
