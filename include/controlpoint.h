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



#ifndef CONTROLPOINT_H
#define CONTROLPOINT_H

// STL headers
#include <vector>
#include <cmath>
#include <memory>

// OpenSceneGraph headers
#include "osg/Vec3d"

// local headers
#include "photoview.h"

// Eigen headers
#include "Eigen/Dense"

namespace itr{

/**
 * @brief The ControlPoint class
 * Represents control point on animation spline.
 */
class ControlPoint{

public:
    /**
     * @brief ControlPoint
     * Creates new ControlPoint to represent PhotoView on the spline.
     * @param pv photoview to be placed on the spline
     */
    ControlPoint(shared_ptr<AbstractPhotoView> pv);


    /**
     * @brief getControlPoint
     * Returns the control point for generating the spline.
     * @return the control point.
     */
    osg::Vec3d getControlPoint() const;

    Eigen::Vector3d getControlPointEigen() const;

    /**
     * @brief getPhotoViewPosition
     * @return position of the photoview in world coordinates.
     */
    osg::Vec3d getPhotoViewPosition();

    /**
     * @brief getPhotoViewOrientation
     * @return quaternion deifining photoview orientation
     */
    osg::Quat getPhotoViewOrientation();

    shared_ptr<AbstractPhotoView> getPhotoView();

    /**
     * @brief setStopPoint
     * Sets the stop point, which must be located on the animation spline
     * and is supposed to be the nearest point on the spline to the position of
     * the photoview.
     * @param stop_pt
     */
    void setStopPoint(osg::Vec3d stop_pt);

    void setStopPointIndex(size_t idx);

    osg::Vec3d getStopPoint();

    size_t getStopPointIndex();



private:
    /// Point that is used as a control point for generating spline
    osg::Vec3d control_point;

    /// Nearest point to the control_point lying on the generated spline
    osg::Vec3d stop_point;

    /// PhotoView that corresponds to this control point
    shared_ptr<AbstractPhotoView> photoview;

    size_t stop_point_idx;
};

struct ControlPointIndexPredicate
{
    inline bool operator() (ControlPoint &p1,
                            ControlPoint &p2) const
    {
        return (p1.getStopPointIndex() < p2.getStopPointIndex());
    }
};

}

#endif // CONTROLPOINT_H
