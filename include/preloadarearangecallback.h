/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   28.1.2018
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



#ifndef PRELOADAREARANGECALLBACK_H
#define PRELOADAREARANGECALLBACK_H

//stl headers
#include <vector>

//osgEarth headers
#include <osgEarth/TerrainEngineNode>

//OSG headers
#include <osg/Vec3d>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

//local headers
#include "abstractphotoview.h"

using namespace osgEarth;
using namespace osg;
using namespace std;

namespace itr{


struct PreloadAreaRangeCallback : public osgEarth::ComputeRangeCallback
{
    vector<osg::Vec3d> positions;
    static constexpr double CLOSE_DISTANCE = 15000; //10 km

    PreloadAreaRangeCallback(vector< shared_ptr<AbstractPhotoView> > pvs)
    {
        for (shared_ptr<AbstractPhotoView> pv : pvs)
        {
            positions.push_back(pv->getECEFPosition());
        }
    }

    bool isClose(osg::Vec3 p)
    {
        for (osg::Vec3d &pos : positions)
        {
            double dist = (pos - p).length();
            //std::cout << "dist: " << dist << std::endl;
            if (dist <= CLOSE_DISTANCE)
            {
                return true;
            }
        }
        return false;
    }

    virtual float operator()(osg::Node* node, osg::NodeVisitor& nv)
    {
        /*osg::Vec3d node_center = node->getBound().center();
        if (isClose(node_center))
        {
            //std::cout << "returning close range." << std::endl;
            return 0.0f; //the range that we want to return
        }*/
        return -1; //else fallback to the default
    }

    virtual void addPosition(osg::Vec3d pos)
    {
        positions.push_back(pos);
    }

    virtual void pop_backPosition()
    {
        positions.pop_back();
    }
    virtual bool isEmpty()
    {
        return positions.size() == 0;
    }
};

} //namespace itr
#endif // PRELOADAREARANGECALLBACK_H
