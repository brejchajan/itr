/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   21.08.2017
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



#ifndef GRIDSAMPLER_H
#define GRIDSAMPLER_H

// STL headers
#include <vector>
#include <cmath>
#include <stdexcept>

// OSG headers
#include <osg/Config>

// osgEarth headers
#include <osgEarth/MapNode>
#include <osgEarth/GeoData>
#include <osgEarth/GeoMath>
#include <osgEarth/SpatialReference>
#include <osgEarth/ElevationPool>
#include "osgEarthDrivers/xyz/XYZOptions"


using namespace std;
using namespace osg;
using namespace osgEarth;

namespace itr{

    /**
     * @brief The GridSampler class
     * Class for sampling points forming a regular rectangular geo-grid.
     */
    class GridSampler
    {
    public:

        /**
         * @brief GridSampler
         * Creates a new GridSampler to sample points on elevation map.
         * @param mapNode mapNode that will be used to get the elevation
         * information from
         * @param center center of the rectangle defining the sampling grid
         * @param width width of the geo rectangle in meters
         * @param height height of the geo rectangle in meters
         * @param offset_x offset between consecutive samples in x-direction
         * in meters
         * @param offset_y offset between consecutive samples in y-direction
         * in meters
         */
        GridSampler(MapNode *mapNode, GeoPoint center,
                    double width, double height,
                    double offset_x, double offset_y,
                    double above_ground = 0.0);


        /**
         * @brief sample
         * Samples the points lying on the geo-grid by querying MapNode
         * data source and stores them in result. The result is not cleared,
         * so new points are appended in case result is not empty.
         */
        void sample(vector<osg::Vec3d> &result);

        /**
         * @brief nextPosition
         * Method for sequential sampling a single location at a time.
         * @return if next position is inside the defined geo-rectangle
         * the next position is returned, Vec3(0,0,0) otherwise.
         */
        osg::Vec3d nextPosition();

        /**
         * @brief resetPosition
         * Resets current position for sequential sampling at the start;
         */
        void resetPosition();

    private:
        /// MapNode that will be used to get the elevation information
        MapNode *mapNode;

        osg::ref_ptr<ElevationPool> ep;

        osg::ref_ptr<ElevationEnvelope> ee;

        /// north-west latitude
        double nw_lat;

        /// nort-west longitude
        double nw_lon;

        /// south-east latitude
        double se_lat;

        /// south-east longitude
        double se_lon;

        /// Offset in x-direction between consecutive samples
        double offset_x;

        /// Offset in y-direction between consecutive samples
        double offset_y;

        /// Current latitude for nextPosition method.
        double current_lat;

        /// Current longitude for nextPosition method.
        double current_lon;

        /// this elevation is added to each sampled position
        double above_ground;

        /// Spatial reference
        SpatialReference *srs;
    };
}

#endif //GRIDSAMPLER_H
