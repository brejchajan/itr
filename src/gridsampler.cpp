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


#include "gridsampler.h"
#include <iostream>

using namespace itr;
using namespace osgEarth;
using namespace osg;

GridSampler::GridSampler(MapNode *_mapNode, GeoPoint center,
            double width, double height,
            double _offset_x, double _offset_y, double above_ground):
    mapNode(_mapNode), offset_x(_offset_x), offset_y(_offset_y),
    above_ground(above_ground)
{
    if (!center.makeGeographic())
    {
        throw std::runtime_error("Unable to convert center to "
                                 "geographic coordinates.");
    }
    double lat_rad = (center.y() / 180.0) * M_PI;
    double lon_rad = (center.x() / 180.0) * M_PI;
    double west_bearing_rad = 1.5 * M_PI;
    double east_bearing_rad = M_PI / 2.0;
    double north_bearing_rad = 0;
    double south_bearing_rad = M_PI;
    double out_lat, out_lon;
    GeoMath::destination(lat_rad, lon_rad, west_bearing_rad, width,
                         out_lat, out_lon);
    nw_lon = out_lon;
    GeoMath::destination(lat_rad, lon_rad, north_bearing_rad, height,
                         out_lat, out_lon);
    nw_lat = out_lat;
    GeoMath::destination(lat_rad, lon_rad, east_bearing_rad, width,
                         out_lat, out_lon);
    se_lon = out_lon;
    GeoMath::destination(lat_rad, lon_rad, south_bearing_rad, height,
                         out_lat, out_lon);
    se_lat = out_lat;

    srs = SpatialReference::get("epsg:4326");
    resetPosition();

    ep = mapNode->getMap()->getElevationPool();

    ep->setMaxEntries(1000);

    //Level 15 equals to 4.773 m/pix, which is optimal for north US
    //and europe
    ee = ep->createEnvelope(srs, 16);

}

void GridSampler::resetPosition()
{
    current_lat = nw_lat;
    current_lon = nw_lon;
}

osg::Vec3d GridSampler::nextPosition()
{
    double east_bearing_rad = M_PI / 2.0;
    double south_bearing_rad = M_PI;
    double tmp_lat, tmp_lon;

    //query the point
    double lon_deg = (current_lon * 180.0) / M_PI;
    double lat_deg = (current_lat * 180.0) / M_PI;
    float elev = ee->getElevation(lon_deg, lat_deg) + above_ground;
    GeoPoint gp(srs, lon_deg, lat_deg, elev, ALTMODE_ABSOLUTE);
    GeoPoint gp_ecef = gp.transform(srs->getGeocentricSRS());
    osg::Vec3d outp(gp_ecef.x(), gp_ecef.y(), gp_ecef.z());

    //update current position
    if (current_lon <= se_lon)
    {
        double offset = offset_x > 0 ? offset_x : 0.0001;
        GeoMath::destination(current_lat, current_lon, east_bearing_rad, offset,
                             tmp_lat, tmp_lon);
        //std::cout << current_lat << " " << current_lon << " tmp lon " << tmp_lon << " se lon " << se_lon << std::endl;
        current_lon = tmp_lon;
    }
    else
    {
        if (current_lat >= se_lat)
        {
            double offset = offset_y > 0 ? offset_y : 0.0001;
            GeoMath::destination(current_lat, current_lon, south_bearing_rad, offset,
                             tmp_lat, tmp_lon);
            current_lon = nw_lon;
            current_lat = tmp_lat;
            //std::cout << current_lat << " " << current_lon << " tmp lat " << tmp_lat << " se lat " << se_lat << std::endl;
            if (current_lat < se_lat)
            {
                return osg::Vec3d(0, 0, 0);
            }
        }
        else
        {
            return osg::Vec3d(0, 0, 0);
        }
    }

    return outp;
}

void GridSampler::sample(vector<osg::Vec3d> &points_ecef)
{
    vector<GeoPoint> geo_points;
    vector<osg::Vec3d> points;
    vector<float> elev;

    double east_bearing_rad = M_PI / 2.0;
    double south_bearing_rad = M_PI;

    //std::pair<float, float> res =
    //        ee->getElevationAndResolution((nw_lon * 180.0) / M_PI,
    //                                      (nw_lat * 180.0) / M_PI);
    //std::cout << "elevation resolution: " << res.second << std::endl;
    std::cout << "Sampling terrain..." << std::endl;
    double lat = nw_lat;
    double lon = nw_lon;
    double tmp_lat, tmp_lon;
    while (lat >= se_lat)
    {
        while (lon <= se_lon)
        {
            double lon_deg = (lon * 180.0) / M_PI;
            double lat_deg = (lat * 180.0) / M_PI;

            //FIXME: sample only points that are not near to poles,
            //since it causes bug when no map is shown.
            if (lon_deg >= -180 && lon_deg <= 180 && lat_deg >= -60 && lat_deg <= 60)
            {
                points.push_back(osg::Vec3d(lon_deg, lat_deg, 0));
                //std::cout << "adding: " << lat_deg << ", " << lon_deg << std::endl;
            }
            GeoMath::destination(lat, lon, east_bearing_rad, offset_x,
                                 tmp_lat, tmp_lon);
            lon = tmp_lon;
        }
        GeoMath::destination(lat, lon, south_bearing_rad, offset_y,
                             tmp_lat, tmp_lon);
        lat = tmp_lat;
        lon = nw_lon;
    }
    std::cout << "Getting elevations..." << std::endl;
    ee->getElevations(points, elev);

    for (size_t i = 0; i < elev.size(); ++i)
    {
        //std::cout << "processing elevation " << i << ", out of: " << elev.size() << std::endl;
        osg::Vec3d point = points[i];

        if (elev[i] == NO_DATA_VALUE)
        {
            //std::cout << "unfetched lat: " << point.y() << ", lon: " << point.x() << std::endl;
        }

        GeoPoint gp(srs, point.x(), point.y(), elev[i] + above_ground, ALTMODE_ABSOLUTE);
        GeoPoint gp_ecef = gp.transform(srs->getGeocentricSRS());
        osg::Vec3d outp(gp_ecef.x(), gp_ecef.y(), gp_ecef.z());
        points_ecef.push_back(outp);
    }
    //ep->clear();
}
