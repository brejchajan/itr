/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   21.09.2017
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



#ifndef JSONEXPORTER_H
#define JSONEXPORTER_H

//local headers
#include "animationspline.h"
#include "photoview.h"
#include "panophotoview.h"
#include "util.h"

//STL headers
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "osg/Vec3d"
#include "osgEarth/GeoData"

//nlohmann/json
#include "json.hpp"

//boost
#include <boost/filesystem.hpp>

//libxml2
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>



namespace itr{

using namespace osg;
using namespace osgEarth;
using namespace std;
using json = nlohmann::json;

class JsonExporter {

public:
    JsonExporter(vector< shared_ptr<AbstractPhotoView> > photoviews,
                 ref_ptr<MapNode> mapNode,
                 vector< shared_ptr<AbstractPhotoView> > pickedPvs,
                 shared_ptr<AnimationSpline> anim_apline,
                 bool exportOnlyPicked);

    /**
     * @brief save
     * Saves the scene data into json file.
     * @param filename path and name of the json file.
     * @root_dir the directory to which the photos are relative. Only relative
     * paths of the photos are stored in the output json file.
     */
    void save(string filename, string root_dir,
              double center_lat = 360, double center_lon = 360);

    /**
     * @brief saveToKML
     * Save the scene data into KML file, which can be opened in Google Earth
     * desktop.
     * @param filename filename path and name of the kml file.
     * @param root_dir the directory to which the photos are relative.
     * Only relative paths of the photos are stored in the output KML file.
     * @param center_lat center latitude which will be used to detect far
     * photoviews (farer than 20 km) that will not be included into the KML
     * file.
     * @param center_lon center longitude (see center_lat);
     */
    void saveToKML(string filename, string root_dir,
                   double center_lat = 360, double center_lon = 360);

    osg::Vec3d ECEFToWgs84(osg::Vec3d pos);

    osg::Vec3d ECEFToWgs84OnGround(osg::Vec3d pos);

    osg::Vec3d findSceneCenter();

private:
    vector< shared_ptr<AbstractPhotoView> > photoviews;
    ref_ptr<SpatialReference> srs;
    ref_ptr<MapNode> mapNode;
    osg::ref_ptr<ElevationPool> ep;
    osg::ref_ptr<ElevationEnvelope> ee;
    vector< shared_ptr<AbstractPhotoView> > pickedPvs;
    shared_ptr<AnimationSpline> anim_spline;
    bool exportOnlyPicked;


    //xml related helper methods
    static void writeXmlStartElement(xmlTextWriterPtr w, const char *name);

    static void writeXmlAttribute(xmlTextWriterPtr w, const char *name,
                                  const char *value);
    static void writeXmlString(xmlTextWriterPtr w, const char *text);

    /**
     * ConvertInput:
     * taken from: http://xmlsoft.org/examples/testWriter.c
     * @in: string in a given encoding
     * @encoding: the encoding used
     *
     * Converts @in into UTF-8 for processing with libxml2 APIs
     *
     * Returns the converted UTF-8 string, or NULL in case of error.
     */
    static xmlChar *convertInput(const char *in, const char *encoding);

    /**
     * @brief rotToGoogleEarthYPR
     * Converts rotation in local coordinates to yaw, pitch, roll used by
     * Google Earth.
     * @param o orientation quaterion in local coordinates
     * @param yaw corresponding to heading in Google Earth
     * @param pitch corresponding to tilt in Google Earth
     * @param roll corresponding to roll in Google Earth
     */
    void rotToGoogleEarthYPR(Quat o, double &yaw, double &pitch, double &roll);
};
}

#endif // JSONEXPORTER_H
