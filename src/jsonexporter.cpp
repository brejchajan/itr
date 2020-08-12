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

#include "jsonexporter.h"

using namespace itr;
namespace fs = boost::filesystem;

JsonExporter::JsonExporter(vector<shared_ptr<AbstractPhotoView> > photoviews,
                           ref_ptr<MapNode> mapNode,
                           vector< shared_ptr<AbstractPhotoView> > pickedPvs,
                           shared_ptr<AnimationSpline> anim_spline,
                           bool exportOnlyPicked)
    :photoviews(photoviews), mapNode(mapNode), pickedPvs(pickedPvs),
     anim_spline(anim_spline), exportOnlyPicked(exportOnlyPicked)
{
    srs = SpatialReference::get("epsg:4326");
    ep = mapNode->getMap()->getElevationPool();
    ee = ep->createEnvelope(srs, 15);
}

void JsonExporter::save(string filename, string root_path,
                        double center_lat, double center_lon)
{
   json res;


   json photoviews_arr = json::array();
   vector< shared_ptr<AbstractPhotoView> > &pvs = exportOnlyPicked ? pickedPvs : photoviews;
   for (shared_ptr<AbstractPhotoView> pv : pvs)
   {
       PhotoView *pvp = dynamic_cast<PhotoView *>(pv.get());
       if (pvp != NULL)
       {
           json scene_center;
           if (center_lat >= -90 && center_lat <= 90 &&
               center_lon >= -180 && center_lon <= 180)
           {
               std::cout << "center_lat: " << center_lat << ", " << center_lon << std::endl;
               GeoPoint our_center(srs, center_lon, center_lat, 1.8, ALTMODE_RELATIVE);
               our_center.transformZ(ALTMODE_ABSOLUTE,  mapNode->getTerrainEngine()->getTerrain());
               GeoPoint our_center_ecef;
               our_center.transform(srs->getGeocentricSRS(), our_center_ecef);
               osg::Vec3d center_xyz(our_center_ecef.x(), our_center_ecef.y(), our_center_ecef.z());
               if ((pv->getECEFPosition() - center_xyz).length() > 40000)
               {
                   std::cout << "filtered out, due to length: " << (pv->getECEFPosition() - center_xyz).length() << std::endl;
                   //if more than 20km from center do not export
                   continue;
               }

               scene_center.push_back((double)our_center.x());
               scene_center.push_back((double)our_center.y());
               scene_center.push_back((double)our_center.z());
           }
           else
           {
               osg::Vec3d sc_wgs = findSceneCenter();
               scene_center.push_back((double)sc_wgs.x());
               scene_center.push_back((double)sc_wgs.y());
               scene_center.push_back((double)sc_wgs.z());
           }
           res["scene_center"] = scene_center;


           json jpv;
           fs::path parent_path(root_path), child_path(pv->getPhotoName());
           jpv["filename"] = fs::relative(child_path, parent_path).string();

           jpv["fov"] = pv->getFOV();
           json position;
           osg::Vec3d pos = ECEFToWgs84(pv->getECEFPosition());
           position.push_back((double)pos.x());
           position.push_back((double)pos.y());
           position.push_back((double)pos.z());
           jpv["position"] = position;

           json rotation;
           osg::Quat rot = pv->getOrientationLocal();

           double yaw, pitch, roll;
           rotToGoogleEarthYPR(rot, yaw, pitch, roll);
           if (pitch > 120 || pitch < 60)
           {
               std::cerr << "Filtered out due to angle." << std::endl;
               //do not use, the photo is looking towards ground.
               continue;
           }


           rotation.push_back((double)rot.x());
           rotation.push_back((double)rot.y());
           rotation.push_back((double)rot.z());
           rotation.push_back((double)rot.w());
           jpv["rotation"] = rotation;
           photoviews_arr.push_back(jpv);
       }
   }
   res["photoviews"] = photoviews_arr;

   json panophotoviews_arr = json::array();
   for (shared_ptr<AbstractPhotoView> pv : pvs)
   {
       PanoPhotoView *pvp = dynamic_cast<PanoPhotoView *>(pv.get());
       if (pvp != NULL)
       {
           if (center_lat >= -90 && center_lat <= 90 &&
               center_lon >= -180 && center_lon <= 180)
           {
               std::cout << "center_lat: " << center_lat << ", " << center_lon << std::endl;
               GeoPoint our_center(srs, center_lon, center_lat, 1.8, ALTMODE_RELATIVE);
               our_center.transformZ(ALTMODE_ABSOLUTE,  mapNode->getTerrainEngine()->getTerrain());
               GeoPoint our_center_ecef;
               our_center.transform(srs->getGeocentricSRS(), our_center_ecef);
               osg::Vec3d center_xyz(our_center_ecef.x(), our_center_ecef.y(), our_center_ecef.z());
               if ((pv->getECEFPosition() - center_xyz).length() > 40000)
               {
                   std::cout << "filtered out, due to length: " << (pv->getECEFPosition() - center_xyz).length() << std::endl;
                   //if more than 20km from center do not export
                   continue;
               }
           }

           json jpv;

           fs::path parent_path(root_path), child_path(pv->getPhotoName());
           jpv["filename"] = fs::relative(child_path, parent_path).string();

           json position;
           osg::Vec3d pos = ECEFToWgs84(pv->getECEFPosition());
           position.push_back((double)pos.x());
           position.push_back((double)pos.y());
           position.push_back((double)pos.z());
           jpv["position"] = position;

           json rotation;
           osg::Quat rot = pv->getOrientationLocal();
           rotation.push_back((double)rot.x());
           rotation.push_back((double)rot.y());
           rotation.push_back((double)rot.z());
           rotation.push_back((double)rot.w());
           jpv["rotation"] = rotation;

           json initRot;
           osg::Quat irot = pvp->getInitRot();
           initRot.push_back((double)irot.x());
           initRot.push_back((double)irot.y());
           initRot.push_back((double)irot.z());
           initRot.push_back((double)irot.w());

           jpv["initrot"] = initRot;
           jpv["inityaw"] = pvp->getYaw();

           //MUST BE LAST
           panophotoviews_arr.push_back(jpv);
       }
    }
    res["panophotoviews"] = panophotoviews_arr;

    //animation spline
    //speed settings
    double segment_length = 100;
    double speed = 100; //in km/h
    //inference of duration from speed and segment length
    double speed_ms = speed / 3.6;
    double duration = segment_length / speed_ms;
    double time = 0 ;

    osg::Vec3d prev_pos(-1, -1, -1);
    bool nextFrame = false;

    json flythrough_arr = json::array();
    if (anim_spline != nullptr)
    {
        anim_spline->reset();
        while (!anim_spline->finished())
        {
            osg::Vec3d pos = anim_spline->position();
            osg::Vec3d pos_wgs84 = ECEFToWgs84(pos);

            bool dist = false;
            double real_length = 0;
            if (prev_pos.x() == -1 && prev_pos.y() == -1 && prev_pos.z() == -1)
            {
                //first segment needs to be added
                dist = true;
            }
            else
            {
                real_length = (prev_pos - pos).length();
                //distance is enough for the segment, or we need to wait on photo
                dist = real_length >= segment_length || nextFrame;
            }
            //check for distance
            if (dist)
            {
                prev_pos = pos;
                duration = real_length / speed_ms;

                if (nextFrame)
                {
                    time += 2.0; //add two seconds to arrive at the photo
                    //get control Point and animate the camera precisely to its
                    //photoview
                    ControlPoint cp = anim_spline->getControlPoint();
                    shared_ptr<AbstractPhotoView> pv = cp.getPhotoView();

                    fs::path parent_path(root_path), child_path(pv->getPhotoName());

                    json keyframe;
                    keyframe["time"] = time;
                    json position;
                    osg::Vec3d pv_pos = ECEFToWgs84(pv->getECEFPosition());
                    position.push_back((double)pv_pos.x());
                    position.push_back((double)pv_pos.y());
                    position.push_back((double)pv_pos.z());
                    keyframe["position"] = position;
                    keyframe["filename"] = fs::relative(child_path, parent_path).string();
                    flythrough_arr.push_back(keyframe);
                }
                else
                {
                    time += duration;
                    //write the camera position to the json file
                    json keyframe;
                    keyframe["time"] = time;
                    json position;
                    position.push_back((double)pos_wgs84.x());
                    position.push_back((double)pos_wgs84.y());
                    position.push_back((double)pos_wgs84.z());
                    keyframe["position"] = position;
                    keyframe["filename"] = "";
                    flythrough_arr.push_back(keyframe);
                    //update animation time
                }
            }
            nextFrame = anim_spline->nextFrame();
        }
        res["flythrough"] = flythrough_arr;
    }

   //save res to file.
   //std::cout << res.dump() << std::endl;
   std::ofstream o(filename);
   o << std::setw(4) << res << std::endl;
}

void JsonExporter::writeXmlStartElement(xmlTextWriterPtr w, const char *name)
{
    int rc;
    rc = xmlTextWriterStartElement(w, BAD_CAST name);
    if (rc < 0) {
        std::cerr << "testXmlwriterFilename: Error at "
                  << "xmlTextWriterStartElement" << std::endl;
        return;
    }
}

void JsonExporter::writeXmlAttribute(xmlTextWriterPtr w, const char *name,
                                     const char *value)
{
    int rc;
    rc = xmlTextWriterWriteAttribute(w, BAD_CAST name, BAD_CAST value);
    if (rc < 0) {
        std::cerr << "testXmlwriterFilename: Error at "
                  << "xmlTextWriterWriteAttribute" << std::endl;
        return;
    }
}


void JsonExporter::writeXmlString(xmlTextWriterPtr w, const char *text)
{
    int rc;
    xmlChar *tmp;
    tmp = convertInput(text, "UTF-8");
    rc = xmlTextWriterWriteString(w, tmp);
    if (rc < 0)
    {
        std::cerr << "testXmlwriterFilename: Error at "
                  << "xmlTextWriterWriteString" << std::endl;
        return;
    }
    if (tmp != NULL)
    {
        xmlFree(tmp);
    }
}

xmlChar *JsonExporter::convertInput(const char *in, const char *encoding)
{
    xmlChar *out;
    int ret;
    int size;
    int out_size;
    int temp;
    xmlCharEncodingHandlerPtr handler;

    if (in == 0)
        return 0;

    handler = xmlFindCharEncodingHandler(encoding);

    if (!handler) {
        printf("ConvertInput: no encoding handler found for '%s'\n",
               encoding ? encoding : "");
        return 0;
    }

    size = (int) strlen(in) + 1;
    out_size = size * 2 - 1;
    out = (unsigned char *) xmlMalloc((size_t) out_size);

    if (out != 0) {
        temp = size - 1;
        ret = handler->input(out, &out_size, (const xmlChar *) in, &temp);
        if ((ret < 0) || (temp - size + 1)) {
            if (ret < 0) {
                printf("ConvertInput: conversion wasn't successful.\n");
            } else {
                printf
                    ("ConvertInput: conversion wasn't successful. converted: %i octets.\n",
                     temp);
            }

            xmlFree(out);
            out = 0;
        } else {
            out = (unsigned char *) xmlRealloc(out, out_size + 1);
            out[out_size] = 0;  /*null terminating out */
        }
    } else {
        printf("ConvertInput: no mem\n");
    }

    return out;
}


void JsonExporter::rotToGoogleEarthYPR(Quat o,
                                       double &yaw, double &pitch, double &roll)
{
    Util::toEulerAngle(o, roll, pitch, yaw);
    yaw = -yaw * 180.0 / M_PI;
    pitch = -pitch * 180.0 / M_PI;
    roll = (roll * 180.0 / M_PI + 90.0) + 90.0;
    double tmp = roll;
    roll = pitch;
    pitch = tmp;
}


void JsonExporter::saveToKML(string filename, string root_path,
                             double center_lat, double center_lon)
{
    //inspired by http://xmlsoft.org/examples/testWriter.c
    LIBXML_TEST_VERSION
    int rc;
    xmlTextWriterPtr writer;

    /* Create a new XmlWriter for uri, with no compression. */
    writer = xmlNewTextWriterFilename(filename.c_str(), 0);
    if (writer == NULL) {
        std::cerr << "testXmlwriterFilename: Error creating the xml writer"
                  << std::endl;
        return;
    }

    /* Start the document with the xml default for the version,
     * encoding ISO 8859-1 and the default for the standalone
     * declaration. */
    rc = xmlTextWriterStartDocument(writer, NULL, "UTF-8", NULL);
    if (rc < 0) {
        std::cerr << "testXmlwriterFilename: Error at "
                  << "xmlTextWriterStartDocument" << std::endl;
        return;
    }

    writeXmlStartElement(writer, "kml");

    writeXmlAttribute(writer, "xmlns", "http://www.opengis.net/kml/2.2");

    writeXmlAttribute(writer, "xmlns:gx", "http://www.google.com/kml/ext/2.2");

    writeXmlAttribute(writer, "xmlns:kml", "http://www.opengis.net/kml/2.2");

    writeXmlAttribute(writer, "xmlns:atom", "http://www.w3.org/2005/Atom");

    writeXmlStartElement(writer, "Document");

    writeXmlStartElement(writer, "name");
        writeXmlString(writer, "Geotagged photos");
    xmlTextWriterEndElement(writer);

    writeXmlStartElement(writer, "Style");
    writeXmlAttribute(writer, "id", "camera");
    writeXmlStartElement(writer, "IconStyle");
    writeXmlStartElement(writer, "Icon");
    writeXmlStartElement(writer, "href");
        writeXmlString(writer, ":/camera_mode.png");
    xmlTextWriterEndElement(writer);
    xmlTextWriterEndElement(writer);
    xmlTextWriterEndElement(writer);
    xmlTextWriterEndElement(writer);

    writeXmlStartElement(writer, "Style");
    writeXmlAttribute(writer, "id", "lineStyle");

    writeXmlStartElement(writer, "LineStyle");

    writeXmlStartElement(writer, "color");
    writeXmlString(writer, "99ffac59");
    xmlTextWriterEndElement(writer); //color

    writeXmlStartElement(writer, "width");
    writeXmlString(writer, "6");
    xmlTextWriterEndElement(writer); //width

    xmlTextWriterEndElement(writer); //LineStyle
    xmlTextWriterEndElement(writer); //Style


    fs::path parent_path(root_path);
    std::cout << "Picked photoviews size: " << pickedPvs.size() << std::endl;
    vector< shared_ptr<AbstractPhotoView> > &pvs = exportOnlyPicked ? pickedPvs : photoviews;
    vector< shared_ptr<AbstractPhotoView> >::reverse_iterator it;
    vector< shared_ptr<AbstractPhotoView> > added_pvs;
    for (it = pvs.rbegin(); it != pvs.rend(); ++it)
    {
        shared_ptr<AbstractPhotoView> &pv = *it;
        fs::path child_path(pv->getPhotoName());
        string filename = fs::relative(child_path, parent_path).string();

        if (center_lat >= -90 && center_lat <= 90 &&
            center_lon >= -180 && center_lon <= 180)
        {
            std::cout << "center_lat: " << center_lat << ", " << center_lon << std::endl;
            GeoPoint our_center(srs, center_lon, center_lat, 1.8, ALTMODE_RELATIVE);
            our_center.transformZ(ALTMODE_ABSOLUTE,  mapNode->getTerrainEngine()->getTerrain());
            GeoPoint our_center_ecef;
            our_center.transform(srs->getGeocentricSRS(), our_center_ecef);
            osg::Vec3d center_xyz(our_center_ecef.x(), our_center_ecef.y(), our_center_ecef.z());
            if ((pv->getECEFPosition() - center_xyz).length() > 20000)
            {
                std::cout << "filtered out, due to length: " << (pv->getECEFPosition() - center_xyz).length() << std::endl;
                //if more than 20km from center do not export
                continue;
            }
        }
        added_pvs.push_back(pv);

        //orientation
        Quat o = pv->getOrientationLocal();
        double yaw, pitch, roll;
        //Vec3d ypr = Util::getHPRfromQuat(o);
        //yaw = ypr.x(); pitch = ypr.y(); roll = ypr.z();
        rotToGoogleEarthYPR(o, yaw, pitch, roll);
        /*Util::toEulerAngle(o, roll, pitch, yaw);
        yaw = -yaw * 180.0 / M_PI;
        pitch = -pitch * 180.0 / M_PI;
        roll = (roll * 180.0 / M_PI + 90.0) + 90.0;
        std::cerr << "yaw: " << yaw << ", pitch: " << roll << ", roll: " << pitch <<  std::endl;
        */
        if (pitch > 120 || pitch < 60)
        {
            std::cerr << "Filtered out due to angle." << std::endl;
            //do not use, the photo is looking towards ground.
            continue;
        }

        double fov = 0.0;
        double vfov = 0.0;
        double near = 100.0;

        writeXmlStartElement(writer, "PhotoOverlay");
        writeXmlAttribute(writer, "id", filename.c_str());

        PhotoView *pvp = dynamic_cast<PhotoView *>(pv.get());
        PanoPhotoView *ppv = dynamic_cast<PanoPhotoView *>(pv.get());

        if (pvp != NULL)
        {
            std::cout << "is photoview" << std::endl;
            fov = pvp->getFOVDeg();
            vfov = (fov / (double)pvp->getImageWidth())
                           * (double)pvp->getImageHeight();
            if (pvp->getImageHeight() > pvp->getImageWidth())
            {
                vfov = fov;
                fov = (vfov / (double)pvp->getImageHeight())
                        * (double)pvp->getImageWidth();
            }
            near = 100.0;

            writeXmlStartElement(writer, "shape");
                writeXmlString(writer, "rectangle");
            xmlTextWriterEndElement(writer); //shape
        }
        if (ppv != NULL)
        {
            std::cout << "is PANOphotoview" << std::endl;
            fov = 360.0; //for sphere
            vfov = 180.0;
            near = 10.0; //radius 10m.

            yaw += ppv->getYaw(); //subtract the init yaw

            writeXmlStartElement(writer, "shape");
                writeXmlString(writer, "sphere");
            xmlTextWriterEndElement(writer); //shape
        }

        std::string left_fov = std::to_string(-fov / 2.0);
        std::string right_fov = std::to_string(fov / 2.0);
        std::string top_fov = std::to_string(vfov / 2.0);
        std::string bottom_fov = std::to_string(-vfov / 2.0);

        string yaw_s = std::to_string(yaw);
        string pitch_s = std::to_string(pitch);
        string roll_s = std::to_string(roll);

        osg::Vec3d pos = ECEFToWgs84(pv->getECEFPosition());
        string lon = std::to_string(pos.x());
        string lat = std::to_string(pos.y());
        //calculate elevation above ground
        float elev = ee->getElevation(pos.x(), pos.y());
        float above_ground = pos.z() - elev;
        above_ground = above_ground > 0 ? above_ground : 2;
        string alt = std::to_string(above_ground);

        writeXmlStartElement(writer, "name");
        writeXmlString(writer, filename.c_str());
        xmlTextWriterEndElement(writer);

        /* HIDE the photoview
         * if (anim_spline != nullptr)
        {
            writeXmlStartElement(writer, "color");
            writeXmlString(writer, "00ffffff");
            xmlTextWriterEndElement(writer);
        }*/

        writeXmlStartElement(writer, "Camera");

        writeXmlStartElement(writer, "longitude");
        writeXmlString(writer, lon.c_str());
        xmlTextWriterEndElement(writer);

        writeXmlStartElement(writer, "latitude");
        writeXmlString(writer, lat.c_str());
        xmlTextWriterEndElement(writer);

        writeXmlStartElement(writer, "altitude");
        writeXmlString(writer, alt.c_str());
        xmlTextWriterEndElement(writer);


        writeXmlStartElement(writer, "heading");
        writeXmlString(writer, yaw_s.c_str());
        xmlTextWriterEndElement(writer);

        writeXmlStartElement(writer, "tilt");
        writeXmlString(writer, pitch_s.c_str());
        xmlTextWriterEndElement(writer);

        writeXmlStartElement(writer, "roll");
        writeXmlString(writer, roll_s.c_str());
        xmlTextWriterEndElement(writer);
        xmlTextWriterEndElement(writer); //Camera

        writeXmlStartElement(writer, "styleUrl");
        writeXmlString(writer, "camera");
        xmlTextWriterEndElement(writer);

        writeXmlStartElement(writer, "Icon");
        writeXmlStartElement(writer, "href");
        writeXmlString(writer, filename.c_str());
        xmlTextWriterEndElement(writer); //href
        xmlTextWriterEndElement(writer); //Icon


        writeXmlStartElement(writer, "ViewVolume");
        writeXmlStartElement(writer, "near");
            writeXmlString(writer, std::to_string(near).c_str());
        xmlTextWriterEndElement(writer);
        writeXmlStartElement(writer, "leftFov");
            writeXmlString(writer, left_fov.c_str());
        xmlTextWriterEndElement(writer);
        writeXmlStartElement(writer, "rightFov");
            writeXmlString(writer, right_fov.c_str());
        xmlTextWriterEndElement(writer);
        writeXmlStartElement(writer, "bottomFov");
            writeXmlString(writer, bottom_fov.c_str());
        xmlTextWriterEndElement(writer);
        writeXmlStartElement(writer, "topFov");
            writeXmlString(writer, top_fov.c_str());
        xmlTextWriterEndElement(writer);
        xmlTextWriterEndElement(writer); //ViewVolume

        std::string coords = lon + "," + lat + "," + alt;
        writeXmlStartElement(writer, "Point");
        writeXmlStartElement(writer, "coordinates");
            writeXmlString(writer, coords.c_str());
        xmlTextWriterEndElement(writer); //coordinates
        xmlTextWriterEndElement(writer); //Point

        xmlTextWriterEndElement(writer); //PhotoOverlay
    }

    //export the animation path to KML
    if (anim_spline != nullptr)
    {
        writeXmlStartElement(writer, "gx:Tour");

        writeXmlStartElement(writer, "name");
        writeXmlString(writer, "Flythrough");
        xmlTextWriterEndElement(writer);

        //speed params
        double segment_length = 100;
        double speed = 100; //in km/h
        //inference of duration from speed and segment length
        double speed_ms = speed / 3.6;
        double duration = segment_length / speed_ms;

        writeXmlStartElement(writer, "gx:Playlist");

        //hide all photoviews
        for (it = added_pvs.rbegin(); it != added_pvs.rend(); ++it)
        {
            shared_ptr<AbstractPhotoView> &pv = *it;
            fs::path child_path(pv->getPhotoName());
            string filename = fs::relative(child_path, parent_path).string();

            writeXmlStartElement(writer, "gx:AnimatedUpdate");
            writeXmlStartElement(writer, "gx:duration");
            writeXmlString(writer, "0.0");
            xmlTextWriterEndElement(writer); //duration
            writeXmlStartElement(writer, "Update");
            writeXmlStartElement(writer, "targetHref");
            xmlTextWriterEndElement(writer); //targetHref
            writeXmlStartElement(writer, "Change");
            writeXmlStartElement(writer, "PhotoOverlay");
            writeXmlAttribute(writer, "targetId", filename.c_str());
            writeXmlStartElement(writer, "color");
            writeXmlString(writer, "00ffffff");
            xmlTextWriterEndElement(writer); //color
            xmlTextWriterEndElement(writer); //PhotoOverlay
            xmlTextWriterEndElement(writer); //Change
            xmlTextWriterEndElement(writer); //Update
            xmlTextWriterEndElement(writer); //gx:AnimatedUpdate
        }


        osg::Vec3d prev_pos(-1, -1, -1);
        bool nextFrame = false;
        anim_spline->reset();
        vector<osg::Vec3d> fly_coordinates;
        while (!anim_spline->finished())
        {
            osg::Vec3d pos = anim_spline->position();
            double distToCP = anim_spline->distanceToNextControlPoint();
            double distFromCP = anim_spline->distanceToPreviousControlPoint();

            bool dist = false;
            double real_length = 0;
            if (prev_pos.x() == -1 && prev_pos.y() == -1 && prev_pos.z() == -1)
            {
                //first segment needs to be added
                dist = true;
            }
            else
            {
                real_length = (prev_pos - pos).length();
                //distance is enough for the segment, or we need to wait on photo
                dist = real_length >= segment_length || nextFrame;
            }
            //check for distance
            if (dist)
            {
                prev_pos = pos;
                //duration = real_length / speed_ms;
                distToCP = std::max(real_length, distToCP);
                distFromCP = std::max(real_length, distFromCP);
                double dcp = std::min(distToCP, distFromCP);
                duration = (real_length / dcp) * 3.0; //3 seconds is minimum between photos

                if (nextFrame)
                {
                    //get control Point and animate the camera precisely to its
                    //photoview
                    ControlPoint cp = anim_spline->getControlPoint();
                    shared_ptr<AbstractPhotoView> pv = cp.getPhotoView();

                    Quat o = pv->getOrientationLocal();
                    double yaw, pitch, roll;
                    rotToGoogleEarthYPR(o, yaw, pitch, roll);

                    string yaw_s = std::to_string(yaw);
                    string pitch_s = std::to_string(pitch);
                    string roll_s = std::to_string(roll);

                    osg::Vec3d pos = ECEFToWgs84(pv->getECEFPosition());
                    string lon = std::to_string(pos.x());
                    string lat = std::to_string(pos.y());
                    //calculate elevation above ground
                    float elev = ee->getElevation(pos.x(), pos.y());
                    float above_ground = pos.z() - elev;
                    above_ground = above_ground > 0 ? above_ground : 2;
                    string alt = std::to_string(above_ground);


                    writeXmlStartElement(writer, "gx:FlyTo");

                    writeXmlStartElement(writer, "gx:duration");
                    writeXmlString(writer, "2");
                    xmlTextWriterEndElement(writer);

                    writeXmlStartElement(writer, "gx:flyToMode");
                    writeXmlString(writer, "smooth");
                    xmlTextWriterEndElement(writer);

                    writeXmlStartElement(writer, "Camera");

                    writeXmlStartElement(writer, "longitude");
                    writeXmlString(writer, lon.c_str());
                    xmlTextWriterEndElement(writer);

                    writeXmlStartElement(writer, "latitude");
                    writeXmlString(writer, lat.c_str());
                    xmlTextWriterEndElement(writer);

                    writeXmlStartElement(writer, "altitude");
                    writeXmlString(writer, alt.c_str());
                    xmlTextWriterEndElement(writer);

                    fly_coordinates.push_back(pos);

                    writeXmlStartElement(writer, "heading");
                    writeXmlString(writer, yaw_s.c_str());
                    xmlTextWriterEndElement(writer);

                    writeXmlStartElement(writer, "tilt");
                    writeXmlString(writer, pitch_s.c_str());
                    xmlTextWriterEndElement(writer);

                    writeXmlStartElement(writer, "roll");
                    writeXmlString(writer, roll_s.c_str());
                    xmlTextWriterEndElement(writer);

                    PhotoView *pvp = dynamic_cast<PhotoView *>(pv.get());
                    //PanoPhotoView *ppv = dynamic_cast<PanoPhotoView *>(pv.get());
                    double fov, vfov;
                    fov = 60;
                    vfov = 60;
                    if (pvp != NULL)
                    {
                        fov = pvp->getFOVDeg();
                        vfov = (fov / (double)pvp->getImageWidth())
                                   * (double)pvp->getImageHeight();
                        if (pvp->getImageHeight() > pvp->getImageWidth())
                        {
                            fov = vfov;
                        }
                    }
                    writeXmlStartElement(writer, "gx:horizFov");
                    writeXmlString(writer, std::to_string(fov).c_str());
                    xmlTextWriterEndElement(writer);

                    xmlTextWriterEndElement(writer); //Camera
                    xmlTextWriterEndElement(writer); //FlyTo


                    //transparency to opaque animation
                    fs::path child_path(pv->getPhotoName());
                    string filename = fs::relative(child_path, parent_path).string();

                    writeXmlStartElement(writer, "gx:AnimatedUpdate");
                    writeXmlStartElement(writer, "gx:duration");
                    writeXmlString(writer, "2.0");
                    xmlTextWriterEndElement(writer); //duration
                    writeXmlStartElement(writer, "Update");
                    writeXmlStartElement(writer, "targetHref");
                    xmlTextWriterEndElement(writer); //targetHref
                    writeXmlStartElement(writer, "Change");
                    writeXmlStartElement(writer, "PhotoOverlay");
                    writeXmlAttribute(writer, "targetId", filename.c_str());
                    writeXmlStartElement(writer, "color");
                    writeXmlString(writer, "ffffffff");
                    xmlTextWriterEndElement(writer); //color
                    xmlTextWriterEndElement(writer); //PhotoOverlay
                    xmlTextWriterEndElement(writer); //Change
                    xmlTextWriterEndElement(writer); //Update
                    xmlTextWriterEndElement(writer); //gx:AnimatedUpdate

                    //wait until the transition is done
                    writeXmlStartElement(writer, "gx:Wait");
                    writeXmlStartElement(writer, "gx:duration");
                    writeXmlString(writer, "2.0");
                    xmlTextWriterEndElement(writer); //gx:duration
                    xmlTextWriterEndElement(writer); //gx:Wait

                    //pause here
                    writeXmlStartElement(writer, "gx:TourControl");
                    writeXmlStartElement(writer, "gx:playMode");
                    writeXmlString(writer, "pause");
                    xmlTextWriterEndElement(writer); //playMode
                    xmlTextWriterEndElement(writer); //TourControl


                    writeXmlStartElement(writer, "gx:AnimatedUpdate");
                    writeXmlStartElement(writer, "gx:duration");
                    writeXmlString(writer, "2.0");
                    xmlTextWriterEndElement(writer); //duration
                    writeXmlStartElement(writer, "Update");
                    writeXmlStartElement(writer, "targetHref");
                    xmlTextWriterEndElement(writer); //targetHref
                    writeXmlStartElement(writer, "Change");
                    writeXmlStartElement(writer, "PhotoOverlay");
                    writeXmlAttribute(writer, "targetId", filename.c_str());
                    writeXmlStartElement(writer, "color");
                    writeXmlString(writer, "00ffffff");
                    xmlTextWriterEndElement(writer); //color
                    xmlTextWriterEndElement(writer); //PhotoOverlay
                    xmlTextWriterEndElement(writer); //Change
                    xmlTextWriterEndElement(writer); //Update
                    xmlTextWriterEndElement(writer); //gx:AnimatedUpdate
                }

                writeXmlStartElement(writer, "gx:FlyTo");
                if (duration == 0)
                {
                    duration = 3.0;//so that duration is not zero at the beginning
                }
                writeXmlStartElement(writer, "gx:duration");
                writeXmlString(writer, std::to_string(duration).c_str());
                xmlTextWriterEndElement(writer);

                writeXmlStartElement(writer, "gx:flyToMode");
                writeXmlString(writer, "smooth");
                xmlTextWriterEndElement(writer);

                osg::Vec3d center = anim_spline->center();
                osg::Vec3d up = anim_spline->up();
                //double fovy = anim_spline->fovy();
                //double aspect = anim_spline->aspect();

                osg::Matrixd lookAt = osg::Matrixd::lookAt(pos, center, up);
                // this is rotation in world coords
                //osg::Quat w_rot = lookAt.getRotate();

                GeoPoint gp(srs->getGeocentricSRS(), pos.x(), pos.y(), pos.z(), ALTMODE_ABSOLUTE);
                Matrixd w2l;
                gp.createWorldToLocal(w2l);
                //osg::Quat rot_w2l = w2l.getRotate();
                //this is orientation in local coordinates
                osg::Matrixd lookAtLocal = osg::Matrixd::inverse(lookAt) * w2l;
                osg::Quat o, so;
                osg::Vec3f trans, scale;
                lookAtLocal.decompose(trans, o, scale, so);
                double yaw, pitch, roll;
                rotToGoogleEarthYPR(o, yaw, pitch, roll);
                string yaw_s = std::to_string(yaw);
                string pitch_s = std::to_string(pitch-180);
                string roll_s = std::to_string(roll);

                //get position and altitude
                osg::Vec3d lpos = ECEFToWgs84(pos);
                string lon = std::to_string(lpos.x());
                string lat = std::to_string(lpos.y());
                //calculate elevation above ground
                float elev = ee->getElevation(lpos.x(), lpos.y());
                float above_ground = lpos.z() - elev;
                above_ground = above_ground > 0 ? above_ground : 2;
                string alt = std::to_string(above_ground);

                writeXmlStartElement(writer, "Camera");

                writeXmlStartElement(writer, "longitude");
                writeXmlString(writer, lon.c_str());
                xmlTextWriterEndElement(writer);

                writeXmlStartElement(writer, "latitude");
                writeXmlString(writer, lat.c_str());
                xmlTextWriterEndElement(writer);

                writeXmlStartElement(writer, "altitude");
                writeXmlString(writer, alt.c_str());
                xmlTextWriterEndElement(writer);

                fly_coordinates.push_back(lpos);

                writeXmlStartElement(writer, "heading");
                writeXmlString(writer, yaw_s.c_str());
                xmlTextWriterEndElement(writer);

                writeXmlStartElement(writer, "tilt");
                writeXmlString(writer, pitch_s.c_str());
                xmlTextWriterEndElement(writer);

                writeXmlStartElement(writer, "roll");
                writeXmlString(writer, roll_s.c_str());
                xmlTextWriterEndElement(writer);
                xmlTextWriterEndElement(writer); //Camera
                xmlTextWriterEndElement(writer); //FlyTo


            }
            nextFrame = anim_spline->nextFrame();
        }
        xmlTextWriterEndElement(writer); //gx:Playlist
        xmlTextWriterEndElement(writer); //gx:Tour

        //generate the line string
        writeXmlStartElement(writer, "Placemark");

        writeXmlStartElement(writer, "name");
        writeXmlString(writer, "fly path");
        xmlTextWriterEndElement(writer); //name

        writeXmlStartElement(writer, "styleUrl");
        writeXmlString(writer, "#lineStyle");
        xmlTextWriterEndElement(writer); //styleUrl

        writeXmlStartElement(writer, "LineString");

        writeXmlStartElement(writer, "tessellate");
        writeXmlString(writer, "1");
        xmlTextWriterEndElement(writer); //tessellate

        writeXmlStartElement(writer, "coordinates");
        for (osg::Vec3d &p : fly_coordinates)
        {
            string lon = std::to_string(p.x());
            string lat = std::to_string(p.y());
            string alt = "0";
            string coord = lon+","+lat+","+alt+" ";
            writeXmlString(writer, coord.c_str());
        }
        xmlTextWriterEndElement(writer); //coordinates
        xmlTextWriterEndElement(writer); //LineString
        xmlTextWriterEndElement(writer); //Placemark
    }

    rc = xmlTextWriterEndDocument(writer);
    if (rc < 0) {
        printf
            ("testXmlwriterFilename: Error at xmlTextWriterEndDocument\n");
        return;
    }

    xmlFreeTextWriter(writer);
}

osg::Vec3d JsonExporter::findSceneCenter()
{
    Vec3d center = Vec3d(0.0, 0.0, 0.0);
    for (shared_ptr<AbstractPhotoView> pv : photoviews)
    {
        osg::Vec3d pos = pv->getECEFPosition();
        center += pos / (double)photoviews.size();
    }
    return ECEFToWgs84OnGround(center); //project to WGS84 and move on ground
}

osg::Vec3d JsonExporter::ECEFToWgs84(osg::Vec3d pos)
{
    GeoPoint posp(srs->getGeocentricSRS(), pos.x(), pos.y(), pos.z(), ALTMODE_ABSOLUTE);
    GeoPoint wgs84pos = posp.transform(srs);

    return osg::Vec3d(wgs84pos.x(), wgs84pos.y(), wgs84pos.z());
}

osg::Vec3d JsonExporter::ECEFToWgs84OnGround(osg::Vec3d pos)
{
    GeoPoint posp(srs->getGeocentricSRS(), pos.x(), pos.y(), pos.z(), ALTMODE_ABSOLUTE);
    GeoPoint wgs84pos = posp.transform(srs);
    GeoPoint p = GeoPoint(srs, wgs84pos.x(), wgs84pos.y(),
                                 0, ALTMODE_RELATIVE);
    p.transformZ(ALTMODE_ABSOLUTE,
                        mapNode->getTerrainEngine()->getTerrain());
    return osg::Vec3d(p.x(), p.y(), p.z());
}
