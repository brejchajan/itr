/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   13.09.2017
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
* @Project: ImmersiveTripReports 2017-2018
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


#include "panophotoview.h"

using namespace itr;

PanoPhotoView::PanoPhotoView(MapNode *mapNode, const string &_photoName, int yaw)
    :mapNode(mapNode), yaw(yaw)
{
    photoName = _photoName;
    srs = SpatialReference::get("epsg:4326");
    loadPhoto(_photoName);
    if (!photoImg)
    {
        std::cerr << "Unable to load panorama photo: " << photoName << std::endl;
    }
    std::cout << "pano img: " << photoName << ", yaw: " << yaw << std::endl;

    sphere = new Geode;
    Sphere *sp = new Sphere(osg::Vec3d(0.0, 0.0, 0.0), 1.0f);
    geom = new ShapeDrawable(sp);
    sphere->addDrawable(geom);

    osg::Texture2D *texture = new Texture2D;
    texture->setDataVariance(Object::DYNAMIC);
    texture->setFilter(Texture::MIN_FILTER, Texture::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(Texture::MAG_FILTER, Texture::LINEAR);
    texture->setWrap(Texture::WRAP_S, Texture::CLAMP);
    texture->setWrap(Texture::WRAP_T, Texture::CLAMP);
    texture->setImage(photoImg);

    ref_ptr<StateSet> ss = sphere->getOrCreateStateSet();
    ss->setTextureAttributeAndModes(0, texture, StateAttribute::ON);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(StateSet::TRANSPARENT_BIN);

    Matrixf initRotM;
    initRotQuat = Quat((90.0 / 180.0) * M_PI, Vec3(1.0, 0.0, 0.0),
                             0, Vec3(0.0, 1.0, 0.0),
                             ((-yaw + 90) / 180.0) * M_PI, Vec3(0.0, 0.0, 1.0));
    // yaw+90 so that center of panorama equals 0°.
    initRotQuat.get(initRotM);

    transf = new PositionAttitudeTransform;
    Matrixf id = Matrixf::identity();
    Quat idq;
    idq = id.getRotate();
    transf->setAttitude(idq);
    transf->setScale(osg::Vec3d(SPHERE_SIZE, SPHERE_SIZE, SPHERE_SIZE));
    transf->addChild(sphere);
    glob_transf = new PositionAttitudeTransform;
    glob_transf->addChild(transf);
    mapNode->addChild(glob_transf);

    osgEarth::Registry::shaderGenerator().run(sphere);

    const char * tex_pos_vertex = "\n"
        "#version 410\n"
        "out vec3 pos;\n"
        "uniform mat4 initRot;\n"
        "void tex_pos(inout vec4 vertex){\n"
        "   pos = (initRot * vertex).xyz;\n"
        "   vertex = vertex;\n"
        "}";

    const char * texture_fragment = "\n"
        "#version 410\n"
        "#define M_PI 3.1415926535897932384626433832795\n"
        "in vec3 pos;\n"
        "uniform sampler2D photoEquirect;\n"
        "uniform vec2 texSize;"
        "float inZeroPi(float x){\n"
        "   while(x > M_PI){\n"
        "       x = x - M_PI;\n"
        "   }\n"
        "   return x;\n"
        "}\n"
        "void apply_texture(inout vec4 color){\n"
        "   float r = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);\n"
        "   float theta = atan(pos.y, pos.x) + M_PI;\n"
        "   float phi = acos(pos.z / r);\n"
        "   vec2 normalized = vec2(theta / (2.0f * M_PI), phi / M_PI);\n"
        "   vec2 sphere_coord = vec2(normalized.x, normalized.y);\n"
        "   color = vec4(texture(photoEquirect, sphere_coord).xyz, 1.0);\n"
        "}";

    const char * transparency_fragment = "\n"
        "#version 410\n"
        "uniform float transparency;\n"
        "void apply_transparency(inout vec4 color){\n"
        "   color = vec4(color.xyz, transparency);\n"
        "}";


    Uniform *transparency = new Uniform("transparency", 0.7f);
    geom->getOrCreateStateSet()->addUniform(transparency);
    ref_ptr<Uniform> textureUniform = new Uniform;
    textureUniform->setName("photoEquirect");
    textureUniform->setType(osg::Uniform::SAMPLER_2D);
    textureUniform->set(0);
    geom->getOrCreateStateSet()->addUniform(textureUniform.get());

    Uniform *initRotUniform = new Uniform("initRot", initRotM);
    geom->getOrCreateStateSet()->addUniform(initRotUniform);

    VirtualProgram* vp = VirtualProgram::getOrCreate(geom->getOrCreateStateSet());
    vp->setFunction( "tex_pos", tex_pos_vertex, ShaderComp::LOCATION_VERTEX_MODEL);
    vp->setFunction( "apply_texture", texture_fragment, ShaderComp::LOCATION_FRAGMENT_COLORING);
    vp->setFunction( "apply_transparency", transparency_fragment, ShaderComp::LOCATION_FRAGMENT_LIGHTING);
}

Geode * PanoPhotoView::getGeode()
{
    return sphere;
}

void PanoPhotoView::setOrientation(double yaw, double pitch, double roll)
{
    Quat orientation(pitch, Vec3(1, 0, 0),           //roll
                     roll, Vec3(0, 1, 0),            //pitch
                     yaw, Vec3(0, 0, 1)              //yaw
                     );
    setOrientation(orientation);
}


void PanoPhotoView::setOrientation(Quat orientation)
{
    Matrixf id = Matrixf::identity();

    osg::Vec3d p = glob_transf->getPosition();
    GeoPoint gp(srs->getGeocentricSRS(), p.x(), p.y(), p.z(), ALTMODE_ABSOLUTE);
    Matrixd l2w;
    gp.createLocalToWorld(l2w);
    osg::Quat rot_l2w = l2w.getRotate();

    transf->setAttitude(orientation);//id.getRotate() * rot_l2w
}

void PanoPhotoView::setECEFPosition(double x, double y, double z)
{
    glob_transf->setPosition(osg::Vec3d(x, y, z));
}


void PanoPhotoView::setWGS84Position(double latitude, double longitude)
{
    GeoPoint position = GeoPoint(srs, longitude, latitude,
                                 1.8, ALTMODE_RELATIVE);
    position.transformZ(ALTMODE_ABSOLUTE,
                        mapNode->getTerrainEngine()->getTerrain());
    GeoPoint tp = position.transform(srs->getGeocentricSRS());
    setECEFPosition(tp.x(), tp.y(), tp.z());
}


void PanoPhotoView::setViewState(AbstractPhotoView::ViewState state)
{
    //TODO: implement this
}


void PanoPhotoView::toggleViewState()
{
    //TODO: implement this
}


ref_ptr<PositionAttitudeTransform> PanoPhotoView::getTransf()
{
    return glob_transf;
}

ref_ptr<GeoTransform> PanoPhotoView::getGeoTransf()
{
    //FIXME: do we need this? Does this implementation cause problems?
    ref_ptr<GeoTransform> gt = new GeoTransform;
    osg::Vec3d pos = glob_transf->getPosition();
    GeoPoint p(srs->getGeocentricSRS(), pos.x(), pos.y(), pos.z());
    gt->setPosition(p);
    return gt;
}


time_t PanoPhotoView::getCreateDate()
{
    return createDate;
}
osg::Vec3d PanoPhotoView::getECEFPosition()
{
    return glob_transf->getPosition();
}

osg::Vec3d PanoPhotoView::getCenterInWorldCoords()
{
    return getECEFPosition() +
            (orientation() * Vec3d(0.0, 0.0, 1.0) * SPHERE_SIZE);
}

osg::Vec3d PanoPhotoView::getECEFPositionElevated(double height)
{
    GeoPoint p = getGeoTransf()->getPosition().transform(srs);
    p.z() += height;
    GeoPoint tp = p.transform(srs->getGeocentricSRS());
    osg::Vec3 res(tp.x(), tp.y(), tp.z());
    std::cout << "pano geo transf: " << res.x() << ", " << res.y() << ", " << res.z() << std::endl;
    return res;
}

osg::Quat PanoPhotoView::orientation()
{
    return transf->getAttitude();
}


double PanoPhotoView::getAspect()
{
    return 1; // returns aspect for square part of the pano
}


double PanoPhotoView::getFOVDeg()
{
    //is for visualization
    return 60.0;
}

double PanoPhotoView::getFOVYDeg()
{
    //is for visualization
    return 60.0;
}

void PanoPhotoView::setFOV(double fov)
{
    //does not make sense for pano.
}

void PanoPhotoView::setOpacity(float opacity)
{
    Uniform *transparency = new Uniform("transparency", opacity);
    geom->getOrCreateStateSet()->addUniform(transparency);
}


void PanoPhotoView::fitPositionToTerrain()
{
    GeoPoint wgs84pos = getGeoTransf()->getPosition().transform(srs);
    GeoPoint fit_pos(srs, wgs84pos.x(), wgs84pos.y(), 2 * SPHERE_SIZE, ALTMODE_RELATIVE);
    fit_pos.transformZ(ALTMODE_ABSOLUTE,
                       mapNode->getTerrainEngine()->getTerrain());
    GeoPoint fitted_xyz = fit_pos.transform(srs->getGeocentricSRS());
    setECEFPosition(fitted_xyz.x(), fitted_xyz.y(), fitted_xyz.z());
}

void PanoPhotoView::hide()
{
    //TODO: implement this
}

void PanoPhotoView::loadPhoto(string photoName)
{
    photoImg = osgDB::readImageFile(photoName);

    //use easy exif instead of exiftool
    std::unique_ptr<Exif> exif_io(new Exif(photoName));
    double lat, lon;
    exif_io->GPSLatitude(&lat);
    exif_io->GPSLongitude(&lon);

    struct std::tm tm;
    string dto = exif_io->getDateTimeOriginal();
    strptime(dto.c_str(), "%Y:%m:%d %H:%M:%S", &tm);
    createDate = mktime(&tm);
    if (createDate < 0)
    {
        //std::cout << "Getting datetimedigitzed: " << dto.c_str() << std::endl;
        dto = exif_io->getDateTimeDigitized();
        strptime(dto.c_str(), "%Y:%m:%d %H:%M:%S", &tm);
        createDate = mktime(&tm);
        if (createDate < 0)
        {
            dto = exif_io->getDateTime();
            //std::cout << "Getting datetime: " << dto.c_str() << std::endl;
            strptime(dto.c_str(), "%Y:%m:%d %H:%M:%S", &tm);
            createDate = mktime(&tm);
        }
    }
}

void PanoPhotoView::remove()
{
    mapNode->removeChild(glob_transf);
}

string PanoPhotoView::getPhotoName() const
{
    return photoName;
}



osg::Quat PanoPhotoView::getOrientationLocal()
{
    osg::Vec3d p = glob_transf->getPosition();
    GeoPoint gp(srs->getGeocentricSRS(), p.x(), p.y(), p.z(), ALTMODE_ABSOLUTE);
    Matrixd w2l;
    gp.createWorldToLocal(w2l);
    osg::Quat rot_w2l = w2l.getRotate();

    return orientation() * rot_w2l;
}


double PanoPhotoView::getFOV()
{
    return (getFOVDeg() / 180.0) * M_PI;
}


osg::Quat PanoPhotoView::getInitRot()
{
    return initRotQuat;
}

double PanoPhotoView::getYaw()
{
    return (double)yaw;
}

bool PanoPhotoView::hasGeoExif()
{
    return false;
}

osg::Vec3d PanoPhotoView::getGeoExif()
{
    return osg::Vec3d(0.0, 0.0, 0.0);
}


bool PanoPhotoView::isPortrait()
{
    return false;
}

bool PanoPhotoView::isAboveGround()
{
    //FIXME: implement this method
    return true;
}
float PanoPhotoView::distanceAboveGround()
{
    //FIXME: implement this method
    return 0;
}
