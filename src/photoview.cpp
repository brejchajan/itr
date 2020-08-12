/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   07.08.2017
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


#include "photoview.h"

using namespace itr;

PhotoView::PhotoView(MapNode *_mapNode, string _photoName) :
    mapNode(_mapNode), view_state(SHOW_NONE), has_geo(false)
{
    srs = SpatialReference::get("epsg:4326");

    ep = mapNode->getMap()->getElevationPool();
    ee = ep->createEnvelope(srs, 16);

    photoName = _photoName;
    loadPhoto(_photoName);

    //geo transform for the photo view
    geo_transf = new GeoTransform();
    geo_transf->setTerrain(mapNode->getTerrain());
    geo_transf->setPosition(position);
    mapNode->addChild(geo_transf);

    //geo transform for the exif position
    geo_transf_ex = new GeoTransform();
    geo_transf_ex->setTerrain(mapNode->getTerrain());
    geo_transf_ex->setPosition(position_exif);
    mapNode->addChild(geo_transf_ex);

    photoplane = new Geode();
    geom = new Geometry();
    photoplane->addDrawable(geom);
    geom->setUseDisplayList(false);

    photoplane_masked = new Geode();
    geom_masked = new Geometry();
    photoplane_masked->addDrawable(geom_masked);
    geom_masked->setUseDisplayList(false);

    vert = new Vec3Array;

    //OTHERS
    vert->push_back(Vec3(-0.5, 0.5, 0));
    vert->push_back(Vec3(0.5, 0.5, 0));
    vert->push_back(Vec3(0.5, -0.5, 0));
    vert->push_back(Vec3(-0.5, -0.5, 0));

    //UPSIDE DOWN
    /*vert->push_back(Vec3(-0.5, -0.5, 0));
    vert->push_back(Vec3(0.5, -0.5, 0));
    vert->push_back(Vec3(0.5, 0.5, 0));
    vert->push_back(Vec3(-0.5, 0.5, 0));*/

    geom->setVertexArray(vert);
    geom_masked->setVertexArray(vert);

    DrawElementsUInt *plane = new DrawElementsUInt(PrimitiveSet::TRIANGLE_FAN, 0);
    plane->push_back(0);
    plane->push_back(1);
    plane->push_back(2);
    plane->push_back(3);
    geom->addPrimitiveSet(plane);
    geom_masked->addPrimitiveSet(plane);

    Vec4Array *colors = new osg::Vec4Array;
    colors->push_back(Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 0 red
    colors->push_back(Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 1 green
    colors->push_back(Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 2 blue
    colors->push_back(Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 3 white
    geom->setColorArray(colors);
    geom->setColorBinding(Geometry::BIND_PER_VERTEX);
    geom_masked->setColorArray(colors);
    geom_masked->setColorBinding(Geometry::BIND_PER_VERTEX);

    Vec2Array *texCoords = new Vec2Array;
    texCoords->push_back(Vec2f(0.0f, 0.0f));
    texCoords->push_back(Vec2f(1.0f, 0.0f));
    texCoords->push_back(Vec2f(1.0f, 1.0f));
    texCoords->push_back(Vec2f(0.0f, 1.0f));

    /*texCoords->push_back(Vec2f(0.0f, 1.0f));
    texCoords->push_back(Vec2f(1.0f, 1.0f));
    texCoords->push_back(Vec2f(1.0f, 0.0f));
    texCoords->push_back(Vec2f(0.0f, 0.0f));*/
    geom->setTexCoordArray(0, texCoords);
    geom_masked->setTexCoordArray(0, texCoords);


    //create image texture
    updatePhotoTexture();

    osgEarth::Registry::shaderGenerator().run(photoplane);
    osgEarth::Registry::shaderGenerator().run(photoplane_masked);

    const char * transparency_fragment = "\n"
        "#version 410\n"
        "uniform float transparency;\n"
        "void apply_transparency(inout vec4 color){\n"
        "   color = vec4(color.xyz, transparency);\n"
        "}";

    const char * transparency_vertex_masked = "\n"
        "#version 410\n"
        "out vec3 pos;\n"
        "void setup_transparency(inout vec4 vertex){\n"
        "   pos = gl_MultiTexCoord0.xyz;\n"
        "}";

    const char * transparency_fragment_masked = "\n"
        "#version 410\n"
        "in vec3 pos;"
        "uniform sampler2D photo_mask;\n"
        "uniform sampler2D photo;\n"
        "void apply_transparency(inout vec4 color){\n"
        "   float transparency = texture(photo_mask, pos.xy).r;\n"
        "   vec3 c = texture(photo, pos.xy).rgb;\n"
        "   color = vec4(c, transparency);\n"
        "}";


    Uniform *transparency = new Uniform("transparency", 0.7f);
    geom->getOrCreateStateSet()->addUniform(transparency);
    VirtualProgram* vp = VirtualProgram::getOrCreate(geom->getOrCreateStateSet());
    vp->setFunction( "apply_transparency", transparency_fragment, ShaderComp::LOCATION_FRAGMENT_COLORING);

    ref_ptr<Uniform> textureUniform = new Uniform;
    textureUniform->setName("photo_mask");
    textureUniform->setType(osg::Uniform::SAMPLER_2D);
    textureUniform->set(0);
    geom_masked->getOrCreateStateSet()->addUniform(textureUniform.get());
    ref_ptr<Uniform> textureUniformPhoto = new Uniform;
    textureUniformPhoto->setName("photo");
    textureUniformPhoto->setType(osg::Uniform::SAMPLER_2D);
    textureUniformPhoto->set(1);
    geom_masked->getOrCreateStateSet()->addUniform(textureUniformPhoto.get());

    VirtualProgram* vp_masked = VirtualProgram::getOrCreate(geom_masked->getOrCreateStateSet());
    vp_masked->setFunction( "setup_transparency", transparency_vertex_masked, ShaderComp::LOCATION_VERTEX_CLIP);
    vp_masked->setFunction( "apply_transparency", transparency_fragment_masked, ShaderComp::LOCATION_FRAGMENT_COLORING);

    glob_transf = new PositionAttitudeTransform;
    //set global transf to initial position
    GeoPoint position_ecef(position);
    position_ecef.transformZ(ALTMODE_ABSOLUTE,
                       mapNode->getTerrainEngine()->getTerrain());
    GeoPoint fitted_xyz = position_ecef.transform(srs->getGeocentricSRS());
    glob_transf->setPosition(osg::Vec3(fitted_xyz.x(), fitted_xyz.y(), fitted_xyz.z()));

    mapNode->addChild(glob_transf);

    transf = new PositionAttitudeTransform;
    transf->addChild(photoplane);
    transf->setScale(Vec3d(100, 100, 100));

    //create line perpendicular to image plane FIXME: for debug
    perp_line = createLine(Vec4(0.0, 0.0, 1.0, 1.0),
                           Vec3(0.0, 0.0, 0.0),
                           Vec3(0.0, 0.0, 10));

    /*ref_ptr<Geode> y_ax = createLine(Vec4(0.0, 1.0, 0.0, 1.0),
                               Vec3(0.0, 0.0, 0.0),
                               Vec3(0.0, 10, 0.0));

    ref_ptr<Geode> x_ax = createLine(Vec4(1.0, 0.0, 0.0, 1.0),
                               Vec3(0.0, 0.0, 0.0),
                               Vec3(10, 0.0, 0.0));*/

    transf->addChild(perp_line);
    //transf->addChild(y_ax);
    //transf->addChild(x_ax);

    //setOrientation(0, 0, 0);
    //geo_transf->addChild(transf);
    glob_transf->addChild(transf);

    // create location pointer
    photo_pointer = createLocationPointer(1.0, 0.0, 0.0);
    geo_transf->addChild(photo_pointer);

    photo_pointer_exif = createLocationPointer(0.0, 1.0, 0.0);
    geo_transf_ex->addChild(photo_pointer_exif);


    updateViewState();
}

void PhotoView::setOpacity(float opacity)
{
    Uniform *transparency = new Uniform("transparency", opacity);
    geom->getOrCreateStateSet()->addUniform(transparency);
}


void PhotoView::setOrientation(double yaw, double pitch, double roll)
{
    Quat orientation(pitch, Vec3(1, 0, 0),           //roll
                     roll, Vec3(0, 1, 0),            //pitch
                     yaw, Vec3(0, 0, 1)              //yaw
                     );
    setOrientation(orientation);
}

void PhotoView::setOrientation(Quat orientation)
{

    transf->setAttitude(orientation);


    GeoPoint gp = geo_transf->getPosition();

    /*
    //gp.makeGeographic();
    std::cout << "geo pos: " << gp.x() << ", " << gp.y() << ", " << gp.z() << ", relative: " << gp.isRelative() << std::endl;
    Matrixd l2w;
    gp.createLocalToWorld(l2w);
    osg::Quat rot_l2w = l2w.getRotate();
    Matrixd id;
    id.makeIdentity();

    Quat ori(M_PI/2.0, Vec3(1, 0, 0),           //roll
                     M_PI/2.0, Vec3(0, 1, 0),            //pitch
                     0.0, Vec3(0, 0, 1));              //yaw

    transf->setAttitude(ori * rot_l2w);*/

    //t = height, s = width, we need to use the longer side as width
    double t = static_cast<double>(photoImg->t());
    double s = static_cast<double>(photoImg->s());
    double tmp = t;
    bool flip = t > s;
    t = flip ? s : t;
    s = flip ? tmp : s;
    double ratio = t / s;

    //offset the photo in viewing direction by d and calculate width and height
    double w = 2 * tan(fov/2.0) * PHOTOPLANE_OFFSET;
    double h = w * ratio;
    tmp = w;
    w = flip ? h : w;
    h = flip ? tmp : h;
    Vec3 newpos = orientation * Vec3(0.0, 0.0, 1.0) * PHOTOPLANE_OFFSET;
    transf->setPosition(newpos);
    transf->setScale(Vec3d(w, h, 1));
}

void PhotoView::setFOV(double _fov)
{
    fov = _fov;
    //update transformation node to reflect new field of view.
    setOrientation(transf->getAttitude());
}

Geode *PhotoView::createLine(Vec4 color, Vec3 start, Vec3 end)
{
    Geode *line = new Geode();
    Geometry *pp_geom = new Geometry();
    line->addDrawable(pp_geom);

    Vec3Array *pp_vert = new Vec3Array;
    pp_vert->push_back(start);
    pp_vert->push_back(end);
    pp_geom->setVertexArray(pp_vert);

    DrawElementsUInt *pp_line = new DrawElementsUInt(PrimitiveSet::LINES, 0);
    pp_line->push_back(1);
    pp_line->push_back(0);
    pp_geom->addPrimitiveSet(pp_line);

    Vec4Array *pp_colors = new Vec4Array;
    pp_colors->push_back(color); //index 0 red
    pp_colors->push_back(color); //index 1 red
    pp_geom->setColorArray(pp_colors);
    pp_geom->setColorBinding(Geometry::BIND_PER_VERTEX);
    osgEarth::Registry::shaderGenerator().run(line);

    return line;
}

Geode *PhotoView::createLocationPointer(float red, float green, float blue)
{
    Vec3 start(0, 0, 0);
    Vec3 end(0, 0, 1000);
    Vec4 pointer_color(red, green, blue, 1.0f);
    return createLine(pointer_color, start, end);
}

bool PhotoView::isAboveGround()
{
    GeoPoint wgs84pos = position.transform(srs);
    float elev = ee->getElevation(wgs84pos.x(), wgs84pos.y());
    GeoPoint fit_pos(srs, wgs84pos.x(), wgs84pos.y(), static_cast<double>(elev), ALTMODE_ABSOLUTE);
    return wgs84pos.z() > fit_pos.z();
}

float PhotoView::distanceAboveGround()
{
    GeoPoint wgs84pos = position.transform(srs);
    float elev = ee->getElevation(wgs84pos.x(), wgs84pos.y());
    GeoPoint fit_pos(srs, wgs84pos.x(), wgs84pos.y(), static_cast<double>(elev), ALTMODE_ABSOLUTE);
    std::cout << "dag wgs84pos: " << wgs84pos.x() << ", " << wgs84pos.y() << ", " << wgs84pos.z() << ", elev: " << elev << std::endl;
    return wgs84pos.z() - fit_pos.z();
}


void PhotoView::fitPositionToTerrain()
{
    GeoPoint wgs84pos = position.transform(srs);

    float elev = ee->getElevation(wgs84pos.x(), wgs84pos.y()) + 2.0f;
    std::cout << "Fitting to terrain, elev: " << elev << std::endl;
    GeoPoint fit_pos(srs, wgs84pos.x(), wgs84pos.y(), static_cast<double>(elev), ALTMODE_ABSOLUTE);
    /*fit_pos.transformZ(ALTMODE_ABSOLUTE,
                       mapNode->getTerrainEngine()->getTerrain());*/
    //fit_pos.z() += 35.0;
    GeoPoint fitted_xyz = fit_pos.transform(srs->getGeocentricSRS());

    /*if ((wgs84pos.z() < fit_pos.z()) && (wgs84pos.z() > fit_pos.z() - min_t))
    {*/
    //we are under terrain
    setECEFPosition(fitted_xyz.x(), fitted_xyz.y(), fitted_xyz.z());
    //}

}

void PhotoView::setECEFPosition(double x, double y, double z)
{
    const SpatialReference *ecefSRS = srs->getGeocentricSRS();
    position = GeoPoint(ecefSRS, x, y, z);
    geo_transf->setPosition(position);
    glob_transf->setPosition(osg::Vec3d(x, y, z));

    /*GeoPoint wgs84pos = position.transform(srs);
    //FIXME: hack fitting position to groud
    GeoPoint fit_pos(srs, wgs84pos.x(), wgs84pos.y(), 1.8, ALTMODE_RELATIVE);
    geo_transf->setPosition(fit_pos);
    GeoPoint fitted_xyz = fit_pos.transform(srs->getGeocentricSRS());
    glob_transf->setPosition(osg::Vec3d(fitted_xyz.x(), fitted_xyz.y(), fitted_xyz.z()));*/
}

void PhotoView::setWGS84Position(double latitude, double longitude)
{
    double elev = static_cast<double>(ee->getElevation(longitude, latitude));
    std::cout << "found elevation: " << elev << std::endl;
    GeoPoint pos = GeoPoint(srs, longitude, latitude, elev, ALTMODE_ABSOLUTE);
    position = pos.transform(srs->getGeocentricSRS());
    geo_transf->setPosition(position);
    glob_transf->setPosition(osg::Vec3d(position.x(), position.y(), position.z()));
}


void PhotoView::setViewState(PhotoView::ViewState state)
{
    view_state = state;
    updateViewState();
}

void PhotoView::toggleViewState()
{
    view_state = static_cast<PhotoView::ViewState>((view_state + 1) %
                                                   VIEW_STATE_LAST);
    updateViewState();
}

void PhotoView::updateViewState()
{
    geo_transf->removeChild(photo_pointer);
    geo_transf_ex->removeChild(photo_pointer_exif);
    transf->removeChild(perp_line);

    switch (view_state) {
    case SHOW_ALL:
    {
        geo_transf->addChild(photo_pointer);
        geo_transf_ex->addChild(photo_pointer_exif);
        break;
    }
    case SHOW_EXIF_POS:
    {
        geo_transf_ex->addChild(photo_pointer_exif);
        break;
    }
    case SHOW_SFM_POS:
    {
        geo_transf->addChild(photo_pointer);
        break;
    }
    case SHOW_SFM_POS_AND_DIR:
    {
        geo_transf->addChild(photo_pointer);
        transf->addChild(perp_line);
    }
    case SHOW_NONE:
        break;
    default:
        break;
    }
}

void PhotoView::loadFullResolutionPhoto()
{
    std::cout << "Loading FULLRESOLUTION IMAGE: " << photoName << std::endl;
    photoImg = osgDB::readImageFile(photoName);
    if (!photoImg.valid())
    {
        std::cout << "Unable to load image file: " << photoName << std::endl;
        throw std::runtime_error("Unable to load image file: " + photoName);
    }
    std::cout << "Loading FULLRESOLUTION IMAGE LOADED" << std::endl;

    //calculate new images size
    int w = photoImg->s();
    int h = photoImg->t();

    int width = min(w, IMAGE_WIDTH_LARGE);

    if (w >= h)
    {
        double r = (double)h/(double)w;
        w = width;
        h = r * w;
    }
    else
    {
        double r = (double)w/(double)h;
        h = width;
        w = r * h;
    }
    std::cout << "Scaling image..." << std::endl;
    photoImg->scaleImage(w, h, 1);
    std::cout << "Image scaled." << std::endl;

    loadPhotoMask(w, h);
}

void PhotoView::loadPhotoMask(int w, int h)
{
    // load photo mask if available
    fs::path photoNamePath = fs::path(photoName);
    string ext = photoNamePath.extension().string();
    string photoNameMask = photoNamePath.stem().string() + "_mask" + ext;
    fs::path photoNameMaskPath = photoNamePath.parent_path();
    photoNameMaskPath = photoNameMaskPath / fs::path(photoNameMask);
    std::cout << "Loading PHOTO MASK: " << photoNameMaskPath.string() << std::endl;

    photoMask = osgDB::readImageFile(photoNameMaskPath.string());

    /*if (!photoMask.valid())
    {
        std::cout << "Unable to load image mask file: " << photoNameMaskPath.string() << std::endl;
        throw std::runtime_error("Unable to load image file: " + photoNameMaskPath.string());
    }*/
    if (photoMask.valid())
    {
    	photoMask->scaleImage(w, h, 1);
    }
}

void PhotoView::loadSmallResolutionPhoto()
{
    std::cout << "Loading SMALLRESOLUTION IMAGE" << std::endl;
    if (!photoImg.valid())
    {
        photoImg = osgDB::readImageFile(photoName);
        if (!photoImg.valid())
        {
            throw std::runtime_error("Unable to load image file: " + photoName);
        }
    }
    //else no need to load from file

    //calculate new images size
    int w = photoImg->s();
    int h = photoImg->t();
    if (w >= h)
    {
        double r = (double)h/(double)w;
        w = IMAGE_WIDTH;
        h = r * w;
    }
    else
    {
        double r = (double)w/(double)h;
        h = IMAGE_WIDTH;
        w = r * h;
    }
    photoImg->scaleImage(w, h, 1);

    loadPhotoMask(w, h);
}

void PhotoView::updatePhotoTexture(bool overlay)
{
    //update the texture
    osg::ref_ptr<Texture2D> pTex = new Texture2D;
    pTex->setImage(photoImg);
    StateSet *pStateSet = geom->getOrCreateStateSet();
    pStateSet->setMode(GL_LIGHTING, StateAttribute::OFF);
    pStateSet->setTextureAttributeAndModes(0, pTex, StateAttribute::ON);

    if (overlay)
    {
        pStateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
        pStateSet->setMode(GL_BLEND, osg::StateAttribute::OFF);
        pStateSet->setRenderingHint(StateSet::OPAQUE_BIN);
	if (photoMask.valid())
	{
	    osg::ref_ptr<Texture2D> pTex_masked = new Texture2D;
	    pTex_masked->setImage(photoMask);
	    StateSet *pStateSetMask = geom_masked->getOrCreateStateSet();
	    pStateSetMask->setMode(GL_LIGHTING, StateAttribute::OFF);
	    pStateSetMask->setTextureAttributeAndModes(0, pTex_masked, StateAttribute::ON);
	    pStateSetMask->setTextureAttributeAndModes(1, pTex, StateAttribute::ON);
	    pStateSetMask->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	    pStateSetMask->setMode(GL_BLEND, osg::StateAttribute::ON);
	    pStateSetMask->setRenderingHint(StateSet::TRANSPARENT_BIN);
	    geom_masked->setStateSet(pStateSetMask);
	    if (transf.valid())
	    {
	        transf->addChild(photoplane_masked);
	    }
	}
    }
    else
    {
        pStateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
        pStateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
        pStateSet->setRenderingHint(StateSet::TRANSPARENT_BIN);
        if (transf.valid())
        {
            transf->removeChild(photoplane_masked);
        }
    }
    geom->setStateSet(pStateSet);
}



void PhotoView::loadPhoto(string photoName)
{
    loadSmallResolutionPhoto();

    photo_yaw = 0; photo_pitch = 0; photo_roll = 0;
    double latitude, longitude = -361;
    fov = -1;

    //use easy exif instead of exiftool
    std::unique_ptr<Exif> exif_io(new Exif(photoName));
    double lat = -180.0;
    double lon = -360.0;
    exif_io->GPSLatitude(&lat);
    exif_io->GPSLongitude(&lon);

    struct std::tm tm = {0};
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

    std::cout << "Create Date Original: " << photoName << ", " << dto.c_str() << ", date: " << createDate << std::endl;
    has_geo = false;
    if (lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180)
    {
        latitude = lat;
        longitude = lon;
        has_geo = true;
    }

    std::cout << "lat: " << latitude << ", lon: " << longitude << std::endl;

    float elev = ee->getElevation(longitude, latitude) + 1.8;
    position = GeoPoint(srs, longitude, latitude, elev, ALTMODE_ABSOLUTE);
    position_exif = position;
}

Geode * PhotoView::getGeode()
{
    return photoplane;
}


ref_ptr<PositionAttitudeTransform> PhotoView::getTransf()
{
    return glob_transf;
}

ref_ptr<GeoTransform> PhotoView::getGeoTransf()
{
    return geo_transf;
}

time_t PhotoView::getCreateDate()
{
    return createDate;
}

osg::Vec3d PhotoView::getECEFPosition()
{
    return glob_transf->getPosition();
}

osg::Vec3d PhotoView::getECEFPositionElevated(double height)
{
    GeoPoint p = geo_transf->getPosition().transform(srs);
    p.z() += height;
    GeoPoint tp = p.transform(srs->getGeocentricSRS());
    osg::Vec3 res(tp.x(), tp.y(), tp.z());
    return res;
}

osg::Vec3d PhotoView::getCenterInWorldCoords()
{
    Vec3d center(0, 0, 0); //center in local coordinates
    Matrix l2r;
    transf->computeLocalToWorldMatrix(l2r, NULL);
    Matrix r2w;
    glob_transf->computeLocalToWorldMatrix(r2w, NULL);
    return center * l2r * r2w;
}

osg::Quat PhotoView::orientation()
{
    return transf->getAttitude();
}

void PhotoView::setFOVYDeg(double fovy)
{
    double w = static_cast<double>(photoImg->s());
    double h = static_cast<double>(photoImg->t());
    double fovy_rad = (fovy / 180.0) * M_PI;
    double fov_rad = (fovy_rad / h) * w;
    if (w < h)
    {
        fov_rad = fovy_rad;
    }
    setFOV(fov_rad);
}


double PhotoView::getFOVYDeg()
{
    double w = (double)photoImg->s();
    double h = (double)photoImg->t();
    if (w < h)
    {
        return getFOVDeg();
    }
    return (getFOVDeg() / w) * h;
}

double PhotoView::getFOVDeg()
{
    return (fov * 180.0) / M_PI;
}

double PhotoView::getFOV()
{
    return fov;
}

double PhotoView::getAspect()
{
    double w = (double)photoImg->s();
    double h = (double)photoImg->t();
    double tmp = w;
    if (w < h)
    {
        w = h;
        h = tmp;
    }
    return w / h;
}

void PhotoView::remove()
{
    if(mapNode->removeChild(geo_transf_ex))
    {
        std::cerr << "Unable to remove geo transf ex of : "
                  << photoName << std::endl;
    }
    if (mapNode->removeChild(geo_transf))
    {
        std::cerr << "Unable to remove geo transf of : "
                  << photoName << std::endl;
    }
    if (!mapNode->removeChild(glob_transf))
    {
        std::cerr << "Unable to remove glob transf of : "
                  << photoName << std::endl;
    }

}

string PhotoView::getPhotoName() const
{
    return photoName;
}



osg::Quat PhotoView::getOrientationLocal()
{
    GeoPoint gp = geo_transf->getPosition();
    Matrixd w2l;
    gp.createWorldToLocal(w2l);
    osg::Quat rot_w2l = w2l.getRotate();

    return orientation() * rot_w2l;
}

osg::Quat PhotoView::getOrientation()
{
    return orientation();
}

bool PhotoView::hasGeoExif()
{
    return has_geo;
}

osg::Vec3d PhotoView::getGeoExif()
{
    GeoPoint gp(position_exif);
    gp.transformZ(ALTMODE_ABSOLUTE, mapNode->getTerrainEngine()->getTerrain());
    GeoPoint xyz = gp.transform(srs->getGeocentricSRS());
    return osg::Vec3d(xyz.x(), xyz.y(), xyz.z());
}

int PhotoView::getImageWidth()
{
    return photoImg->s();
}


int PhotoView::getImageHeight()
{
    return photoImg->t();
}

bool PhotoView::isPortrait()
{
    return getImageHeight() > getImageWidth();
}
