/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   29.1.2018
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
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


#include "terrainsampling/equirectcamerabuilder.h"

namespace itr{

EquirectCameraBuilder::EquirectCameraBuilder(Viewer *viewer, int resolution)
{
    //set the main camera
    double fovy, aspect, znear, zfar;
    viewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspect, znear, zfar);
    viewer->getCamera()->setProjectionMatrixAsPerspective(90, 1.0,
                                                          znear, zfar);
    gc = createGraphicsContext(resolution, resolution);
    if (gc.valid())
    {
        viewer->getCamera()->setGraphicsContext(gc);
        osg::Vec3d eye, center, up;
        viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

        osg::Vec3d forward = center - eye;
        forward.normalize();
        osg::Vec3d side = forward ^ up;
        side.normalize();

        cubeMap = new TextureCubeMap;
        cubeMap->setTextureSize(resolution, resolution);
        cubeMap->setSourceFormat(GL_RGB);
        //cubeMap->setInternalFormat(GL_RGBA32F_ARB);
        //cubeMap->setSourceType(GL_FLOAT);
        cubeMap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        cubeMap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        cubeMap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
        cubeMap->setFilter(osg::TextureCubeMap::MIN_FILTER,osg::TextureCubeMap::LINEAR);
        cubeMap->setFilter(osg::TextureCubeMap::MAG_FILTER,osg::TextureCubeMap::LINEAR);



        osg::Matrixd viewM = viewer->getCamera()->getViewMatrix();
        osg::Quat rl(-M_PI/2.0, up);
        osg::Matrixd rlm(rl);
        cam_left = new RenderToImageCamera(cubeMap, TextureCubeMap::Face::POSITIVE_X);
        cam_left->setReferenceFrame(osg::Camera::RELATIVE_RF);
        cam_left->setViewport(0 * resolution, 0 * resolution, resolution, resolution);
        cam_left->setGraphicsContext(gc);
        viewer->addSlave(cam_left, osg::Matrixd(), rlm);

        //osg::Quat rf(0.0, up);
        //osg::Matrixd rfm(rf);
        cam_front = new RenderToImageCamera(cubeMap, TextureCubeMap::Face::NEGATIVE_Z);
        cam_front->setReferenceFrame(osg::Camera::RELATIVE_RF);
        cam_front->setViewport(0 * resolution, 0 * resolution, resolution, resolution);
        cam_front->setGraphicsContext(gc);
        viewer->addSlave(cam_front, osg::Matrixd(), osg::Matrixd());

        osg::Quat rr(M_PI/2.0, up);
        osg::Matrixd rrm(rr);
        cam_right = new RenderToImageCamera(cubeMap, TextureCubeMap::Face::NEGATIVE_X);
        cam_right->setReferenceFrame(osg::Camera::RELATIVE_RF);
        cam_right->setViewport(0 * resolution, 0 * resolution, resolution, resolution);
        cam_right->setGraphicsContext(gc);
        viewer->addSlave(cam_right, osg::Matrixd(), rrm);

        osg::Quat rb(M_PI, up);
        osg::Matrixd rrb(rb);
        cam_back = new RenderToImageCamera(cubeMap, TextureCubeMap::Face::POSITIVE_Z);
        cam_back->setReferenceFrame(osg::Camera::RELATIVE_RF);
        cam_back->setViewport(0 * resolution, 0 * resolution, resolution, resolution);
        cam_back->setGraphicsContext(gc);
        viewer->addSlave(cam_back, osg::Matrixd(), rrb);

        osg::Quat rup(-M_PI/2.0, side);
        osg::Matrixd rrup(rup);
        cam_up = new RenderToImageCamera(cubeMap, TextureCubeMap::Face::NEGATIVE_Y);
        cam_up->setReferenceFrame(osg::Camera::RELATIVE_RF);
        cam_up->setViewport(0 * resolution, 0 * resolution, resolution, resolution);
        cam_up->setGraphicsContext(gc);
        viewer->addSlave(cam_up, osg::Matrixd(), rrb * rrup);


        osg::Quat rdown(M_PI/2.0, side);
        osg::Matrixd rrdown(rdown);
        cam_down = new RenderToImageCamera(cubeMap, TextureCubeMap::Face::POSITIVE_Y);
        cam_down->setReferenceFrame(osg::Camera::RELATIVE_RF);
        cam_down->setViewport(0 * resolution, 0, resolution, resolution);
        cam_down->setGraphicsContext(gc);
        viewer->addSlave(cam_down, osg::Matrixd(), rrb * rrdown);


        displayTexture = new DisplayEquirectTexture(cubeMap);
        displayTexture->setViewport(0, 0, gc->getTraits()->width,
                                    gc->getTraits()->height);
        displayTexture->setGraphicsContext(gc);
        viewer->addSlave(displayTexture, osg::Matrixd(), osg::Matrixd(), false);

        wcc = new WindowCaptureCallback;
        wcc->addToViewer(*viewer);
    }
    else
    {
        osg::notify(osg::INFO) << "Unable to create graphics context." << std::endl;
    }
}

osg::ref_ptr<GC> EquirectCameraBuilder::createGraphicsContext(int width, int height)
{
    osg::ref_ptr<GC::Traits> traits = new GC::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = 4 * width;
    traits->height = 2 * height;
    traits->windowDecoration = false; // window border etc.
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    traits->vsync = false;
    traits->readDISPLAY();
    osg::ref_ptr<GC> graphicsContext = GC::createGraphicsContext(traits.get());
    return graphicsContext;
}


osg::ref_ptr<WindowCaptureCallback> EquirectCameraBuilder::getCaptureCallback()
{
    return wcc;
}

} //namespace itr
