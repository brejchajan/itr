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


#ifndef EquirectCameraBuilder_H
#define EquirectCameraBuilder_H

//local headers
#include "terrainsampling/windowcapturecallback.h"
#include "terrainsampling/rendertoimagecamera.h"
#include "terrainsampling/displayequirecttexture.h"

//OSG headers
#include <osg/ref_ptr>
#include <osgViewer/Viewer>
#include <osg/GraphicsContext>
#include <osgDB/WriteFile>
#include <osg/Image>
#include <osg/Texture2D>
#include <osg/Camera>

//stl headers
#include <memory>
#include <string>

using namespace osgViewer;
using namespace std;
using namespace osg;

namespace itr{

typedef osg::GraphicsContext GC;

/**
 * @brief The EquirectCameraBuilder class
 * Manages 6 slave cameras which render 6 faces of a cube.
 */
class EquirectCameraBuilder{
public:
    /**
     * @brief EquirectCameraBuilder
     * @param viewer        osgViewer to attach the camera to.
     * @param resolution    size of a single camera rectangular
     *                      framebuffer in pixels.
     * @return true if the image was saved sucessfuly, false otherwise.
     */
    EquirectCameraBuilder(Viewer *viewer, int resolution);

    /**
     * @brief getCaptureCallback
     * @return returns the capture callback.
     */
    osg::ref_ptr<WindowCaptureCallback> getCaptureCallback();

private:

    osg::ref_ptr<WindowCaptureCallback> wcc;

    //graphics context
    osg::ref_ptr<GC> gc;

    //Six camereras for rendering the cubemap
    ref_ptr<RenderToImageCamera> cam_front;
    ref_ptr<RenderToImageCamera> cam_left;
    ref_ptr<RenderToImageCamera> cam_right;
    ref_ptr<RenderToImageCamera> cam_back;
    ref_ptr<RenderToImageCamera> cam_up;
    ref_ptr<RenderToImageCamera> cam_down;

    ref_ptr<DisplayEquirectTexture> displayTexture;

    osg::ref_ptr<osg::Image> screen_left;
    osg::ref_ptr<osg::Image> screen_front;

    TextureCubeMap * cubeMap;

    /**
     * @brief createGraphicsContext
     * Factory that creates graphics context for a camera.
     * @param width     width of the framebuffer
     * @param height    height of the framebuffer
     * @return new graphics context
     */
    static osg::ref_ptr<GC> createGraphicsContext(int width, int height);

    void addCallbackToViewer(osgViewer::ViewerBase& viewer,
                             WindowCaptureCallback* callback);
};

}
#endif // EquirectCameraBuilder_H
