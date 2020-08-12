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


#ifndef WINDOWCAPTURECALLBACK_H
#define WINDOWCAPTURECALLBACK_H

#include <osg/Camera>
#include <osg/RenderInfo>
#include <osg/ref_ptr>
#include <osg/Image>
#include <osg/GraphicsContext>
#include <osgViewer/Viewer>
#include <osgDB/WriteFile>

#include <string>
#include <fstream>
#include <sstream>
#include <mutex>

#include <boost/filesystem.hpp>

#include "util.h"

using namespace osg;
using namespace std;

namespace itr{

class WindowCaptureCallback : public osg::Camera::DrawCallback{
private:
    mutable bool save;
    string filename;
    int width;
    int height;
    mutable bool sizeSet;

    mutable bool save_depth;
    /// near and far planes to recalculate the depth from
    /// OpenGL depth buffer to world units
    double znear;
    double zfar;

    mutable std::mutex m;

    //the camera registered for postDrawCallback using addToViewer method
    osg::Camera* camera;

    virtual void saveDepth(int w, int h) const
    {
        osg::ref_ptr<osg::Image> image = new osg::Image;
        image->readPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT);
        float *data = reinterpret_cast<float*>(image->data());

        if (data != nullptr)
        {
            boost::filesystem::path photoPath(filename);
            boost::filesystem::path photo_base = photoPath.parent_path() / photoPath.stem();
            string depth_image = photo_base.string() + "_depth.txt.gz";
            ostringstream out;


            for (int j = h - 1; j >= 0; --j)
            {
                for (int i = 0; i < w; ++i)
                {
                    float zn = 2.0f * data[j * w + i] - 1.0f;
                    double ze = 2.0 * znear * zfar / (zfar + znear - static_cast<double>(zn) * (zfar - znear));
                    out << ze << " ";
                }
                out << std::endl;
            }
            string out_str = out.str();
            out.str("");
            out.clear();
            string out_gz = Util::gz_compress(out_str);
            out_str = "";
            fstream f(depth_image, f.out);
            f << out_gz;
            f.close();

        }
        data = nullptr;
    }
public:
    WindowCaptureCallback()
    {
        save = false;
        sizeSet = false;
        camera = nullptr;
        save_depth = false;
    }

    /**
     * @brief setSize
     * Set the size of the output image for next callback. You need to reset
     * it again before the next callback, as it will be cleared after the
     * callback.
     * @param width
     * @param height
     */
    virtual void setSize(int _width, int _height)
    {
        sizeSet = true;
        width = _width;
        height = _height;
    }

    virtual void operator () (osg::RenderInfo& renderInfo) const
    {
        if (save)
        {
            std::cout << "Trying to save the image..." << std::endl;
            const GraphicsContext::Traits *t = renderInfo.getState()->getGraphicsContext()->getTraits();
            if (renderInfo.getState()->getGraphicsContext()->makeCurrent())
            {
                osg::ref_ptr<osg::Image> image = new osg::Image;
                int w, h;
                if (sizeSet)
                {
                    w = width;
                    h = height;
                    sizeSet = false;
                }
                else
                {
                    w = t->width;
                    h = t->height;
                }

                std::cout << "Trying to read image pixels to: " << filename << std::endl;
                image->readPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE);
                if (image != nullptr)
                {
                    osgDB::writeImageFile(*image, filename);
                    save = false;
                }
                else
                {
                    std::cout << "Unable to read image pixels." << std::endl;
                }

                if (save_depth)
                {
                    saveDepth(w, h);
                    save_depth = false;
                }
            }
            m.unlock();
        }
    }

    virtual void waitUntilDone()
    {
        //tries to ackquire lock, if yes, unlock and return
        //if no, it will wait until available.
        m.lock();
        m.unlock();
    }

    virtual void saveOnNextFrame(string _filename, bool _save_depth = false)
    {
        m.lock();
        filename = _filename;
        save = true;
        save_depth = _save_depth;
    }

    /**
     * @brief remove
     * Removes the callback from the camera it was previously associated with
     * using addToViewer() method.
     */
    void remove()
    {
        if (camera != nullptr)
        {
            camera->removePostDrawCallback(this);
        }
    }

    void addToViewer(osgViewer::ViewerBase& viewer)
    {
        osgViewer::ViewerBase::Windows windows;
        viewer.getWindows(windows);

        for(osgViewer::ViewerBase::Windows::iterator itr = windows.begin();
            itr != windows.end();
            ++itr)
        {
            osgViewer::GraphicsWindow* window = *itr;
            osg::GraphicsContext::Cameras& cameras = window->getCameras();
            osg::Camera* lastCamera = nullptr;
            for(osg::GraphicsContext::Cameras::iterator cam_itr = cameras.begin();
                cam_itr != cameras.end();
                ++cam_itr)
            {
                if (lastCamera)
                {
                    if ((*cam_itr)->getRenderOrder() > lastCamera->getRenderOrder())
                    {
                        lastCamera = (*cam_itr);
                    }
                    if ((*cam_itr)->getRenderOrder() == lastCamera->getRenderOrder() &&
                        (*cam_itr)->getRenderOrderNum() >= lastCamera->getRenderOrderNum())
                    {
                        lastCamera = (*cam_itr);
                    }
                }
                else
                {
                    lastCamera = *cam_itr;
                }
            }

            if (lastCamera)
            {
                osg::notify(osg::NOTICE)<<"Last camera "<<lastCamera<<std::endl;
                camera = lastCamera;
                camera->setPostDrawCallback(this);

            }
            else
            {
                osg::notify(osg::NOTICE)<<"No camera found"<<std::endl;
            }
        }

    }


    void addToViewerMainCamera(osgViewer::Viewer &viewer)
    {
        std::cout << "Adding image capture callback to viewer main camera" << std::endl;
        viewer.getCamera()->setPostDrawCallback(this);
    }

    void setNearFar(double _znear, double _zfar)
    {
        znear = _znear;
        zfar = _zfar;
    }


};

}//namespace itr
#endif // WINDOWCAPTURECALLBACK_H
