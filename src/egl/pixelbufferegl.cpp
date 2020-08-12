/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   09.10.2018
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
* @Project LandscapeAR 2018
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

#include "egl/pixelbufferegl.h"
#include <string>

PixelBufferEGL::PixelBufferEGL(GraphicsContext::Traits *traits, int _gpu_idx)
    :_valid(false), _realized(false), _initialized(false),
     _eglDpy(0), _eglSurf(0), _eglCfg(0), _eglCtx(0), gpu_idx(_gpu_idx)
{
    _traits = traits;
    init();

    if (valid())
    {
        setState( new osg::State );
        getState()->setGraphicsContext(this);

        if (_traits.valid() && _traits->sharedContext.valid())
        {
            getState()->setContextID( _traits->sharedContext->getState()->getContextID() );
            incrementContextIDUsageCount( getState()->getContextID() );
        }
        else
        {
            getState()->setContextID( osg::GraphicsContext::createNewContextID() );
        }

    }
}

/** Realise the GraphicsContext.*/
bool PixelBufferEGL::realizeImplementation()
{
    std::cout << "Realizing implementation of EGL Pixelbuffer" << std::endl;
    if (_realized)
    {
        OSG_NOTICE<<"PixelBufferEGL::realizeImplementation() Already realized"<<std::endl;
        return true;
    }

    if (!_initialized) init();

    if (!_initialized) return false;

    _realized = true;

    return true;
}

/** Close the graphics context.*/
void PixelBufferEGL::closeImplementation()
{
    std::cout << "Closing implementation of EGL Pixelbuffer" << std::endl;
    if (_eglCtx)
    {
        eglDestroyContext(_eglDpy, _eglCtx);
    }
    _eglCtx = 0;
    _initialized = false;
    _realized = false;
    _valid = false;
}

/** Make this graphics context current.*/
bool PixelBufferEGL::makeCurrentImplementation()
{
    int res = eglMakeCurrent(_eglDpy, _eglSurf, _eglSurf, _eglCtx);
    std::cout << "Making current implementation of EGL Pixelbuffer result: " << res << std::endl;
    return res;//eglMakeCurrent(_eglDpy, _eglSurf, _eglSurf, _eglCtx);
}

/** Make this graphics context current with specified read context implementation. */
bool PixelBufferEGL::makeContextCurrentImplementation(osg::GraphicsContext* readContext)
{
    std::cout << "ERROR NOT IMPLEMENTED: making Context current implementation of EGL Pixelbuffer" << std::endl;
    return false;
}

/** Bind the graphics context to associated texture implementation.*/
void PixelBufferEGL::bindPBufferToTextureImplementation(GLenum buffer)
{
    std::cout << "BindingPBufferToTexture implementation of EGL Pixelbuffer" << std::endl;
}

/** Swap the front and back buffers.*/
void PixelBufferEGL::swapBuffersImplementation()
{
    if (!_realized)
    {
        return;
    }
    eglSwapBuffers(_eglDpy, _eglSurf);
}

bool PixelBufferEGL::releaseContextImplementation()
{
    std::cout << "Releasing implementation of EGL Pixelbuffer" << std::endl;
    if (!_realized)
    {
        OSG_NOTICE<<"Warning: PixelBufferEGL not realized, cannot do release context."<<std::endl;
        return false;
    }
    return eglMakeCurrent(_eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
}

void PixelBufferEGL::resizeImplementation(int width, int height)
{
    if (!_realized)
    {
        OSG_NOTICE<<"Warning: PixelBufferEGL not realized, cannot resize the surface."<<std::endl;
        return;
    }
    std::cout << "Resizing implementation of EGL Pixelbuffer" << std::endl;
    //eglMakeCurrent(_eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroySurface(_eglDpy, _eglSurf);

    _traits->width = width;
    _traits->height = height;
    const EGLint pbufferAttribs[] = {
            EGL_WIDTH, (EGLint)_traits->width,
            EGL_HEIGHT, (EGLint)_traits->height,
            EGL_NONE,
    };

    _eglSurf = eglCreatePbufferSurface(_eglDpy, _eglCfg, pbufferAttribs);
}

int PixelBufferEGL::getWidth()
{
    return _traits->width;
}

int PixelBufferEGL::getHeight()
{
    return _traits->height;
}


void PixelBufferEGL::init()
{
    const EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_BLUE_SIZE, (EGLint)_traits->blue,
        EGL_GREEN_SIZE, (EGLint)_traits->green,
        EGL_RED_SIZE, (EGLint)_traits->red,
        EGL_DEPTH_SIZE, (EGLint)_traits->depth,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_NONE
    };

    const EGLint pbufferAttribs[] = {
        EGL_WIDTH, (EGLint)_traits->width,
        EGL_HEIGHT, (EGLint)_traits->height,
        EGL_NONE,
    };

    // 1. Initialize EGL
    static const int MAX_DEVICES = 255;
    EGLDeviceEXT eglDevs[MAX_DEVICES];
    EGLint numDevices;

    PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
      (PFNEGLQUERYDEVICESEXTPROC) eglGetProcAddress("eglQueryDevicesEXT");
    if (eglQueryDevicesEXT == NULL)
    {
        throw std::runtime_error("Unable to get eglQueryDevicesEXT function pointer.");
    }
    bool res = eglQueryDevicesEXT(MAX_DEVICES, eglDevs, &numDevices);
    std::string ress = res ? "true" : "false";
    printf("Query device success: %s\n", ress.c_str());
    printf("Detected %d devices\n", numDevices);
    printf("GPU idx: %d\n", gpu_idx);

    if (gpu_idx > (numDevices - 1))
    {
        throw std::runtime_error("Wrong gpu index. GPU index cannot exceed the number of \
                                  detected GPU devices - 1. GPU index: " + std::to_string(gpu_idx) + ", \
                                  number of devices: " + std::to_string(numDevices) + ".");
    }

    PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
      (PFNEGLGETPLATFORMDISPLAYEXTPROC) eglGetProcAddress("eglGetPlatformDisplayEXT");

    if (eglGetPlatformDisplayEXT == NULL)
    {
        throw std::runtime_error("Unable to get eglGetPlatformDisplayEXT function pointer.");
    }

    _eglDpy = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT,
                                    eglDevs[gpu_idx], 0);

    EGLint major, minor;
    eglInitialize(_eglDpy, &major, &minor);
    std::cout << "EGL version: " << major << "." << minor << std::endl;
    if (major < 1 || minor < 4)
    {
        throw std::runtime_error("Unable to initialize EGL 1.4. Got version: " + std::to_string(major) + "." + std::to_string(minor));
    }

    // 2. Select an appropriate configuration
    EGLint numConfigs;

    eglChooseConfig(_eglDpy, configAttribs, &_eglCfg, 1, &numConfigs);

    // 3. Create a surface
    _eglSurf = eglCreatePbufferSurface(_eglDpy, _eglCfg, pbufferAttribs);

    // 4. Bind the API
    eglBindAPI(EGL_OPENGL_API);

    // 5. Create a context and make it current
    _eglCtx = eglCreateContext(_eglDpy, _eglCfg, EGL_NO_CONTEXT,
                                       NULL);
    if (_eglCtx == EGL_NO_CONTEXT)
    {
        OSG_NOTICE<<"PixelBufferEGL::init() - eglCreateContext(..) failed."<<std::endl;
        _valid = false;
        return;
    }

    _initialized = true;
    _valid = true;
}
