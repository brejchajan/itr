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


#ifndef PIXELBUFFEREGL_H
#define PIXELBUFFEREGL_H

//OpenGL headers
#define EGL_EGLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <EGL/eglext.h>

//OpenSceneGraph headers
#include <osg/GraphicsContext>

//stl headers
#include <iostream>
#include <stdexcept>
#include <string>

class PixelBufferEGL : public osg::GraphicsContext
{
public:
    PixelBufferEGL(GraphicsContext::Traits *traits, int gpu_idx = 0);

    virtual bool valid() const { return _valid; }

    /** Realise the GraphicsContext.*/
    virtual bool realizeImplementation();

    /** Return true if the graphics context has been realised and is ready to use.*/
    virtual bool isRealizedImplementation() const { return _realized; }

    /** Close the graphics context.*/
    virtual void closeImplementation();

    /** Make this graphics context current.*/
    virtual bool makeCurrentImplementation();

    /** Make this graphics context current with specified read context implementation. */
    virtual bool makeContextCurrentImplementation(osg::GraphicsContext* readContext);

    /** Bind the graphics context to associated texture implementation.*/
    virtual void bindPBufferToTextureImplementation(GLenum buffer);

    /** Swap the front and back buffers.*/
    virtual void swapBuffersImplementation();

    /** Release the graphics context.*/
    virtual bool releaseContextImplementation();

    void resizeImplementation(int width, int height);

    int getWidth();

    int getHeight();

protected:
    void init();

private:
    /// True after init() is called sucessfully
    bool _valid;

    /// True after realizeImplementation() is called
    bool _realized;

    /// True after init() ended
    bool _initialized;

    EGLDisplay _eglDpy;

    EGLSurface _eglSurf;

    EGLConfig _eglCfg;

    EGLContext _eglCtx;

    //index of the gpu device to be used
    int gpu_idx;
};

#endif // PIXELBUFFEREGL_H
