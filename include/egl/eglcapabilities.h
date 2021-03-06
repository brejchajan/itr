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

#ifndef EGLCAPABILITIES_H
#define EGLCAPABILITIES_H

#include <osgEarth/Capabilities>
#include <osg/GL2Extensions>
#include <osg/ref_ptr>

#include <egl/pixelbufferegl.h>

#include <iostream>

class EGLCapabilities : public osgEarth::Capabilities
{

private:
    bool _supportsGLSL;

public:

    EGLCapabilities(PixelBufferEGL *gc)
    {
        _supportsGLSL = true;
        /*const char* versionString = (const char*) glGetString( GL_VERSION );
        bool validContext = versionString!=0;
        std::cout << "EGL Capabilities Version string:" << versionString << std::endl;
        if (validContext)
        {
            std::cout << "The context is valid!" << std::endl;
        }

        unsigned int id = gc->getState()->getContextID();
        std::cout << "context id: " << id << std::endl;
        const osg::GL2Extensions* GL2 = osg::GL2Extensions::Get(id, true);
        //osg::ref_ptr<osg::GL2Extensions> GL2 = new osg::GL2Extensions(id);
        std::cout << "obtained gl2 extensions" << std::endl;
        if (GL2)
        {
            std::cout << "setting supportsGLSL: " << GL2->isGlslSupported << std::endl;
            std::cout << "glsl version: " << GL2->glslLanguageVersion << std::endl;
            if (GL2->glslLanguageVersion > 0)
            {
                _supportsGLSL = true;
            }
        }*/

    }

    virtual bool supportsGLSL() const
    {
        std::cout << "Getting GLSL support " << _supportsGLSL << std::endl;
        return _supportsGLSL;
    }



};

#endif // EGLCAPABILITIES_H
