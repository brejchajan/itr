/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   2.2.2018
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

#include "terrainsampling/displayequirecttexture.h"

namespace itr{

DisplayEquirectTexture::DisplayEquirectTexture(osg::ref_ptr<TextureCubeMap> _cubeMap)
{
    setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    setRenderOrder(osg::Camera::POST_RENDER);
    setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    setViewMatrix(osg::Matrix());
    setProjectionMatrix(osg::Matrix::ortho2D(0, 1, 0, 1));
    getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF);
    cubeMap = _cubeMap;

    screen = createFullScreenCylQuad();
    addChild(screen);

    setName("Display");
}

ref_ptr<osg::Geode> DisplayEquirectTexture::createFullScreenCylQuad()
{
    ref_ptr<Geode> quad = new Geode();
    ref_ptr<Geometry> geom = new Geometry();
    quad->addDrawable(geom);
    geom->setUseDisplayList(false);

    ref_ptr<Vec3Array> vert = new Vec3Array;

    //OTHERS
    vert->push_back(Vec3(-1.0, 1.0, 0));
    vert->push_back(Vec3(1.0, 1.0, 0));
    vert->push_back(Vec3(1.0, -1.0, 0));
    vert->push_back(Vec3(-1.0, -1.0, 0));

    geom->setVertexArray(vert);

    DrawElementsUInt *plane = new DrawElementsUInt(PrimitiveSet::TRIANGLE_FAN, 0);
    plane->push_back(0);
    plane->push_back(1);
    plane->push_back(2);
    plane->push_back(3);
    geom->addPrimitiveSet(plane);

    osgEarth::Registry::shaderGenerator().run(quad);

    ref_ptr<StateSet> ss = quad->getOrCreateStateSet();
    ss->setTextureAttributeAndModes(0, cubeMap.get(), StateAttribute::ON);

    const char * tex_pos_vertex = "\n"
        "#version 410\n"
        "out vec3 pos;\n"
        "void tex_pos(inout vec4 vertex){\n"
        "   pos = vertex.xyz / vertex.w;\n"
        "   vertex = vertex;\n"
        "}";

    const char * cubemap_fragment = "\n"
        "#version 410\n"
        "#define M_PI 3.1415926535897932384626433832795\n"
        "in vec3 pos;\n"
        "uniform samplerCube cubemap;\n"
        "void apply_texture(inout vec4 color){\n"
        "   float alpha = -(pos.x) * (2.0*M_PI);\n"
        "   float beta = (pos.y * M_PI) + (M_PI/2.0);\n"
        "   vec3 dir = vec3(cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta))\n;"
        "   color = vec4(texture(cubemap, dir));\n"
        "   //color = vec4(dir, 1.0f);\n"
        "}";

    ref_ptr<Uniform> textureUniform = new Uniform;
    textureUniform->setName("cubemap");
    textureUniform->setType(osg::Uniform::SAMPLER_CUBE);
    textureUniform->set(0);
    geom->getOrCreateStateSet()->addUniform(textureUniform.get());

    VirtualProgram* vp = VirtualProgram::getOrCreate(geom->getOrCreateStateSet());
    vp->setFunction( "tex_pos", tex_pos_vertex, ShaderComp::LOCATION_VERTEX_MODEL);
    vp->setFunction( "apply_texture", cubemap_fragment, ShaderComp::LOCATION_FRAGMENT_COLORING);

    return quad;
}


} //namespace itr
