/*
 *
 *
* This source file is part of the osgOcean library
*
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <coral/SkyDome.h>
#include <osg/Geometry>
#include <osgOcean/ShaderManager>

using namespace coral;


SkyDome::SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
    setupStateSet(cubemap);
}


void SkyDome::setupStateSet( osg::TextureCubeMap* cubemap )
{
    auto ss = osg::make_ref<osg::StateSet>();

    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
    ss->setAttributeAndModes( createShader().get(), osg::StateAttribute::ON );
    ss->addUniform( osg::make_ref<osg::Uniform>("uEnvironmentMap", 0) );

    setStateSet(ss);
}

void SkyDome::compute( float radius,
							unsigned int longitudeSteps,
							unsigned int lattitudeSteps,
							float longStart,
							float longEnd,
							float latStart,
							float latEnd )

{
  removeDrawables(0,getNumDrawables());

  osg::Vec3Array* vertices = new osg::Vec3Array();
  osg::Vec2Array* texcoords = new osg::Vec2Array();

  double longInc = (longEnd - longStart) / (double)longitudeSteps;
  double latInc  = (latEnd  - latStart ) / (double)lattitudeSteps;

  double theta = longStart, phi = latStart;

  float uScale = 1.f / longitudeSteps;
  float vScale = 1.f / lattitudeSteps;

  for( unsigned int i = 0; i <= longitudeSteps; ++i)
  {
    const auto t{osg::DegreesToRadians(theta)};
    const auto sin_t{sin(t)};
    const auto cos_t{cos(t)};

	for( unsigned int j = 0; j <= lattitudeSteps; ++j)
	{
	  const auto p{osg::DegreesToRadians( phi )};

	  const auto x{radius * sin_t * cos(p)};
	  const auto y{radius * sin_t * sin(p)};
	  const auto z{radius * cos_t};

	  vertices->push_back( osg::Vec3( x, y, z ) );
	  texcoords->push_back( osg::Vec2( j*vScale, i*uScale ) );

	  phi += latInc;
	}

	theta -= longInc;
	phi = latStart;
  }

  auto geom = osg::make_ref<osg::Geometry>();

  for(unsigned int r = 0; r <= longitudeSteps-1; r += 1)
  {
    auto indices = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, 0 );

	for(unsigned int c = 0; c <= lattitudeSteps; c += 1 )
	{
	  indices->push_back( idx( r,    c,  lattitudeSteps+1 ) );
	  indices->push_back( idx( r+1,    c,  lattitudeSteps+1 ) );
	}

    geom->addPrimitiveSet( indices );
  }

  osg::Vec4Array* colors = new osg::Vec4Array();
  colors->push_back( osg::Vec4( 1.f, 1.f, 1.f, 1.f ) );

  geom->setVertexArray( vertices );
  geom->setTexCoordArray( 0, texcoords );
  geom->setColorArray( colors );
  geom->setColorBinding( osg::Geometry::BIND_OVERALL );

  addDrawable( geom.get() );
}


osg::ref_ptr<osg::Program> SkyDome::createShader(void)
{
    osg::ref_ptr<osg::Program> program = new osg::Program;

    // Do not use shaders if they were globally disabled.
    if (osgOcean::ShaderManager::instance().areShadersEnabled())
    {
        char vertexSource[]=
            "varying vec3 vTexCoord;\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "    gl_Position = ftransform();\n"
            "    vTexCoord = gl_Vertex.xyz;\n"
            "}\n";

        char fragmentSource[]=
            "uniform samplerCube uEnvironmentMap;\n"
            "varying vec3 vTexCoord;\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "   vec3 texcoord = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);\n"
            "   gl_FragData[0] = textureCube( uEnvironmentMap, texcoord.xzy );\n"
            "   gl_FragData[0].a = 0.0;\n"
            "   gl_FragData[1] = vec4(0.0);\n"
            "}\n";

        program->setName( "sky_dome_shader" );
        program->addShader(osg::make_ref<osg::Shader>(osg::Shader::VERTEX,   vertexSource));
        program->addShader(osg::make_ref<osg::Shader>(osg::Shader::FRAGMENT, fragmentSource));
    }

    return program;
}

osg::Vec2 SkyDome::sphereMap( osg::Vec3& vertex, float radius)
{
  float u, v;

  float TWOPI = M_PI * 2.f;

  v = acos( vertex.y() / radius ) / M_PI;

  if (vertex.z() >= 0.f)
    u = acos( vertex.x() / (radius * sin( M_PI*v ) ) )  / TWOPI;
  else
    u = (M_PI + acos( vertex.x() / ( radius * sin( M_PI * v ) ) ) ) / TWOPI;

  return osg::Vec2( u, v );
}
