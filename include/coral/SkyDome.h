/*
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

#pragma once
#include <osg/Program>
#include <osg/Uniform>
#include <osg/TextureCubeMap>

#include <osg/Geode>
#include <coral/osg_make_ref.h>

namespace coral
{

class SkyDome : public osg::Geode
{
public:
  inline SkyDome( void ) {}
  inline SkyDome( const SkyDome& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY ) : osg::Geode(copy, copyop) {}
  SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap );

public:
  void setupStateSet( osg::TextureCubeMap* cubemap );
  //void create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap );

  inline void setCubeMap( osg::TextureCubeMap* cubemap ){
    getOrCreateStateSet()->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
  }

  // 0 >= longStart/longEnd <= 180
  // 0 >= latStart/latEnd <= 360
  void compute( float radius,
               unsigned int longitudeSteps,
               unsigned int lattitudeSteps,
               float longStart,
               float longEnd,
               float latStart,
               float latEnd	);
private:
  osg::Vec2 sphereMap( osg::Vec3& vertex, float radius);

  inline unsigned int idx(unsigned int r, unsigned int c, unsigned int row_len)
  {
    return c + r * row_len;
  }

  osg::ref_ptr<osg::Program> createShader(void);

};
}
