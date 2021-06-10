#ifndef CORAL_SCENE_TYPE_H
#define CORAL_SCENE_TYPE_H

#include <string>
#include <osg/Vec3f>
#include <osg/Vec4f>
#include <osg/Light>

namespace coral
{

struct SceneType
{
  enum SCENE_TYPE{ CLEAR, DUSK, CLOUDY };

  std::string cubemap;
  osg::Vec4f lightColor = intColor( 105,138,174 );
  osg::Vec4f fogColor = intColor( 199,226,255 );
  osg::Vec3f underwaterAttenuation = osg::Vec3f(0.015f, 0.0075f, 0.005f);
  osg::Vec4f underwaterDiffuse = intColor(27,57,109);
  osg::Vec3f sunPosition = osg::Vec3f(326.573, 1212.99 ,1275.19);
  osg::Vec4f sunDiffuse =  intColor( 191, 191, 191 );
  osg::Vec4f waterFogColor = intColor(27,57,109);

  inline SceneType()
  {
    // default values for CLEAR
    switchTo(SceneType::CLEAR);
  }

  static inline osg::Vec4f intColor(unsigned r, unsigned g, unsigned b, unsigned a = 255 )
  {
    float div = 1.f/255.f;
    return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
  }

  inline void switchTo(SCENE_TYPE type)
  {
    if(type == CLEAR)
    {
      cubemap = "sky_clear";
      fogColor = intColor( 199,226,255 );
      underwaterAttenuation = osg::Vec3f(0.015f, 0.0075f, 0.005f);
      underwaterDiffuse = intColor(27,57,109);
      sunPosition = osg::Vec3f(326.573, 1212.99 ,1275.19);
      sunDiffuse =  intColor( 191, 191, 191 );
      waterFogColor = intColor(27,57,109);
    }
    else if(type == DUSK)
    {
      cubemap = "sky_dusk";
      fogColor = intColor( 244,228,179 );
      underwaterAttenuation =  osg::Vec3f(0.015f, 0.0075f, 0.005f);
      underwaterDiffuse = intColor(44,69,106);
      sunPosition = osg::Vec3f(520.f, 1900.f, 550.f );
      sunDiffuse =  intColor( 251, 251, 161 );
      waterFogColor = intColor(44,69,106 );
    }
    else if(type == CLOUDY)
    {
      cubemap = "sky_fair_cloudy";
      fogColor = intColor( 172,224,251 );
      underwaterAttenuation = osg::Vec3f(0.008f, 0.003f, 0.002f);
      underwaterDiffuse = intColor(84,135,172);
      sunPosition = osg::Vec3f(-1056.89f, -771.886f, 1221.18f );
      sunDiffuse =  intColor( 191, 191, 191 );
      waterFogColor = intColor(84,135,172 );
    }
  }
};

}



#endif // CORAL_SCENE_TYPE_H
