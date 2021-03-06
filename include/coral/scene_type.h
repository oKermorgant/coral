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
  enum class Type{ CLEAR, DUSK, CLOUDY, NIGHT };

  std::string cubemap;
  osg::Vec4f lightColor = intColor( 105,138,174 );
  osg::Vec4f fogColor = intColor( 199,226,255 );
  osg::Vec3f underwaterAttenuation = osg::Vec3f(0.015f, 0.0075f, 0.005f);
  osg::Vec4f underwaterDiffuse = intColor(27,57,109);
  osg::Vec3f sunPosition = osg::Vec3f(326.573, 1212.99 ,1275.19);
  osg::Vec4f sunDiffuse =  intColor( 191, 191, 191 );
  osg::Vec4f waterFogColor = intColor(27,57,109);

  inline SceneType(const std::string &type)
  {
    if(type == "cloudy")
      switchTo(Type::CLOUDY);
    else if(type == "dusk")
      switchTo(Type::DUSK);
    else if(type == "night")
      switchTo(Type::NIGHT);
    else
      switchTo(Type::CLEAR);
  }

  static inline osg::Vec4f intColor(unsigned r, unsigned g, unsigned b, unsigned a = 255 )
  {
    float div = 1.f/255.f;
    return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
  }

  inline void switchTo(Type type)
  {
    switch (type)
    {
    case Type::CLEAR:
      cubemap = "sky_clear";
      fogColor = intColor( 199,226,255 );
      underwaterAttenuation = osg::Vec3f(0.015f, 0.0075f, 0.005f);
      underwaterDiffuse = intColor(27,57,109);
      sunPosition = osg::Vec3f(326.573, 1212.99 ,1275.19);
      sunDiffuse =  intColor( 191, 191, 191 );
      waterFogColor = intColor(27,57,109);
      break;
    case Type::DUSK:
      cubemap = "sky_dusk";
      fogColor = intColor( 244,228,179 );
      underwaterAttenuation =  osg::Vec3f(0.015f, 0.0075f, 0.005f);
      underwaterDiffuse = intColor(44,69,106);
      sunPosition = osg::Vec3f(520.f, 1900.f, 550.f );
      sunDiffuse =  intColor( 251, 251, 161 );
      waterFogColor = intColor(44,69,106 );
      break;
    case Type::CLOUDY:
      cubemap = "sky_fair_cloudy";
      fogColor = intColor( 172,224,251 );
      underwaterAttenuation = osg::Vec3f(0.008f, 0.003f, 0.002f);
      underwaterDiffuse = intColor(84,135,172);
      sunPosition = osg::Vec3f(-1056.89f, -771.886f, 1221.18f );
      sunDiffuse =  intColor( 191, 191, 191 );
      waterFogColor = intColor(84,135,172 );
      break;
    case Type::NIGHT:
      cubemap = "sky_night";
      fogColor = intColor(20,20,50);
      underwaterAttenuation = osg::Vec3f(0.008f, 0.003f, 0.002f);
      underwaterDiffuse = intColor(10, 10, 30);
      sunPosition = osg::Vec3f(0.f, 0.f, -100.f );
      sunDiffuse =  intColor( 10, 10, 30 );
      waterFogColor = underwaterDiffuse;
    }
  }
};

}



#endif // CORAL_SCENE_TYPE_H
