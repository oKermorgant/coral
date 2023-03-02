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
  enum class Mood{ CLEAR, DUSK, CLOUDY, NIGHT }; 
  std::string cubemap;
  osg::Vec4f lightColor;
  osg::Vec4f fogColor;
  osg::Vec3f underwaterAttenuation;
  osg::Vec4f underwaterDiffuse;
  osg::Vec3f sunPosition;
  osg::Vec4f sunDiffuse;
  osg::Vec4f sunAmbient;
  osg::Vec4f waterFogColor;

  inline static Mood from(const std::string &name)
  {
    if(name == "cloudy") return Mood::CLOUDY;
    if(name == "dusk")   return Mood::DUSK;
    if(name == "night")  return Mood::NIGHT;
    return Mood::CLEAR;
  }

  inline explicit SceneType(const std::string &name)
  {
    init(from(name));
  }

  inline explicit SceneType(Mood mood)
  {
    init(mood);
  }

  static inline osg::Vec4f intColor(unsigned r, unsigned g, unsigned b, unsigned a = 255 )
  {
    const float div{1.f/255.f};
    return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
  }

  inline void init(Mood mood)
  {
    switch (mood)
    {
      case Mood::CLEAR:
        cubemap = "sky_clear";
        fogColor = lightColor = intColor( 199,226,255 );
        underwaterAttenuation = osg::Vec3f(0.015f, 0.0075f, 0.005f);
        underwaterDiffuse = intColor(27,57,109);
        sunPosition = osg::Vec3f(326.573, 1212.99 ,1275.19);
        sunDiffuse = intColor( 191, 191, 191 );
        sunAmbient = sunDiffuse / 3.f;
        waterFogColor = intColor(27,57,109);
        break;
      case Mood::DUSK:
        cubemap = "sky_dusk";
        fogColor = lightColor = intColor( 244,228,179 );
        underwaterAttenuation =  osg::Vec3f(0.015f, 0.0075f, 0.005f);
        underwaterDiffuse = intColor(44,69,106);
        sunPosition = osg::Vec3f(520.f, 1900.f, 550.f );
        sunDiffuse =  intColor( 251, 251, 161 );
        sunAmbient = sunDiffuse / 4.f;
        waterFogColor = intColor(44,69,106 );
        break;
      case Mood::CLOUDY:
        cubemap = "sky_fair_cloudy";
        fogColor = lightColor = intColor( 172,224,251 );
        underwaterAttenuation = osg::Vec3f(0.008f, 0.003f, 0.002f);
        underwaterDiffuse = intColor(84,135,172);
        sunPosition = osg::Vec3f(-1056.89f, -771.886f, 1221.18f );
        sunDiffuse =  intColor( 191, 191, 191 );
        sunAmbient = sunDiffuse / 2.f;
        waterFogColor = intColor(84,135,172 );
        break;
      case Mood::NIGHT:
        cubemap = "sky_night";
        fogColor = lightColor = intColor(20,20,50);
        underwaterAttenuation = osg::Vec3f(0.008f, 0.003f, 0.002f);
        underwaterDiffuse = intColor(10, 10, 30);
        sunPosition = osg::Vec3f(0.f, 0.f, -100.f );
        sunDiffuse = intColor( 10, 10, 30 );
        sunAmbient = sunDiffuse / 3.f;
        waterFogColor = underwaterDiffuse;
    }
    lightColor /= 10.f;
  }
};

}



#endif // CORAL_SCENE_TYPE_H
