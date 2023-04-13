#ifndef CORAL_WEATHER_H
#define CORAL_WEATHER_H

#include <string>
#include <osg/Vec3f>
#include <osg/Vec4f>
#include <osg/Light>

namespace coral
{

struct Weather
{
  enum class Mood{ CLEAR, DUSK, CLOUDY, NIGHT, CUSTOM};

  std::string cubemap;
  osg::Vec4f lightColor;
  osg::Vec4f fogColor;

  osg::Vec3f underwaterAttenuation;
  osg::Vec4f underwaterDiffuse;
  osg::Vec3f sunDirection;
  osg::Vec3f sunPosition;
  osg::Vec4f sunDiffuse;
  osg::Vec4f sunAmbient;
  osg::Vec4f underwaterFogColor;
  float aboveWaterFogDensity = {0.0012f};
  float underwaterFogDensity = {0.002f};

  inline static Mood from(const std::string &name)
  {
    if(name == "cloudy") return Mood::CLOUDY;
    if(name == "dusk")   return Mood::DUSK;
    if(name == "night")  return Mood::NIGHT;
    if(name == "custom")  return Mood::CUSTOM;
    return Mood::CLEAR;
  }

  inline explicit Weather(const std::string &name = "clear")
  {
    switchTo(from(name));
  }

  static inline osg::Vec4f intColor(unsigned r, unsigned g, unsigned b, unsigned a = 255 )
  {
    const float div{1.f/255.f};
    return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
  }

  inline void switchTo(Mood mood)
  {
    switch (mood)
    {
      case Mood::CUSTOM:
        cubemap = "sky_custom";
        break;
      case Mood::CLEAR:
        cubemap = "sky_clear";
        fogColor = intColor( 199,226,255 );
        lightColor = intColor( 136,226,255 );
        underwaterAttenuation = osg::Vec3f(0.015f, 0.0075f, 0.005f);
        underwaterDiffuse = intColor(27,57,109);
        sunPosition = osg::Vec3f(326.573, 1212.99 ,1275.19);
        sunDiffuse = intColor( 191, 191, 191 );
        sunAmbient = sunDiffuse / 3.f;
        underwaterFogColor = intColor(72, 110, 123);
        underwaterFogDensity = 0.015f;
        break;
      case Mood::DUSK:
        cubemap = "sky_dusk";
        fogColor = lightColor = intColor( 244,228,179 );
        underwaterAttenuation =  osg::Vec3f(0.015f, 0.0075f, 0.005f);
        underwaterDiffuse = intColor(44,69,106);
        sunPosition = osg::Vec3f(520.f, 1900.f, 550.f );
        sunDiffuse = intColor( 251, 251, 161 );
        sunAmbient = sunDiffuse / 4.f;
        underwaterFogColor = intColor(44,69,106 );
        underwaterFogDensity = 0.03f;
        break;
      case Mood::CLOUDY:
        cubemap = "sky_fair_cloudy";
        fogColor = lightColor = intColor( 172,224,251 );
        underwaterAttenuation = osg::Vec3f(0.008f, 0.003f, 0.002f);
        underwaterDiffuse = intColor(84,135,172);
        sunPosition = osg::Vec3f(-1056.89f, -771.886f, 1221.18f );
        sunDiffuse = intColor( 191, 191, 191 );
        sunAmbient = sunDiffuse / 2.f;
        underwaterFogColor = intColor(28,80,100);
        underwaterFogDensity = 0.02f;
        break;
      case Mood::NIGHT:
        cubemap = "sky_night";
        fogColor = intColor(20,20,50);
        lightColor = intColor(20,20,50);
        underwaterAttenuation = osg::Vec3f(0.8f, 0.3f, 0.2f);
        underwaterDiffuse = intColor(10, 10, 30);
        sunPosition = osg::Vec3f(100000.f, 100000.f, 100000.f );
        sunDiffuse = intColor( 10, 10, 10 );
        sunAmbient = sunDiffuse / 3.f;
        underwaterFogColor = underwaterDiffuse;
        underwaterFogDensity = 0.06f;
        break;
    }

    lightColor /= 10.f;
    sunDirection = -sunPosition;
    sunDirection.normalize();
  }
};

}



#endif // CORAL_WEATHER_H
