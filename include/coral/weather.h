#ifndef CORAL_WEATHER_H
#define CORAL_WEATHER_H

#include <string>
#include <osg/Vec3f>
#include <osg/Vec4f>
#include <osg/Light>

//#define ALLOW_CUSTOM_COLORS

namespace coral
{

struct Weather
{
  enum class Mood{ CLEAR, DUSK, CLOUDY, NIGHT, CUSTOM};

  std::string cubemap;
  osg::Vec4f lightColor;

  osg::Vec4f fogColor;
  float fogDensity = {0.0012f};

  inline auto sunDirection() const {return -sunPosition;}
  osg::Vec3f sunPosition;
  osg::Vec4f sunDiffuse;
  osg::Vec4f sunAmbient;

  inline static Mood from(const std::string &name)
  {
    if(name == "cloudy") return Mood::CLOUDY;
    if(name == "dusk")   return Mood::DUSK;
    if(name == "night")  return Mood::NIGHT;
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
        sunPosition = osg::Vec3f(326.573, 1212.99 ,1275.19);
        sunDiffuse = intColor( 191, 191, 191 );
        sunAmbient = sunDiffuse / 3.f;
        break;
      case Mood::DUSK:
        cubemap = "sky_dusk";
        fogColor = lightColor = intColor( 244,228,179 );
        sunPosition = osg::Vec3f(520.f, 1900.f, 550.f );
        sunDiffuse = intColor( 251, 251, 161 );
        sunAmbient = sunDiffuse / 4.f;
        break;
      case Mood::CLOUDY:
        cubemap = "sky_fair_cloudy";
        fogColor = lightColor = intColor( 172,224,251 );
        sunPosition = osg::Vec3f(-1056.89f, -771.886f, 1221.18f );
        sunDiffuse = intColor( 191, 191, 191 );
        sunAmbient = sunDiffuse / 2.f;
        break;
      case Mood::NIGHT:
        cubemap = "sky_night";
        fogColor = intColor(20,20,50);
        lightColor = intColor(20,20,50);
        sunPosition = osg::Vec3f(100000.f, 100000.f, 100000.f );
        sunDiffuse = intColor( 10, 10, 10 );
        sunAmbient = sunDiffuse / 3.f;
        break;
    }
    lightColor /= 10.f;
    sunPosition.normalize();
  }
};

}



#endif // CORAL_WEATHER_H
