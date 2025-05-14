#ifndef CORAL_WATER_LIGHTS_H
#define CORAL_WATER_LIGHTS_H

#include <osg/Vec3f>
#include <osg/Vec4f>
#include <coral/weather.h>

namespace coral
{

struct Water
{
  osg::Vec3f attenuation;
  osg::Vec4f diffuse;
  osg::Vec4f fogColor;
  float fogDensity = {0.002f};

  inline Water()
  {
    switchTo(Weather::Mood::CLEAR);
  }

#ifdef ALLOW_CUSTOM_COLORS

  inline Water(float jerlov = 0.2, float density = 0.002f) : fogDensity{density}
  {
    fromJerlov(jerlov);
  }

  void fromJerlov(float val);

#endif

  void switchTo(Weather::Mood mood);
};

}

#endif // CORAL_WATER_LIGHTS_H
