#ifndef CORAL_CUSTOM_SCENE_H_
#define CORAL_CUSTOM_SCENE_H_

#include <coral/coral_node.h>
#include <coral/OceanScene.h>
#include <coral/msg/color.hpp>



namespace coral
{

auto colorCallback(OceanScene *scene)
{
  using coral::msg::Color;
  return [=](const Color &msg)
  {
    static Weather weather(msg.base);
    static Color prev;
    if(prev == msg)
      return;
    if(msg.base != prev.base)
      weather = Weather(msg.base);
    prev = msg;

    const auto color = Weather::intColor(msg.r, msg.g, msg.b, msg.a);
    if(msg.field == "light")
      weather.lightColor = color;
    else if(msg.field == "fog")
      weather.fogColor = color;
    else if(msg.field == "uwDiffuse")
      weather.underwaterDiffuse = color;
    else if(msg.field == "sunAmbient")
      weather.sunAmbient = color;
    else if(msg.field == "uwFog")
      weather.underwaterFogColor = color;
    else if(msg.field == "sunDiffuse")
      weather.sunDiffuse = color;
    else if(msg.field == "uwAttenuation")
      weather.underwaterAttenuation = {msg.r/100.f, msg.g/100.f, msg.b/100.f};
    else if(msg.field == "uwDensity")
      weather.underwaterFogDensity = color.a() + 10*color.r() + 100*color.g() + 1000*color.b();

  scene->changeMood(weather);
  };
}

}



#endif
