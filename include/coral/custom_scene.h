#ifndef CORAL_CUSTOM_SCENE_H_
#define CORAL_CUSTOM_SCENE_H_

#include <coral/coral_node.h>
#include <coral/OceanScene.h>
#include <coral/srv/scene_color.hpp>



namespace coral
{

auto colorCallback(OceanScene *scene)
{
  using coral::srv::SceneColor;
  return [=](const SceneColor::Request::SharedPtr req, SceneColor::Response::SharedPtr)
  {
    static Weather weather(req->weather);
    static SceneColor::Request prev;
    if(prev == *req)
      return;
    if(!req->weather.empty() && req->weather != prev.weather)
      weather = Weather(req->weather);
    prev = *req;

    const auto color = Weather::intColor(req->r, req->g, req->b, req->a);
    if(req->origin == "light")
      weather.lightColor = color;
    else if(req->origin == "fog")
      weather.fogColor = color;
    else if(req->origin == "uwDiffuse")
      weather.underwaterDiffuse = color;
    else if(req->origin == "sunAmbient")
      weather.sunAmbient = color;
    else if(req->origin == "uwFog")
      weather.underwaterFogColor = color;
    else if(req->origin == "sunDiffuse")
      weather.sunDiffuse = color;
    else if(req->origin == "uwAttenuation")
      weather.underwaterAttenuation = {req->r/100.f, req->g/100.f, req->b/100.f};
    else if(req->origin == "uwDensity")
      weather.underwaterFogDensity = color.a() + 10*color.r() + 100*color.g() + 1000*color.b();

  scene->changeMood(weather);
  };
}

}



#endif
