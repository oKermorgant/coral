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
    const auto mood{Weather::from(req->weather)};
    auto &weather{scene->weather(mood)};
    static SceneColor::Request prev;
    if(prev == *req)
      return;
    if(!req->weather.empty() && req->weather != prev.weather)
      weather = Weather(req->weather);
    prev = *req;

    const auto color = Weather::intColor(req->r, req->g, req->b, req->a);
    if(req->origin == "light")          // useless?
      weather.lightColor = color;
    else if(req->origin == "fog")       // horizon fog
      weather.fogColor = color;
    else if(req->origin == "sunAmbient")  // similar to sunDiffuse
      weather.sunAmbient = color;
    else if(req->origin == "sunDiffuse")    // overall light
      weather.sunDiffuse = color;

  scene->changeMood(mood);
  };
}

}



#endif
