#pragma once
#include <osg/Switch>
#include <osg/TextureCubeMap>
#include <osgText/Text>

#include <osgOcean/FFTOceanSurface>

#include <coral/OceanScene.h>
#include <coral/SkyDome.h>
#include <coral/scene_params.h>
#include <coral/scene_type.h>

//#define USE_SCENE_LOCK

#ifdef USE_SCENE_LOCK
#include <mutex>
#endif
namespace coral
{

enum DrawMask
{
  CAST_SHADOW             = (0x1<<30),
  RECEIVE_SHADOW          = (0x1<<29),
};


class Scene
{
private:
  SceneParams params;
  osg::Vec4f base_water_color;
#ifdef USE_SCENE_LOCK
  std::mutex mutex;
#endif
  osg::ref_ptr<osgOcean::OceanScene> world;
  osg::ref_ptr<osgOcean::FFTOceanSurface> ocean_surface;
  osg::ref_ptr<osg::TextureCubeMap> cubemap;
  osg::ref_ptr<SkyDome> skyDome;
  osg::ref_ptr<osg::Light> sun;

public:
  Scene(const SceneParams &params);
  const SceneParams & parameters() const {return params;}
#ifdef USE_SCENE_LOCK
  [[nodiscard]] inline auto lock() {return std::lock_guard(mutex);}
#endif
  inline auto underwaterColor(float f = 1.f)
  {
    const auto scaled{base_water_color * f};
    world->setUnderwaterFog(0.002f,   scaled);
    world->setUnderwaterDiffuse(scaled);
    return scaled;
  }
  void changeScene(const SceneType &scene_type);
  inline void changeScene( const std::string &type)
  {
    changeScene(SceneType(type));
  }

  void loadCubeMapTextures( const std::string& dir );

  inline auto oceanSurface() {return ocean_surface.get();}
  inline auto getWorld() const {return world.get();}
};

}
