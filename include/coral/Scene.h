#pragma once
#include <osg/Switch>
#include <osg/TextureCubeMap>

#include <osgText/Text>

#include <coral/OceanScene.h>
#include <osgOcean/FFTOceanSurface>

#include <coral/SkyDome.h>
#include <coral/scene_params.h>
#include <coral/scene_type.h>
#include <osg/Version>
#include <mutex>
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

  std::mutex mutex;

  osg::ref_ptr<osg::Group> root;
  osg::ref_ptr<osgOcean::OceanScene> ocean_scene;
  osg::ref_ptr<osgOcean::FFTOceanSurface> ocean_surface;
  osg::ref_ptr<osg::TextureCubeMap> cubemap;
  osg::ref_ptr<SkyDome> skyDome;
  osg::ref_ptr<osg::Light> sun;

public:
  Scene(const SceneParams &params);
  const SceneParams & parameters() const {return params;}

  [[nodiscard]] inline auto lock() {return std::lock_guard(mutex);}

  inline auto underwaterColor(float f = 1.f)
  {
    const auto scaled{base_water_color * f};
    ocean_scene->setUnderwaterFog(0.002f,   scaled);
    ocean_scene->setUnderwaterDiffuse(scaled);
    return scaled;
  }
  void changeScene(const SceneType &scene_type);
  inline void changeScene( const std::string &type)
  {
    changeScene(SceneType(type));
  }

  void loadCubeMapTextures( const std::string& dir );

  inline auto oceanSurface() {return ocean_surface.get();}
  inline auto fullScene() const {return root.get();}
  inline auto oceanScene() const {return ocean_scene.get();}
};

}
