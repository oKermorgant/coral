#pragma once
#include <osg/Switch>
#include <osg/TextureCubeMap>

#include <osgText/Text>

#include <coral/OceanScene.h>
#include <osgOcean/FFTOceanTechnique>

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


class Scene : public osg::Referenced
{
private:
  SceneParams params;
  SceneType::Mood mood;
  std::mutex mutex;


  osg::ref_ptr<osg::Group> scene, root;

  osg::ref_ptr<osgOcean::OceanScene> ocean_scene;
  osg::ref_ptr<osgOcean::FFTOceanTechnique> ocean_surface;
  osg::ref_ptr<osg::TextureCubeMap> cubemap;
  osg::ref_ptr<SkyDome> skyDome;
  osg::ref_ptr<osg::Light> light;

public:
  Scene(const SceneParams &params);
  const SceneParams & parameters() const {return params;}

  [[nodiscard]] inline auto lock() {return std::lock_guard(mutex);}

  inline auto getMood() const {return SceneType(mood);}
  void changeScene(const SceneType &scene_type);
  inline void changeScene( const std::string &type)
  {
    changeScene(SceneType(type));
  }

  void loadCubeMapTextures( const std::string& dir );

  inline osgOcean::OceanTechnique* oceanSurface()
  {
    return ocean_surface.get();
  }

  inline auto fullScene() const
  {
    return root.get();
  }

  inline auto oceanScene() const
  {
    return ocean_scene.get();
  }

  osg::Light* getLight() const { return light.get(); }

};

}
