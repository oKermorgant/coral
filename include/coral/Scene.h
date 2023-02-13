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


class Scene : public osg::Referenced
{
private:
  SceneParams params;
  SceneType scene_type;
  std::mutex mutex;


  osg::ref_ptr<osg::Group> scene, root;

  osg::ref_ptr<osgOcean::OceanScene> ocean_scene;
  osg::ref_ptr<osgOcean::FFTOceanSurface> ocean_surface;
  osg::ref_ptr<osg::TextureCubeMap> cubemap;
  osg::ref_ptr<SkyDome> skyDome;
  osg::ref_ptr<osg::Light> light;

public:
  Scene(const SceneParams &params);
  const SceneParams & parameters() const {return params;}

  [[nodiscard]] inline auto lock() {return std::lock_guard(mutex);}


  void changeScene( SceneType::Type type);

  osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures( const std::string& dir );

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


  class EventHandler : public osgGA::GUIEventHandler
  {
  public:
    EventHandler(Scene* scene) : scene{scene} {}
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*) override;
    void getUsage(osg::ApplicationUsage& usage) const override;
  protected:
    Scene* scene;
  };

  osg::ref_ptr<EventHandler> event_handler;

  EventHandler* getEventHandler()
  {
    if (!event_handler.valid())
      event_handler = new EventHandler(this);
    return event_handler.get();
  }
};

}
