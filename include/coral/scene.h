#ifndef CORAL_SCENE_H_
#define CORAL_SCENE_H_

#include <osgOcean/FFTOceanSurface>
#include <osgOcean/ShaderManager>
#include <coral/OceanScene.h>
#include <coral/SkyDome.h>
#include <coral/weather.h>
#include <coral/water.h>

namespace coral
{

class Scene
{
  class EventHandler : public osgGA::GUIEventHandler
  {
  public:
    EventHandler(Scene* scene) : scene{scene} {}
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*) override;
    void getUsage(osg::ApplicationUsage& usage) const override;
  protected:
    Scene* scene;
  };

  Weather::Mood mood;
  osg::ref_ptr<osg::Group> root,scene;

  osg::ref_ptr<OceanScene> oceanScene;
  osg::ref_ptr<osgOcean::FFTOceanTechnique> FFToceanSurface;
  osg::ref_ptr<osg::TextureCubeMap> cubemap;
  osg::ref_ptr<SkyDome> skyDome;
  osg::ref_ptr<osg::Light> sun;
  osg::ref_ptr<EventHandler> event_handler;

public:

  Scene(const SceneParams &params);
  inline auto getEventHandler()
  {
    if(!event_handler.valid())
      event_handler = osg::make_ref<EventHandler>(this);
    return event_handler.get();
  }

  void changeMood(const Weather::Mood &mood, bool force = false);
  void loadCubeMapTextures( const std::string& dir = "sky_clear");

  inline auto getSurface() const
  {
    return FFToceanSurface.get();
  }

  inline auto getOceanScene() const
  {
    return oceanScene.get();
  }

  inline auto getScene() const
  {
    return scene.get();
  }

  inline auto getRoot() const
  {
    return root.get();
  }
};
}

#endif
