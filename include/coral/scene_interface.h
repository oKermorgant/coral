#ifndef CORAL_SCENE_INTERFACE_H
#define CORAL_SCENE_INTERFACE_H

#include <coral/scene_params.h>
#include <string>
#include <coral/Scene.h>
#include <memory>
#include <mutex>

#include <osgViewer/ViewerBase>

namespace coral
{

class SceneInterface
{
public:
  SceneInterface(SceneParams params);

  inline osg::Group* world() {return root.get();}

  inline osgOcean::OceanScene* oceanScene()
  {
      return scene.oceanScene();
  }
  inline osgOcean::OceanTechnique* oceanSurface()
  {
    return scene.oceanSurface();
  }
  inline Scene* fullScene()
  {
    return &scene;
  }

  inline SceneParams sceneParams() const {return params;}

  inline void lock() {scene_mtx.lock();}
  inline void unlock() {scene_mtx.unlock();}
  inline std::mutex* mutex() {return &scene_mtx;}

protected:
  SceneParams params;
  // wrapper around osgocean example
  Scene scene;
  osg::ref_ptr<osg::Group> root;

  osgViewer::View* view{nullptr};

  std::mutex scene_mtx;
};

}

#endif // SCENEINTERFACE_H
