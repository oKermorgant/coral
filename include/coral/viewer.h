#ifndef CORAL_VIEWER_H
#define CORAL_VIEWER_H

#include <osgViewer/Viewer>
#include <osgWidget/WindowManager>
#include <coral/scene_interface.h>
#include <coral/TextHUD.h>

namespace coral
{

class Viewer
{
public:
  Viewer(SceneInterface &interface);

  inline bool done() const {return viewer->done();}
  inline void frame()
  {
    scene_mutex->lock();
    viewer->frame();
    scene_mutex->unlock();
  }

private:
  osg::ref_ptr<osgViewer::Viewer> viewer;
  osg::ref_ptr<osgWidget::WindowManager> wm;
  osg::ref_ptr<TextHUD> hud;
  std::mutex* scene_mutex;
};

}

#endif // CORAL_VIEWER_H
