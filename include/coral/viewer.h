#ifndef CORAL_VIEWER_H
#define CORAL_VIEWER_H

#include <osgViewer/Viewer>
#include <osgWidget/WindowManager>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <coral/Scene.h>

namespace coral
{

class Viewer
{
public:
  Viewer(Scene &scene);

  inline bool done() const {return viewer->done();}
  void frame();

  void lockCamera(const osg::Matrixd &M);
  void freeCamera();

  inline osgViewer::Viewer* osg() const {return viewer.get();}

private:
  osg::ref_ptr<osgViewer::Viewer> viewer;
  osg::ref_ptr<osgWidget::WindowManager> wm;
  osgOcean::OceanScene* ocean_scene;
  std::mutex* scene_mutex;

  // free / tracking manipulators
  osg::ref_ptr <osg::MatrixTransform> cam_pose = new osg::MatrixTransform;
  osg::ref_ptr <osgGA::TrackballManipulator> free_manip = new osgGA::TrackballManipulator;
  osg::ref_ptr <osgGA::NodeTrackerManipulator> tracking_manip = new osgGA::NodeTrackerManipulator;

  bool cam_is_free = true;

  // detect when window has been resized
  osgViewer::GraphicsWindow* window;
  int width, height;

  bool windowWasResized();
};

}

#endif // CORAL_VIEWER_H
