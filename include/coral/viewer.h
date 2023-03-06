#ifndef CORAL_VIEWER_H
#define CORAL_VIEWER_H

#include <osgViewer/Viewer>
#include <osgWidget/WindowManager>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osg/Camera>
#include <coral/OceanScene.h>

namespace coral
{

class Viewer : public osgViewer::Viewer
{
public:
  Viewer(OceanScene *scene);

  void frame(double simulationTime=USE_REFERENCE_TIME) override;

  void lockCamera(const osg::Matrixd &M);
  void freeCamera();

private:  
  OceanScene* scene;

  // free / tracking manipulators
  osg::ref_ptr<osg::Camera> camera;
  osg::ref_ptr <osg::MatrixTransform> cam_pose = new osg::MatrixTransform;
  osg::ref_ptr <osgGA::TrackballManipulator> free_manip = new osgGA::TrackballManipulator;
  osg::ref_ptr <osgGA::NodeTrackerManipulator> tracking_manip = new osgGA::NodeTrackerManipulator;

  // detect when window has been resized
  osg::ref_ptr<osgWidget::WindowManager> wm;
  osgViewer::GraphicsWindow* window;
  int width, height;

  bool windowWasResized();

  void changeMood(Weather::Mood mood);

  class EventHandler : public osgGA::GUIEventHandler
  {
  public:
    EventHandler(Viewer* viewer) : viewer{viewer} {}
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*) override;
    void getUsage(osg::ApplicationUsage& usage) const override;
  protected:
    Viewer* viewer;
  };

  osg::ref_ptr<EventHandler> event_handler;
};

}

#endif // CORAL_VIEWER_H
