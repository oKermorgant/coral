#ifndef CORAL_VIEWER_H
#define CORAL_VIEWER_H

#include <osgViewer/Viewer>
#include <osgWidget/WindowManager>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <coral/scene.h>

namespace coral
{

class Viewer
{
  class EventHandler : public osgGA::GUIEventHandler
  {
  public:
    EventHandler(Viewer* viewer) : viewer{viewer} {}
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*) override;
    void getUsage(osg::ApplicationUsage& usage) const override;
  protected:
    Viewer* viewer;
  };

public:
  Viewer(Scene &scene);

  void frame(double simulationTime=USE_REFERENCE_TIME);
  inline auto done() const
  {
    return viewer->done();
  }

  void lockCamera(const osg::Matrix &M);
  void freeCamera();

  void addCamWidget(osgWidget::Window* widget, int width);

//private:
  osg::ref_ptr<osgViewer::Viewer> viewer{new osgViewer::Viewer};
  Scene* scene;
  osg::ref_ptr<OceanScene> ocean_scene;

  // free / tracking manipulators
  osg::ref_ptr <osg::MatrixTransform> cam_pose{new osg::MatrixTransform};
  osg::ref_ptr <osgGA::TrackballManipulator> free_manip{new osgGA::TrackballManipulator};
  osg::ref_ptr <osgGA::NodeTrackerManipulator> tracking_manip{new osgGA::NodeTrackerManipulator};

  // detect when window has been resized
  osg::ref_ptr<osgWidget::WindowManager> wm;
  std::vector < osg::ref_ptr<osgWidget::Window> > camWidgets;
  osgViewer::GraphicsWindow* window;
  int width, height;
  uint widget_offset{};

  bool windowWasResized();
  inline auto handleResolution()
  {
    ocean_scene->setScreenDims(width, height);
    viewer->getCamera()->setProjectionMatrixAsPerspective(60, (double)width/height, 0.1, 10000.);
  }
};

}

#endif // CORAL_VIEWER_H
