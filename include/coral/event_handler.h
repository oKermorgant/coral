#ifndef CORAL_EVENT_HANDLER_H
#define CORAL_EVENT_HANDLER_H

#include <osgGA/GUIEventHandler>
#include <osgGA/FlightManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/DriveManipulator>

#include <coral/Scene.h>
#include <osgViewer/Viewer>

namespace coral
{

class EventHandler : public osgGA::GUIEventHandler
{
public:
  EventHandler(Scene* scene, osgViewer::Viewer* view) : scene(scene), view(view)
  {  }

  void getUsage(osg::ApplicationUsage& usage) const;
  bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);

private:
  Scene*                scene;
  osgViewer::Viewer*      view;     
};




}

#endif // CORAL_EVENT_HANDLER_H
