#ifndef CORAL_EVENT_HANDLER_H
#define CORAL_EVENT_HANDLER_H

#include <osgGA/GUIEventHandler>
#include <osgGA/FlightManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/DriveManipulator>

#include <coral/Scene.h>
#include <coral/viewer.h>

namespace coral
{

class EventHandler : public osgGA::GUIEventHandler
{
public:
  EventHandler(Scene* scene) : scene{scene}
  {  }

  void getUsage(osg::ApplicationUsage& usage) const;
  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*);


private:
  Scene* scene;
};




}

#endif // CORAL_EVENT_HANDLER_H
