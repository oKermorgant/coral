#include <coral/event_handler.h>

namespace coral
{


void EventHandler::getUsage(osg::ApplicationUsage& usage) const
{ 
  usage.addKeyboardMouseBinding("1","Select scene \"Clear Blue Sky\"");
  usage.addKeyboardMouseBinding("2","Select scene \"Dusk\"");
  usage.addKeyboardMouseBinding("3","Select scene \"Pacific Cloudy\"");
  usage.addKeyboardMouseBinding("4","Select scene \"Night\"");
  usage.addKeyboardMouseBinding("0","Toggle ocean surface");

  /* usage.addKeyboardMouseBinding("r","Toggle reflections (above water)");
  usage.addKeyboardMouseBinding("R","Toggle refractions (underwater)");
  usage.addKeyboardMouseBinding("o","Toggle Depth of Field (DOF) (underwater)");
  usage.addKeyboardMouseBinding("g","Toggle glare (above water)");
  usage.addKeyboardMouseBinding("G","Toggle God rays (underwater)");
  usage.addKeyboardMouseBinding("T","Toggle scattering (underwater)");
  usage.addKeyboardMouseBinding("H","Toggle Height lookup for shoreline foam and sine shape (above water)");
  usage.addKeyboardMouseBinding("+","Raise ocean surface");
  usage.addKeyboardMouseBinding("-","Lower ocean surface");*/

}


bool EventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*)
{
  if (ea.getHandled()) return false;

  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
  {

    if(key == '1')
    {
      scene->changeScene( SceneType::Type::CLEAR );
      return false;
    }
    else if(key == '2')
    {
      scene->changeScene( SceneType::Type::DUSK );
      return false;
    }
    else if(key == '3' )
    {
      scene->changeScene( SceneType::Type::CLOUDY );
      return false;
    }
    else if(key == '4' )
    {
      scene->changeScene( SceneType::Type::NIGHT);
      return false;
    }
    // Reflections
    /* else if (key == 'r')
    {
      oceanScene->enableReflections(!oceanScene->areReflectionsEnabled());
      osg::notify(osg::NOTICE) << "Reflections " << (oceanScene->areReflectionsEnabled()? "enabled" : "disabled") << std::endl;
      return true;
    }
    // Refractions
    if (key == 'R')
    {
      oceanScene->enableRefractions(!oceanScene->areRefractionsEnabled());
      osg::notify(osg::NOTICE) << "Refractions " << (oceanScene->areRefractionsEnabled()? "enabled" : "disabled") << std::endl;
      return true;
    }
    // DOF
    if (key == 'o')
    {
      oceanScene->enableUnderwaterDOF(!oceanScene->isUnderwaterDOFEnabled());
      osg::notify(osg::NOTICE) << "Depth of field " << (oceanScene->isUnderwaterDOFEnabled()? "enabled" : "disabled") << std::endl;
      return true;
    }
    // Glare
    if (key == 'g')
    {
      oceanScene->enableGlare(!oceanScene->isGlareEnabled());
      osg::notify(osg::NOTICE) << "Glare " << (oceanScene->isGlareEnabled()? "enabled" : "disabled") << std::endl;
      return true;
    }
    // God rays
    if (key == 'G')
    {
      oceanScene->enableGodRays(!oceanScene->areGodRaysEnabled());
      osg::notify(osg::NOTICE) << "God rays " << (oceanScene->areGodRaysEnabled()? "enabled" : "disabled") << std::endl;
      return true;
    }
    // Underwater scattering
    if (key == 'T')
    {
      oceanScene->enableUnderwaterScattering(!oceanScene->isUnderwaterScatteringEnabled());
      osg::notify(osg::NOTICE) << "Underwater scattering " << (oceanScene->isUnderwaterScatteringEnabled()? "enabled" : "disabled") << std::endl;
      return true;
    }
    // Height map
    if (key == 'H')
    {
      oceanScene->enableHeightmap(!oceanScene->isHeightmapEnabled());
      oceanScene->getOceanTechnique()->dirty();      // Make it reload shaders.
      osg::notify(osg::NOTICE) << "Height lookup for shoreline foam and sine shape " << (oceanScene->isHeightmapEnabled()? "enabled" : "disabled") << std::endl;
      return true;
    }
    // Ocean surface height
    if (key == '+')
    {
      oceanScene->setOceanSurfaceHeight(oceanScene->getOceanSurfaceHeight() + 1.0);
      osg::notify(osg::NOTICE) << "Ocean surface is now at z = " << oceanScene->getOceanSurfaceHeight() << std::endl;
      return true;
    }
    if (key == '-')
    {
      oceanScene->setOceanSurfaceHeight(oceanScene->getOceanSurfaceHeight() - 1.0);
      osg::notify(osg::NOTICE) << "Ocean surface is now at z = " << oceanScene->getOceanSurfaceHeight() << std::endl;
      return true;
    }*/
    if (key == '0')
    {
      static bool surface0(true);
      surface0 = !surface0;
      scene->oceanScene()->setOceanSurfaceHeight(surface0 ? 0. : -1000.);
      return true;
    }
  }

  return false;
}


}
