#include <coral/viewer.h>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <chrono>
#include <coral/debug_msg.h>
#include <coral/scene_lock.h>

int DebugMsg::indent{};

using namespace coral;
using std::chrono::system_clock;

Viewer::Viewer(OceanScene *scene) :
  scene(scene),
  width{scene->getParams().width},
  height{scene->getParams().height},
  event_handler{osg::make_ref<Viewer::EventHandler>(this)}
{
  setUpViewInWindow( 150, 150, width, height, 0 );
  setSceneData(scene->getRoot());

  if(scene->getParams().stats_keys)
    addEventHandler( new osgViewer::StatsHandler );
  if(scene->getParams().stateset_keys)
    addEventHandler( new osgGA::StateSetManipulator( getCamera()->getOrCreateStateSet() ) );

  addEventHandler(event_handler);
  addEventHandler(scene->getEventHandler(scene->getParams().surface_keys));
  addEventHandler( new osgViewer::HelpHandler);

  getCamera()->setName("MainCamera");
  scene->registerCamera(getCamera());

  // init free-flying cam + default
  osg::Vec3 eye(scene->getParams().initialCameraPosition);
  free_manip->setHomePosition( eye, eye + osg::Vec3(0,20,0), osg::Vec3f(0,0,1) );
  free_manip->setVerticalAxisFixed(true);
  setCameraManipulator(free_manip);

  // init tracking cam
  scene->addChild(cam_pose);
  tracking_manip->setTrackNode(cam_pose);
  tracking_manip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
  tracking_manip->setHomePosition({3,0,0}, {0,0,0}, {0,0,1});
  tracking_manip->setVerticalAxisFixed(true);

  // virtual cam
  camera = osg::make_ref<osg::Camera>();
  scene->fitToSize(width, height);
  camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF);
  camera->setRenderOrder(osg::Camera::POST_RENDER);
  camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
  camera->setClearMask(GL_DEPTH_BUFFER_BIT);

  getCamera()->addChild(camera);

  osg::DisplaySettings::instance()->setNumMultiSamples(16);
  realize();

  osgViewer::Viewer::Windows windows;
  getWindows(windows);
  window = windows[0];
  window->setWindowName("Coral");

}

void Viewer::frame(double simTime)
{
  static auto prev_underwater(true);
  if(windowWasResized())
    scene->fitToSize(width, height);

  const auto z{getCameraManipulator()->getMatrix().getTrans().z()};
  const auto zSurf{scene->getOceanSurfaceHeight()};
  const auto underwater(z < zSurf);

  if(underwater != prev_underwater)
  {
    scene->setOceanSurfaceHeight(underwater ? zSurf-0.05f : zSurf);
    prev_underwater = underwater;
  }

  advance(simTime);
  {
    [[maybe_unused]] const auto lock{coral_lock()};
    eventTraversal();
    updateTraversal();
  }
  renderingTraversals();
}

void Viewer::freeCamera()
{
  if(getCameraManipulator() != free_manip)
    setCameraManipulator(free_manip, false);
}

void Viewer::lockCamera(const osg::Matrix &M)
{
  cam_pose->setMatrix(M);

  if(getCameraManipulator() == free_manip)
    setCameraManipulator(tracking_manip);
}

bool Viewer::windowWasResized()
{
  static int x,y,w,h;
  static auto last_resize{system_clock::now()};
  static const auto delay{std::chrono::milliseconds(100)};
  static auto pending_resize{false};

  const auto now(system_clock::now());

  window->getWindowRectangle(x,y,w,h);
  if(w != width || h != height)
  {
    width = w;
    height = h;
    last_resize = now;
    pending_resize = true;
    return false;
  }

  if(pending_resize && now - last_resize > delay)
  {
    pending_resize = false;
    return true;
  }
  return false;
}

void Viewer::changeMood(Weather::Mood mood)
{
  scene->changeMood(mood);
  getCamera()->setClearColor(scene->scaleUnderwaterColor());
}


bool Viewer::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*)
{
  if (ea.getHandled()) return false;

  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
  {
    const auto key(ea.getKey());
    if(key == '1')
    {
      viewer->changeMood(Weather::Mood::CLEAR);
      return true;
    }
    else if(key == '2')
    {
      viewer->changeMood( Weather::Mood::DUSK );
      return true;
    }
    else if(key == '3' )
    {
      viewer->changeMood( Weather::Mood::CLOUDY );
      return true;
    }
    else if(key == '4' )
    {
      viewer->changeMood( Weather::Mood::NIGHT);
      return true;
    }
    else if(key == '5' )
    {
      viewer->changeMood( Weather::Mood::CUSTOM);
      return true;
    }
  }
  return false;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void Viewer::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
  usage.addKeyboardMouseBinding("1","Select scene \"Clear Blue Sky\"");
  usage.addKeyboardMouseBinding("2","Select scene \"Dusk\"");
  usage.addKeyboardMouseBinding("3","Select scene \"Pacific Cloudy\"");
  usage.addKeyboardMouseBinding("4","Select scene \"Night\"");
  usage.addKeyboardMouseBinding("5","Select scene \"Custom\"");
}
