#include <coral/viewer.h>
#include <coral/event_handler.h>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <chrono>
#include <coral/debug_msg.h>

int DebugMsg::indent{};

using namespace coral;
using std::chrono::system_clock;

Viewer::Viewer(Scene &scene) : scene(&scene)
{
  width = scene.parameters().width;
  height = scene.parameters().height;
  setUpViewInWindow( 150, 150, width, height, 0 );

  setSceneData(scene.getWorld());

  addEventHandler( new osgViewer::StatsHandler );
  addEventHandler( new osgGA::StateSetManipulator( getCamera()->getOrCreateStateSet() ) );

  event_handler = osg::make_ref<Viewer::EventHandler>(this);
  addEventHandler(event_handler);
  addEventHandler(scene.getWorld()->getEventHandler());
  addEventHandler(scene.oceanSurface()->getEventHandler());
  addEventHandler( new osgViewer::HelpHandler);

  getCamera()->setName("MainCamera");
  getCamera()->setClearColor(scene.underwaterColor());

  // init free-flying cam + default
  osg::Vec3 eye(scene.parameters().initialCameraPosition);
  free_manip->setHomePosition( eye, eye + osg::Vec3(0,20,0), osg::Vec3f(0,0,1) );
  free_manip->setVerticalAxisFixed(true);
  cam_is_free = true;
  setCameraManipulator(free_manip);

  // init tracking cam
  scene.getWorld()->addChild(cam_pose);
  tracking_manip->setTrackNode(cam_pose);
  tracking_manip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
  tracking_manip->setHomePosition({3,0,0}, {0,0,0}, {0,0,1});

  // virtual cam
  camera = osg::make_ref<osg::Camera>();
  scene.getWorld()->setScreenDims(width, height);
  camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
  camera->setRenderOrder(osg::Camera::POST_RENDER);
  camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
  camera->setClearMask(GL_DEPTH_BUFFER_BIT);

  getCamera()->addChild(camera);

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
    scene->getWorld()->setScreenDims(width, height);

  const auto z{getCameraManipulator()->getMatrix().getTrans().z()};
  const auto underwater(z < 0.);

  if(underwater && scene->parameters().depth_attn > 0.f)
  {
    getCamera()->setClearColor(scene->underwaterColor(1.f+z/scene->parameters().depth_attn));
  }

  if(underwater != prev_underwater)
  {
    scene->getWorld()->setOceanSurfaceHeight(underwater ? -0.05f : 0.f);
    prev_underwater = underwater;
  }

  advance(simTime);
  {
    [[maybe_unused]] const auto lock{scene->scoped_lock()};
    eventTraversal();
    updateTraversal();
  }
  renderingTraversals();
}

void Viewer::freeCamera()
{
  if(!cam_is_free)
  {
    cam_is_free = true;
    setCameraManipulator(free_manip, false);
  }
}

void Viewer::lockCamera(const osg::Matrixd &M)
{
  cam_pose->setMatrix(M);

  if(cam_is_free)
  {
    cam_is_free = false;
    setCameraManipulator(tracking_manip);
  }
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

void Viewer::changeMood(SceneType::Mood mood)
{
  const SceneType scene_type(mood);
  scene->changeScene(scene_type);
  getCamera()->setClearColor(scene->underwaterColor());
}


bool Viewer::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*)
{
  if (ea.getHandled()) return false;

  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
  {
    const auto key(ea.getKey());
    if(key == '1')
    {
      viewer->changeMood(SceneType::Mood::CLEAR);
      return true;
    }
    else if(key == '2')
    {
      viewer->changeMood( SceneType::Mood::DUSK );
      return true;
    }
    else if(key == '3' )
    {
      viewer->changeMood( SceneType::Mood::CLOUDY );
      return true;
    }
    else if(key == '4' )
    {
      viewer->changeMood( SceneType::Mood::NIGHT);
      return true;
    }
    else if (key == '0')
    {
      static bool surface0(true);
      surface0 = !surface0;
      viewer->scene->getWorld()->setOceanSurfaceHeight(surface0 ? 0. : -1000.);
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
  usage.addKeyboardMouseBinding("0","Toggle ocean surface");
}
