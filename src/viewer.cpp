#include <coral/viewer.h>
#include <coral/event_handler.h>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <chrono>

using namespace coral;
using std::chrono::system_clock;

Viewer::Viewer(Scene &scene)
  : viewer(new osgViewer::Viewer),
    ocean_scene(scene.oceanScene()),
    scene_mutex(scene.mutex())
{

  width = scene.parameters().width;
  height = scene.parameters().height;
  viewer->setUpViewInWindow( 150, 150, width, height, 0 );

  viewer->setSceneData(scene.fullScene());

  viewer->addEventHandler( new osgViewer::StatsHandler );
  viewer->addEventHandler( new osgGA::StateSetManipulator( viewer->getCamera()->getOrCreateStateSet() ) );

  viewer->addEventHandler(new EventHandler(&scene, viewer));
  viewer->addEventHandler(scene.oceanSurface()->getEventHandler());

  viewer->addEventHandler( new osgViewer::HelpHandler );
  viewer->getCamera()->setName("MainCamera");

  // init free-flying cam + default
  osg::Vec3 eye(scene.parameters().initialCameraPosition);
  free_manip->setHomePosition( eye, eye + osg::Vec3(0,20,0), osg::Vec3f(0,0,1) );
  cam_is_free = true;
  viewer->setCameraManipulator(free_manip);

  // init tracking cam
  scene.oceanScene()->addChild(cam_pose);
  tracking_manip->setTrackNode(cam_pose);
  tracking_manip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
  tracking_manip->setHomePosition({3,0,0}, {0,0,0}, {0,0,1});

  viewer->realize();
  osgViewer::Viewer::Windows windows;
  viewer->getWindows(windows);
  window = windows[0];
  window->setWindowName("Coral");
}

void Viewer::frame()
{
  static bool prev_above_water(true);
  scene_mutex->lock();

  if(windowWasResized())
    ocean_scene->setScreenDims(width, height);

  const auto above_water(viewer->getCameraManipulator()->getMatrix().getTrans().z() > 0.);
  if(above_water != prev_above_water)
  {
    if(above_water) ocean_scene->setOceanSurfaceHeight(0.);
    else            ocean_scene->setOceanSurfaceHeight(-0.05f);
    prev_above_water = above_water;
  }

  viewer->frame();
  scene_mutex->unlock();
}

void Viewer::freeCamera()
{
  if(!cam_is_free)
  {
    cam_is_free = true;
    viewer->setCameraManipulator(free_manip, false);
  }
}

void Viewer::lockCamera(osg::Vec3d trans, osg::Quat q)
{
  osg::Matrixd M(q);
  M.setTrans(trans);

  cam_pose->setMatrix(M);

  if(cam_is_free)
  {
    cam_is_free = false;
    viewer->setCameraManipulator(tracking_manip);
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
