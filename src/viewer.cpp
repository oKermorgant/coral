#include <coral/viewer.h>
#include <coral/event_handler.h>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <chrono>
#include <coral/debug_msg.h>

using namespace coral;
using std::chrono::system_clock;

Viewer::Viewer(osg::ref_ptr<Scene> scene)
  : viewer(new osgViewer::Viewer),
    scene(scene)
{
  width = scene->parameters().width;
  height = scene->parameters().height;
  viewer->setUpViewInWindow( 150, 150, width, height, 0 );

  viewer->setSceneData(scene->fullScene());

  viewer->addEventHandler( new osgViewer::StatsHandler );
  viewer->addEventHandler( new osgGA::StateSetManipulator( viewer->getCamera()->getOrCreateStateSet() ) );

  //viewer->addEventHandler(new EventHandler(scene.get(), this));
  viewer->addEventHandler(scene->getEventHandler());
  viewer->addEventHandler(scene->oceanScene()->getEventHandler());
  viewer->addEventHandler(scene->oceanSurface()->getEventHandler());

  viewer->addEventHandler( new osgViewer::HelpHandler );
  viewer->getCamera()->setName("MainCamera");

  // init free-flying cam + default
  osg::Vec3 eye(scene->parameters().initialCameraPosition);
  free_manip->setHomePosition( eye, eye + osg::Vec3(0,20,0), osg::Vec3f(0,0,1) );
  free_manip->setVerticalAxisFixed(true);
  cam_is_free = true;
  viewer->setCameraManipulator(free_manip);

  // init tracking cam
  scene->oceanScene()->addChild(cam_pose);
  tracking_manip->setTrackNode(cam_pose);
  tracking_manip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
  tracking_manip->setHomePosition({3,0,0}, {0,0,0}, {0,0,1});

  // virtual cam
  camera = new osg::Camera();
  //camera = viewer->getCamera();
  resizeWindow(width, height);
  camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
  camera->setRenderOrder(osg::Camera::POST_RENDER);
  camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
  camera->setClearMask(GL_DEPTH_BUFFER_BIT);

  viewer->getCamera()->addChild(camera);

  viewer->realize();
  osgViewer::Viewer::Windows windows;
  viewer->getWindows(windows);
  window = windows[0];
  window->setWindowName("Coral");
}

void Viewer::frame()
{
  static bool prev_above_water(true);
  [[maybe_unused]] const auto lock{scene->lock()};

  if(windowWasResized())
    resizeWindow(width, height);

  const auto above_water(viewer->getCameraManipulator()->getMatrix().getTrans().z() > 0.);
  if(above_water != prev_above_water)
  {
    scene->oceanScene()->setOceanSurfaceHeight(above_water ? 0.f : -0.05f);
    prev_above_water = above_water;
  }
  {
    //auto debug{DebugMsg("viewer:frame()")};
  viewer->frame();
  }
}

void Viewer::resizeWindow(int width, int height)
{
   scene->oceanScene()->setScreenDims(width, height);
   camera->setViewport(0,0,width,height);
   camera->setProjectionMatrixAsOrtho2D(0,width,0,height);
}

void Viewer::freeCamera()
{
  if(!cam_is_free)
  {
    cam_is_free = true;
    viewer->setCameraManipulator(free_manip, false);
  }
}

void Viewer::lockCamera(const osg::Matrixd &M)
{
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
