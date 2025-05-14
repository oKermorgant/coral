#include <coral/viewer.h>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgWidget/ViewerEventHandlers>
#include <chrono>
#include <coral/debug_msg.h>
#include <coral/scene_lock.h>

int DebugMsg::indent{};

using namespace coral;
using std::chrono::system_clock;

bool Viewer::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*)
{
  if (ea.getHandled())
    return false;

  if(ea.getEventType() != osgGA::GUIEventAdapter::KEYUP)
    return false;

  if(ea.getKey() != 'c')
    return false;

  if(viewer->camWidgets.empty())
    return false;
  const auto hide{viewer->camWidgets.front()->isVisible()};

  for(auto &widget: viewer->camWidgets)
  {
    if(hide)
      widget->hide();
    else
      widget->show();
  }

  return true;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void Viewer::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
  usage.addKeyboardMouseBinding("c","Toggle cam widgets");
}

Viewer::Viewer(Scene& scene):
	scene{&scene},
	ocean_scene{scene.getOceanScene()},
	width{ocean_scene->params.width},
	height{ocean_scene->params.height}
{
  const auto &params{ocean_scene->params};

  viewer->setUpViewInWindow( 150, 150, width, height, 0 );
  viewer->setSceneData(scene.getRoot());

  auto camera{viewer->getCamera()};

  if(params.stats_keys)
    viewer->addEventHandler( new osgViewer::StatsHandler );
  if(params.stateset_keys)
    viewer->addEventHandler( new osgGA::StateSetManipulator( camera->getOrCreateStateSet() ) );

  viewer->addEventHandler(scene.getEventHandler());
  viewer->addEventHandler( new osgViewer::HelpHandler);
  viewer->addEventHandler(new EventHandler(this));

  camera->setName("MainCamera");
  //scene->registerCamera(camera);

  // init free-flying cam + default
  osg::Vec3 eye(params.initialCameraPosition);
  free_manip->setHomePosition( eye, eye + osg::Vec3(0,20,0), osg::Vec3f(0,0,1) );
  free_manip->setVerticalAxisFixed(true);
  viewer->setCameraManipulator(free_manip);

  // init tracking cam
  scene.getScene()->addChild(cam_pose);
  tracking_manip->setTrackNode(cam_pose);
  tracking_manip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
  tracking_manip->setHomePosition({3,0,0}, {0,0,0}, {0,0,1});
  tracking_manip->setVerticalAxisFixed(true);

  // virtual cam
  handleResolution();


  wm = osg::make_ref<osgWidget::WindowManager>(viewer, width, height, 0xF0000000, 0);
  wm->setPointerFocusMode(osgWidget::WindowManager::PFM_SLOPPY);
  osg::ref_ptr < osg::Group > appgroup = new osg::Group();
  osg::ref_ptr < osg::Camera > appcamera = wm->createParentOrthoCamera();

  appgroup->addChild(appcamera);
  appgroup->addChild(scene.getRoot());
  viewer->addEventHandler(new osgWidget::ResizeHandler(wm, appcamera));
  viewer->setSceneData(appgroup);

  // FSAA
  osg::DisplaySettings::instance()->setNumMultiSamples(4);

  viewer->setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext);
  viewer->realize();

  osgViewer::Viewer::Windows windows;
  viewer->getWindows(windows);
  window = windows[0];
  window->setWindowName("Coral");
}

void Viewer::frame(double simTime)
{
  static auto prev_underwater(true);
  if(windowWasResized())
    handleResolution();

  const auto z{viewer->getCameraManipulator()->getMatrix().getTrans().z()};
  const auto zSurf{ocean_scene->getOceanSurfaceHeight()};
  const auto underwater(z < zSurf);

  if(underwater != prev_underwater)
  {
    ocean_scene->setOceanSurfaceHeight(underwater ? zSurf-0.05f : zSurf);
    prev_underwater = underwater;
  }

  static auto cnter{0};

  //std::cout << "Frame: " << cnter++ << std::endl;


  viewer->advance(simTime);
  {
    [[maybe_unused]] const auto lock{scene_lock()};
    viewer->eventTraversal();
    viewer->updateTraversal();    
  }
  viewer->renderingTraversals();

  //std::cout << "    Frame " << cnter << " done" << std::endl;
}

void Viewer::freeCamera()
{
  if(viewer->getCameraManipulator() != free_manip)
    viewer->setCameraManipulator(free_manip, false);
}

void Viewer::lockCamera(const osg::Matrix &M)
{
  cam_pose->setMatrix(M);

  if(viewer->getCameraManipulator() == free_manip)
    viewer->setCameraManipulator(tracking_manip);
}

bool Viewer::windowWasResized()
{
  static auto last_resize{system_clock::now()};
  static const auto delay{std::chrono::milliseconds(100)};
  static auto pending_resize{false};

  const auto now(system_clock::now());

  int _,w,h;
  window->getWindowRectangle(_,_,w,h);
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

void Viewer::addCamWidget(osgWidget::Window* widget, int width)
{
  camWidgets.emplace_back(widget);
  widget->setX(widget_offset);
  widget->setY(0);  
  wm->addChild(widget);
  widget->hide();
  widget_offset += width + 20;
}
