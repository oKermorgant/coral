#include <coral/viewer.h>
#include <coral/SceneEventHandler.h>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

using namespace coral;


Viewer::Viewer(SceneInterface &interface)
{
  scene_mutex = interface.mutex();
  // Otherwise, a window with a single view.
  viewer = new osgViewer::Viewer;
  viewer->setUpViewInWindow( 150, 150, 1024, 768, 0 );

  viewer->setSceneData(interface.world());

  viewer->addEventHandler( new osgViewer::StatsHandler );
  viewer->addEventHandler( new osgGA::StateSetManipulator( viewer->getCamera()->getOrCreateStateSet() ) );

  hud = new TextHUD;
  viewer->getCamera()->addChild( hud->getHudCamera() );

  viewer->addEventHandler(interface.oceanScene()->getEventHandler());
  viewer->addEventHandler(interface.oceanSurface()->getEventHandler());

  viewer->addEventHandler( new SceneEventHandler(interface.fullScene(), hud.get(), viewer,
                                                 interface.sceneParams().initialCameraPosition ) );
  viewer->addEventHandler( new osgViewer::HelpHandler );
  viewer->getCamera()->setName("MainCamera");
  viewer->setCameraManipulator(NULL);

  viewer->realize();
}
