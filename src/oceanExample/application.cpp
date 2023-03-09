/*
* This source file is part of the osgOcean library
*
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <string>
#include <vector>
#include <iostream>

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>

#include <osgViewer/ViewerEventHandlers>

#include <osgGA/StateSetManipulator>

#include <osg/Notify>

#include <osgDB/ReadFile>



#include <osgOcean/Version>
#include <osgOcean/ShaderManager>

#include "SceneEventHandler.h"
#include "Scene.h"
//#include "TextHUD.h"


#include <coral/resource_helpers.h>
#include <coral/OceanScene.h>

int main(int argc, char *argv[])
{
  osg::notify(osg::NOTICE) << "osgOcean " << osgOceanGetVersion() << std::endl << std::endl;

  osg::ArgumentParser arguments(&argc,argv);
  arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
  arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is an example of osgOcean.");
  arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] ...");
  arguments.getApplicationUsage()->addCommandLineOption("--windx <x>","Wind X direction. Default 1.1");
  arguments.getApplicationUsage()->addCommandLineOption("--windy <y>","Wind Y direction. Default 1.1");
  arguments.getApplicationUsage()->addCommandLineOption("--windSpeed <speed>","Wind speed. Default: 12");
  arguments.getApplicationUsage()->addCommandLineOption("--depth <depth>","Depth. Default: 10000");
  arguments.getApplicationUsage()->addCommandLineOption("--isNotChoppy","Set the waves not choppy (by default they are).");
  arguments.getApplicationUsage()->addCommandLineOption("--choppyFactor <factor>","How choppy the waves are. Default: 2.5");
  arguments.getApplicationUsage()->addCommandLineOption("--crestFoamHeight <height>","How high the waves need to be before foam forms on the crest. Default: 2.2 ");
  arguments.getApplicationUsage()->addCommandLineOption("--oceanSurfaceHeight <z>","Z position of the ocean surface in world coordinates. Default: 0.0");
  arguments.getApplicationUsage()->addCommandLineOption("--testCollision","Test ocean surface collision detection by making a boat float on its surface.");
  arguments.getApplicationUsage()->addCommandLineOption("--disableShaders","Disable use of shaders for the whole application. Also disables most visual effects as they depend on shaders.");

  coral::initCoralResources();

  unsigned int helpType = 0;
  if ((helpType = arguments.readHelpType()))
  {
    arguments.getApplicationUsage()->write(std::cout, helpType);
    return 1;
  }

  // report any errors if they have occurred when parsing the program arguments.
  if (arguments.errors())
  {
    arguments.writeErrorMessages(std::cout);
    return 1;
  }

  float windx = 1.1f, windy = 1.1f;
  while (arguments.read("--windx", windx));
  while (arguments.read("--windy", windy));
  osg::Vec2f windDirection(windx, windy);

  float windSpeed = 12.f;
  while (arguments.read("--windSpeed", windSpeed));

  float depth = 1000.f;
  while (arguments.read("--depth", depth));

  float reflectionDamping = 0.35f;
  while (arguments.read("--reflectionDamping", reflectionDamping));

  float scale = 1e-8;
  while (arguments.read("--waveScale", scale ) );

  bool isChoppy = true;
  while (arguments.read("--isNotChoppy")) isChoppy = false;

  float choppyFactor = 2.5f;
  while (arguments.read("--choppyFactor", choppyFactor));
  choppyFactor = -choppyFactor;

  float crestFoamHeight = 2.2f;
  while (arguments.read("--crestFoamHeight", crestFoamHeight));

  double oceanSurfaceHeight = 0.0f;
  while (arguments.read("--oceanSurfaceHeight", oceanSurfaceHeight));

  bool useVBO = false;
  if (arguments.read("--vbo")) useVBO = true;

  osg::Vec3 initialCameraPosition(0,0,20);
  while (arguments.read("--initialCameraPosition", initialCameraPosition.x(), initialCameraPosition.y(), initialCameraPosition.z()));

  osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);

  // any option left unread are converted into errors to write out later.
  arguments.reportRemainingOptionsAsUnrecognized();

  // report any errors if they have occurred when parsing the program arguments.
  if (arguments.errors())
  {
    arguments.writeErrorMessages(std::cout);
    return 1;
  }

  //------------------------------------------------------------------------
  // Set up the scene
  //------------------------------------------------------------------------

  std::string terrain_shader_basename = "terrain";


  osgOcean::ShaderManager::instance().enableShaders(true);
  osg::ref_ptr<Scene> scene = new Scene(windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, crestFoamHeight, useVBO, terrain_shader_basename);

  osg::ref_ptr<osg::Group> root = new osg::Group;


  scene->getOceanScene()->setOceanSurfaceHeight(oceanSurfaceHeight);

  root->addChild( scene->getScene() );

  if (loadedModel.valid())
  {
    loadedModel->setNodeMask( scene->getOceanScene()->getNormalSceneMask()    |
                              scene->getOceanScene()->getReflectedSceneMask() |
                              scene->getOceanScene()->getRefractedSceneMask() |
                              CAST_SHADOW | RECEIVE_SHADOW );

    scene->getOceanScene()->addChild(loadedModel.get());
  }


  //------------------------------------------------------------------------
  // Set up the viewer
  //------------------------------------------------------------------------

  osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
  //osgViewer::View* viewer = viewer;

  // Otherwise, a window with a single view.
  viewer->setUpViewInWindow( 150, 150, 1024, 768, 0 );

  viewer->setSceneData( root.get() );

  viewer->addEventHandler( new osgViewer::StatsHandler );
  viewer->addEventHandler( new osgGA::StateSetManipulator( viewer->getCamera()->getOrCreateStateSet() ) );

  osg::ref_ptr<TextHUD> hud = new TextHUD;
  // Add the HUD to the main view (if compositeViewer == true there will be a second one)
  viewer->getCamera()->addChild( hud->getHudCamera() );

  viewer->addEventHandler(scene->getOceanSceneEventHandler());
  viewer->addEventHandler(scene->getOceanSurface()->getEventHandler());

  viewer->addEventHandler( new SceneEventHandler(scene.get(),
                                              // hud.get(),
                                               viewer, initialCameraPosition ) );
  viewer->addEventHandler( new osgViewer::HelpHandler );
  viewer->getCamera()->setName("MainCamera");

  viewer->realize();

  while(!viewer->done())
  {
    viewer->frame();
  }

  return 0;
}
