#include <coral/Scene.h>
#include <coral/ScopedTimer.h>

#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osg/LightSource>
#include <osgOcean/ShaderManager>

#include <coral/resource_helpers.h>

// ----------------------------------------------------
//               Camera Track Callback
// ----------------------------------------------------

using namespace coral;

class CameraTrackCallback: public osg::NodeCallback
{
public:
  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
      osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
      osg::Vec3f centre,up,eye;
      // get MAIN camera eye,centre,up
      cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye,centre,up);
      // update position
      osg::MatrixTransform* mt = static_cast<osg::MatrixTransform*>(node);
      mt->setMatrix( osg::Matrix::translate( eye.x(), eye.y(), mt->getMatrix().getTrans().z() ) );
    }

    traverse(node, nv);
  }
};

// ----------------------------------------------------
//                       Scene 
// ----------------------------------------------------

Scene::Scene(const SceneParams &params, const std::string& terrain_shader_basename )
{
  osgOcean::ShaderManager::instance().enableShaders(true);

  {
    ScopedTimer buildSceneTimer("Building scene... \n", osg::notify(osg::NOTICE));

    scene = new osg::Group;

    {
      ScopedTimer cubemapTimer("  . Loading cubemaps: ", osg::notify(osg::NOTICE));
      cubemap = loadCubeMapTextures( scene_type.cubemap );
    }

    // Set up surface
    {
      ScopedTimer oceanSurfaceTimer("  . Generating ocean surface: ", osg::notify(osg::NOTICE));

      if (params.useVBO)
      {
        ocean_surface =
            new osgOcean::FFTOceanSurfaceVBO(
              64, 256, 17,
              params.windDirection,
              params.windSpeed, params.depth,
              params.reflectionDamping,
              params.waveScale, params.isChoppy,
              params.choppyFactor, 10.f, 256 );
      }
      else
      {
        ocean_surface =
            new osgOcean::FFTOceanSurface(
              64, 256, 17,
              params.windDirection,
              params.windSpeed, params.depth,
              params.reflectionDamping,
              params.waveScale, params.isChoppy,
              params.choppyFactor, 10.f, 256 );
      }

      ocean_surface->setEnvironmentMap( cubemap.get() );
      ocean_surface->setFoamBottomHeight( 2.2f );
      ocean_surface->setFoamTopHeight( 3.0f );
      ocean_surface->enableCrestFoam( true );
      ocean_surface->setLightColor( scene_type.lightColor );
      // Make the ocean surface track with the main camera position, giving the illusion
      // of an endless ocean surface.
      ocean_surface->enableEndlessOcean(true);
    }

    // Set up ocean scene, add surface
    {
      ScopedTimer oceanSceneTimer("  . Creating ocean scene: ", osg::notify(osg::NOTICE));
      osg::Vec3f sunDir = -scene_type.sunPosition;
      sunDir.normalize();

      ocean_scene = new osgOcean::OceanScene( ocean_surface.get() );
      //ocean_scene->createDefaultSceneShader();
      ocean_scene->setLightID(0);
      ocean_scene->enableReflections(true);
      ocean_scene->enableRefractions(true);
      ocean_scene->enableHeightmap(true);

      // Set the size of _oceanCylinder which follows the camera underwater.
      // This cylinder prevents the clear from being visible past the far plane
      // instead it will be the fog color.
      // The size of the cylinder should be changed according the size of the ocean surface.
      ocean_scene->setCylinderSize( 1900.f, 4000.f );

      ocean_scene->setAboveWaterFog(0.0012f, scene_type.fogColor );
      ocean_scene->setUnderwaterFog(0.002f,  scene_type.waterFogColor );
      ocean_scene->setUnderwaterDiffuse( scene_type.underwaterDiffuse );
      ocean_scene->setUnderwaterAttenuation( scene_type.underwaterAttenuation);

      ocean_scene->setSunDirection( sunDir );
      ocean_scene->enableGodRays(true);
      ocean_scene->enableSilt(true);
      ocean_scene->enableUnderwaterDOF(true);
      ocean_scene->enableUnderwaterScattering(true);
      ocean_scene->enableDistortion(true);
      ocean_scene->enableGlare(true);
      ocean_scene->setGlareAttenuation(0.8f);

      // create sky dome and add to ocean scene
      // set masks so it appears in reflected scene and normal scene
      skyDome = new SkyDome( 1900.f, 16, 16, cubemap.get() );
      skyDome->setNodeMask( ocean_scene->getReflectedSceneMask() |
                            ocean_scene->getNormalSceneMask()    |
                            ocean_scene->getRefractedSceneMask());

      // add a pat to track the camera
      osg::MatrixTransform* transform = new osg::MatrixTransform;
      transform->setDataVariance( osg::Object::DYNAMIC );
      transform->setMatrix( osg::Matrixf::translate( osg::Vec3f(0.f, 0.f, 0.f) ));
      transform->setCullCallback( new CameraTrackCallback );

      transform->addChild( skyDome.get() );

      ocean_scene->addChild( transform );

      {
        // Create and add fake texture for use with nodes without any texture
        // since the OceanScene default scene shader assumes that texture unit
        // 0 is used as a base texture map.
        osg::Image * image = new osg::Image;
        image->allocateImage( 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE );
        *(osg::Vec4ub*)image->data() = osg::Vec4ub( 0xFF, 0xFF, 0xFF, 0xFF );

        osg::Texture2D* fakeTex = new osg::Texture2D( image );
        fakeTex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
        fakeTex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
        fakeTex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
        fakeTex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);

        osg::StateSet* stateset = ocean_scene->getOrCreateStateSet();
        stateset->setTextureAttribute(0,fakeTex,osg::StateAttribute::ON);
        stateset->setTextureMode(0,GL_TEXTURE_1D,osg::StateAttribute::OFF);
        stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
        stateset->setTextureMode(0,GL_TEXTURE_3D,osg::StateAttribute::OFF);
      }

    }

    {
      ScopedTimer islandsTimer("  . Loading islands: ", osg::notify(osg::NOTICE));
      osg::ref_ptr<osg::Node> islandModel = loadIslands(terrain_shader_basename);

      if( islandModel.valid() )
      {
        island_switch = new osg::Switch;
        island_switch->addChild( islandModel.get(), true );

        island_switch->setNodeMask( ocean_scene->getNormalSceneMask()    |
                                    ocean_scene->getReflectedSceneMask() |
                                    ocean_scene->getRefractedSceneMask() |
                                    ocean_scene->getHeightmapMask()      |
                                    RECEIVE_SHADOW);

        ocean_scene->addChild( island_switch.get() );
      }
    }

    {
      ScopedTimer lightingTimer("  . Setting up lighting: ", osg::notify(osg::NOTICE));
      osg::LightSource* lightSource = new osg::LightSource;
      lightSource->setNodeMask(lightSource->getNodeMask() & ~CAST_SHADOW & ~RECEIVE_SHADOW);
      lightSource->setLocalStateSetModes();

      light = lightSource->getLight();
      light->setLightNum(0);
      light->setAmbient( osg::Vec4d(0.3f, 0.3f, 0.3f, 1.0f ));
      light->setDiffuse( scene_type.sunDiffuse );
      light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );

      osg::Vec3f direction(scene_type.sunPosition);
      direction.normalize();
      light->setPosition( osg::Vec4f(direction, 0.0) );  // directional light

      scene->addChild( lightSource );
      scene->addChild( ocean_scene.get() );
      //_scene->addChild( sunDebug(_sunPositions[CLOUDY]) );
    }

    osg::notify(osg::NOTICE) << "complete.\nTime Taken: ";
  }
}

void Scene::changeScene( SceneType::SCENE_TYPE type )
{
  scene_type.switchTo(type);

  cubemap = loadCubeMapTextures(scene_type.cubemap);
  skyDome->setCubeMap( cubemap.get() );

  ocean_surface->setEnvironmentMap( cubemap.get() );
  ocean_surface->setLightColor( scene_type.lightColor);

  ocean_scene->setAboveWaterFog(0.0012f,  scene_type.fogColor );
  ocean_scene->setUnderwaterFog(0.002f,   scene_type.waterFogColor );
  ocean_scene->setUnderwaterDiffuse(  scene_type.underwaterDiffuse );
  ocean_scene->setUnderwaterAttenuation(  scene_type.underwaterAttenuation );

  osg::Vec3f sunDir = - scene_type.sunPosition;
  sunDir.normalize();

  ocean_scene->setSunDirection( sunDir );
  light->setPosition( osg::Vec4f(-sunDir, 0.f) );
  light->setDiffuse( scene_type.sunDiffuse);

  if(island_switch.valid() )
  {
    if(type == SceneType::CLEAR || type == SceneType::CLOUDY)
      island_switch->setAllChildrenOn();
    else
      island_switch->setAllChildrenOff();
  }
}

#define USE_CUSTOM_SHADER

// Load the islands model
// Here we attach a custom shader to the model.
// This shader overrides the default shader applied by OceanScene but uses uniforms applied by OceanScene.
// The custom shader is needed to add multi-texturing and bump mapping to the terrain.
osg::Node* Scene::loadIslands(const std::string& terrain_shader_basename)
{

  //  osgDB::Registry::instance()->getDataFilePathList().push_back("island");
  const std::string filename = "island/islands.ive";
  osg::ref_ptr<osg::Node> island = osgDB::readNodeFile(filename);

  if(!island.valid()){
    osg::notify(osg::WARN) << "Could not find: " << filename << std::endl;
    return NULL;
  }

#ifdef USE_CUSTOM_SHADER
  const std::string terrain_vertex   = terrain_shader_basename + ".vert";
  const std::string terrain_fragment = terrain_shader_basename + ".frag";

  osg::Program* program = osgOcean::ShaderManager::instance().createProgram("terrain", terrain_vertex, terrain_fragment, "", "" );
  if(program) program->addBindAttribLocation("aTangent", 6);

#endif
  island->setNodeMask( ocean_scene->getNormalSceneMask()    |
                       ocean_scene->getReflectedSceneMask() |
                       ocean_scene->getRefractedSceneMask() |
                       ocean_scene->getHeightmapMask()      |
                       RECEIVE_SHADOW);
  island->getStateSet()->addUniform( new osg::Uniform( "uTextureMap", 0 ) );

#ifdef USE_CUSTOM_SHADER
  island->getOrCreateStateSet()->setAttributeAndModes(program,osg::StateAttribute::ON);
  island->getStateSet()->addUniform( new osg::Uniform( "uOverlayMap", 1 ) );
  island->getStateSet()->addUniform( new osg::Uniform( "uNormalMap",  2 ) );
#endif
  osg::PositionAttitudeTransform* islandpat = new osg::PositionAttitudeTransform;
  islandpat->setPosition(osg::Vec3f( -island->getBound().center()+osg::Vec3f(80, 100, 58) ) );
  islandpat->setScale( osg::Vec3f(4.f, 4.f, 3.f ) );
  islandpat->addChild(island.get());

  return islandpat;
}

osg::ref_ptr<osg::TextureCubeMap> Scene::loadCubeMapTextures( const std::string& dir )
{
  static const std::map<osg::TextureCubeMap::Face, std::string> filenames
  {{osg::TextureCubeMap::NEGATIVE_X,"west"},
  {osg::TextureCubeMap::POSITIVE_X, "east"},
  {osg::TextureCubeMap::NEGATIVE_Y, "up"},
  {osg::TextureCubeMap::POSITIVE_Y, "down"},
  {osg::TextureCubeMap::NEGATIVE_Z, "south"},
  {osg::TextureCubeMap::POSITIVE_Z, "north"}};

  osg::ref_ptr<osg::TextureCubeMap> cubeMap = new osg::TextureCubeMap;
  cubeMap->setInternalFormat(GL_RGBA);

  cubeMap->setFilter( osg::Texture::MIN_FILTER,    osg::Texture::LINEAR_MIPMAP_LINEAR);
  cubeMap->setFilter( osg::Texture::MAG_FILTER,    osg::Texture::LINEAR);
  cubeMap->setWrap  ( osg::Texture::WRAP_S,        osg::Texture::CLAMP_TO_EDGE);
  cubeMap->setWrap  ( osg::Texture::WRAP_T,        osg::Texture::CLAMP_TO_EDGE);

  const auto path = "/textures/" + dir + "/";

  for(const auto &[dir, filename]: filenames)
  {
    cubeMap->setImage(dir, osgDB::readImageFile(path + filename + ".png") );
  }

  return cubeMap;
}

osg::Geode* Scene::sunDebug( const osg::Vec3f& position )
{
  osg::ShapeDrawable* sphereDraw = new osg::ShapeDrawable( new osg::Sphere( position, 15.f ) );
  sphereDraw->setColor(osg::Vec4f(1.f,0.f,0.f,1.f));

  osg::Geode* sphereGeode = new osg::Geode;
  sphereGeode->addDrawable( sphereDraw );

  return sphereGeode;
}
