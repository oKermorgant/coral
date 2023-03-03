#include <coral/Scene.h>

#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osg/LightSource>
#include <osgOcean/ShaderManager>
#include <osgOcean/FFTOceanSurface>

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

Scene::Scene(const SceneParams &params) : params(params)
{
  initCoralResources();

  osgOcean::ShaderManager::instance().enableShaders(true);

  const SceneType scene_type(params.scene_type);

  loadCubeMapTextures( scene_type.cubemap);

  // Set up surface
  ocean_surface = osg::make_ref<osgOcean::FFTOceanSurface>(
        64, 256, 17,
        params.windDirection,
        params.windSpeed, params.depth,
        params.reflectionDamping,
        params.waveScale, params.isChoppy(),
        params.choppyFactor, 10.f, 256 );

  ocean_surface->setEnvironmentMap( cubemap.get() );
  ocean_surface->setFoamBottomHeight( 2.2f );
  ocean_surface->setFoamTopHeight( 3.0f );
  ocean_surface->enableCrestFoam( true );
  // Make the ocean surface track with the main camera position, giving the illusion
  // of an endless ocean surface.
  ocean_surface->enableEndlessOcean(false);

  // Set up ocean scene, with surface
  world = osg::make_ref<osgOcean::OceanScene>( ocean_surface.get() );
  //ocean_scene->createDefaultSceneShader();
  world->setLightID(0);
  world->enableReflections(true);
  world->enableRefractions(true);
  world->enableHeightmap(true);

  // Set the size of _oceanCylinder which follows the camera underwater.
  // This cylinder prevents the clear from being visible past the far plane
  // instead it will be the fog color.
  // The size of the cylinder should be changed according the size of the ocean surface.
  world->setCylinderSize( 1900.f, 4000.f );

  world->enableSilt(true);
  world->enableUnderwaterDOF(params.underwaterDof);
  world->enableUnderwaterScattering(true);
  world->enableDistortion(true);
  world->enableGlare(params.glare);
  world->setGlareAttenuation(0.8f);

  world->enableGodRays(params.godrays);
  world->setScreenDims(params.width, params.height);

  // create sky dome and add to ocean scene
  // set masks so it appears in reflected scene and normal scene
  skyDome = osg::make_ref<SkyDome>( 1900.f, 16, 16, cubemap.get() );
  osgOcean::OceanScene::setupMeshNode(skyDome);

  // add a pat to track the camera
  auto transform = osg::make_ref<osg::MatrixTransform>();
  transform->setDataVariance( osg::Object::DYNAMIC );
  transform->setMatrix( osg::Matrixf::translate( osg::Vec3f(0.f, 0.f, 0.f) ));
  transform->setCullCallback( new CameraTrackCallback );

  transform->addChild( skyDome.get() );

  world->addChild( transform );

  // Create and add fake texture for use with nodes without any texture
  // since the OceanScene default scene shader assumes that texture unit
  // 0 is used as a base texture map.
  auto image = osg::make_ref<osg::Image>();
  image->allocateImage( 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE );
  *(osg::Vec4ub*)image->data() = osg::Vec4ub( 0xFF, 0xFF, 0xFF, 0xFF );

  auto fakeTex = osg::make_ref<osg::Texture2D>( image );
  fakeTex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
  fakeTex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
  fakeTex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
  fakeTex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);

  auto stateset = world->getOrCreateStateSet();
  stateset->setTextureAttribute(0,fakeTex,osg::StateAttribute::ON);
  stateset->setTextureMode(0,GL_TEXTURE_1D,osg::StateAttribute::OFF);
  stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
  stateset->setTextureMode(0,GL_TEXTURE_3D,osg::StateAttribute::OFF);

  auto lightSource = osg::make_ref<osg::LightSource>();
  lightSource->setNodeMask(lightSource->getNodeMask() & ~CAST_SHADOW & ~RECEIVE_SHADOW);
  lightSource->setLocalStateSetModes();
  sun = lightSource->getLight();
  sun->setLightNum(0);

  // init overall scene
  changeScene(scene_type);

  world->addChild(lightSource);
  world->getOrCreateStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
  world->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
  world->getStateSet()->addUniform(new osg::Uniform("SLStex", 3));
  world->getStateSet()->addUniform(new osg::Uniform("SLStex2", 4));
  world->getStateSet()->addUniform( new osg::Uniform("stddev", 0.0f ) );
  world->getStateSet()->addUniform( new osg::Uniform("mean", 0.0f ) );
  world->getStateSet()->addUniform( new osg::Uniform("light", 1.f ) );

  world->setOceanSurfaceHeight(0);
}

void Scene::changeScene(const SceneType &scene_type)
{
  base_water_color = scene_type.waterFogColor;

  loadCubeMapTextures(scene_type.cubemap);
  skyDome->setCubeMap( cubemap.get() );

  ocean_surface->setEnvironmentMap( cubemap.get() );
  ocean_surface->setLightColor( scene_type.lightColor);

  world->setAboveWaterFog(0.0012f,  scene_type.fogColor );
  world->setUnderwaterFog(0.002f,   scene_type.waterFogColor );
  world->setUnderwaterDiffuse(  scene_type.underwaterDiffuse );
  world->setUnderwaterAttenuation(  scene_type.underwaterAttenuation );

  osg::Vec3f sunDir = - scene_type.sunPosition;
  sunDir.normalize();

  world->setSunDirection( sunDir );
  sun->setPosition( osg::Vec4f(-sunDir, 0.f) );
  sun->setAmbient( scene_type.sunAmbient);
  sun->setDiffuse( scene_type.sunDiffuse);
}

void Scene::loadCubeMapTextures( const std::string& dir )
{
  if(!cubemap)
  {
    cubemap = osg::make_ref<osg::TextureCubeMap>();
    cubemap->setInternalFormat(GL_RGBA);

    cubemap->setFilter( osg::Texture::MIN_FILTER,    osg::Texture::LINEAR_MIPMAP_LINEAR);
    cubemap->setFilter( osg::Texture::MAG_FILTER,    osg::Texture::LINEAR);
    cubemap->setWrap  ( osg::Texture::WRAP_S,        osg::Texture::CLAMP_TO_EDGE);
    cubemap->setWrap  ( osg::Texture::WRAP_T,        osg::Texture::CLAMP_TO_EDGE);
  }

  const auto path = "/textures/" + dir + "/";
  const std::vector<std::pair<osg::TextureCubeMap::Face, std::string>> filenames
  {{osg::TextureCubeMap::NEGATIVE_X,"west.png"},
    {osg::TextureCubeMap::POSITIVE_X, "east.png"},
    {osg::TextureCubeMap::NEGATIVE_Y, "up.png"},
    {osg::TextureCubeMap::POSITIVE_Y, "down.png"},
    {osg::TextureCubeMap::NEGATIVE_Z, "south.png"},
    {osg::TextureCubeMap::POSITIVE_Z, "north.png"}};

  for(const auto &[dir, filename]: filenames)
    cubemap->setImage(dir, osgDB::readImageFile(path + filename));
}

