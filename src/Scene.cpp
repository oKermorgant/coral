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

  scene = new osg::Group;
  loadCubeMapTextures( scene_type.cubemap);

  // Set up surface
  ocean_surface =
      new osgOcean::FFTOceanSurface(
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

  // Set up ocean scene, add surface
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

  ocean_scene->enableSilt(true);
  ocean_scene->enableUnderwaterDOF(params.underwaterDof);
  ocean_scene->enableUnderwaterScattering(true);
  ocean_scene->enableDistortion(true);
  ocean_scene->enableGlare(params.glare);
  ocean_scene->setGlareAttenuation(0.8f);

  ocean_scene->enableGodRays(params.godrays);
  ocean_scene->setScreenDims(params.width, params.height);

  // create sky dome and add to ocean scene
  // set masks so it appears in reflected scene and normal scene
  skyDome = new SkyDome( 1900.f, 16, 16, cubemap.get() );
  osgOcean::OceanScene::setupMeshNode(skyDome);

  // add a pat to track the camera
  osg::MatrixTransform* transform = new osg::MatrixTransform;
  transform->setDataVariance( osg::Object::DYNAMIC );
  transform->setMatrix( osg::Matrixf::translate( osg::Vec3f(0.f, 0.f, 0.f) ));
  transform->setCullCallback( new CameraTrackCallback );

  transform->addChild( skyDome.get() );

  ocean_scene->addChild( transform );

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

  auto stateset = ocean_scene->getOrCreateStateSet();
  stateset->setTextureAttribute(0,fakeTex,osg::StateAttribute::ON);
  stateset->setTextureMode(0,GL_TEXTURE_1D,osg::StateAttribute::OFF);
  stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
  stateset->setTextureMode(0,GL_TEXTURE_3D,osg::StateAttribute::OFF);

  auto lightSource = new osg::LightSource;
  lightSource->setNodeMask(lightSource->getNodeMask() & ~CAST_SHADOW & ~RECEIVE_SHADOW);
  lightSource->setLocalStateSetModes();
  light = lightSource->getLight();
  light->setLightNum(0);

  // init overall scene
  changeScene(scene_type);

  scene->addChild( lightSource );
  scene->addChild( ocean_scene.get() );

  root = scene;
  //root = new osg::Group;

  root->getOrCreateStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
  root->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
  root->getStateSet()->addUniform(new osg::Uniform("SLStex", 3));
  root->getStateSet()->addUniform(new osg::Uniform("SLStex2", 4));
  root->getStateSet()->addUniform( new osg::Uniform("stddev", 0.0f ) );
  root->getStateSet()->addUniform( new osg::Uniform("mean", 0.0f ) );
  root->getStateSet()->addUniform( new osg::Uniform("light", 1.f ) );

  ocean_scene->setOceanSurfaceHeight(0);

  if(root.get() != scene.get())
    root->addChild(scene);
}

void Scene::changeScene(const SceneType &scene_type)
{
  mood = scene_type.mood;

  loadCubeMapTextures(scene_type.cubemap);
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
  light->setAmbient( scene_type.sunAmbient);
  light->setDiffuse( scene_type.sunDiffuse);
}

void Scene::loadCubeMapTextures( const std::string& dir )
{
  if(!cubemap)
  {
    cubemap = new osg::TextureCubeMap;
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

