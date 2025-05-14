#include <coral/resource_helpers.h>
#include <coral/scene.h>
#include <osg/LightSource>

using namespace coral;

class CameraTrackCallback : public osg::NodeCallback
{
public:
  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    if (nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
      osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
      osg::Vec3f centre, up, eye;
      // get MAIN camera eye,centre,up
      cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye, centre, up);
      // update position
      osg::MatrixTransform* mt = static_cast<osg::MatrixTransform*>(node);
      mt->setMatrix(osg::Matrix::translate(eye.x(), eye.y(), mt->getMatrix().getTrans().z()));
    }

    traverse(node, nv);
  }
};

bool Scene::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*)
{
  if (ea.getHandled())
    return false;

  if(ea.getEventType() != osgGA::GUIEventAdapter::KEYUP)
    return false;

  const auto key(ea.getKey());
  auto ocean_scene{scene->getOceanScene()};
  auto fftSurface{scene->getSurface()};
  switch (key)
  {
    case '1':
      scene->changeMood(Weather::Mood::CLEAR);
      break;
    case '2':
      scene->changeMood(Weather::Mood::DUSK);
      break;
    case '3':
      scene->changeMood(Weather::Mood::CLOUDY);
      break;
    case '4':
      scene->changeMood(Weather::Mood::NIGHT);
      break;
    case '5':
      scene->changeMood(Weather::Mood::CUSTOM);
      break;
    case 'r':
      ocean_scene->switchParam(ocean_scene->params.reflections);
      break;
    // Refractions
    case 'R':
      ocean_scene->switchParam(ocean_scene->params.refractions);
      break;
    // DOF
    case 'o':
      ocean_scene->switchParam(ocean_scene->params.underwaterDOF);
      break;	  
    // Glare
    case 'g':
      ocean_scene->switchParam(ocean_scene->params.glare);
      break;
    // God rays
    case 'G':
      ocean_scene->switchParam(ocean_scene->params.godrays);
      break;
    // Silt
    case 'u':
      ocean_scene->switchParam(ocean_scene->params.silt);
      break;
    // Underwater scattering
    case 'd':
      ocean_scene->switchParam(ocean_scene->params.underwaterScattering);
      break;
    // auto dirty
    case osgGA::GUIEventAdapter::KEY_BackSpace:
      ocean_scene->dirty();
      break;
    // Height map
    case 'H':
      ocean_scene->switchParam(ocean_scene->params.heightmap);
      fftSurface->dirty();      // Make it reload shaders.
      break;
    // Ocean surface height
    case osgGA::GUIEventAdapter::KEY_Page_Up:
      ocean_scene->setOceanSurfaceHeight(ocean_scene->getOceanSurfaceHeight() + .5);
      break;
    case osgGA::GUIEventAdapter::KEY_Page_Down:
      ocean_scene->setOceanSurfaceHeight(ocean_scene->getOceanSurfaceHeight() - .5);
      break;
    case osgGA::GUIEventAdapter::KEY_Home:
      ocean_scene->setOceanVisible(true);
      ocean_scene->setOceanSurfaceHeight(0.);
      break;
    case osgGA::GUIEventAdapter::KEY_End:
      ocean_scene->setOceanVisible(false);
      ocean_scene->setOceanSurfaceHeight(-1000.);
      break;
    // Crest foam
    case 'f' :
      fftSurface->enableCrestFoam(!fftSurface->isCrestFoamEnabled());
      break;
    // isChoppy
    case 'p' :
      fftSurface->setIsChoppy(!fftSurface->isChoppy(), true);
      break;
    // Wind speed + 0.5
    case '+':
      fftSurface->setWindSpeed(fftSurface->getWindSpeed() + 0.5, true);
      break;
    // Wind speed - 0.5
    case '-':
      fftSurface->setWindSpeed(fftSurface->getWindSpeed() - 0.5, true);
      break;
    // Scale factor + 1e-9
    case 'K' :
      fftSurface->setWaveScaleFactor(fftSurface->getWaveScaleFactor()+(1e-9), true);
      break;
    // Scale factor - 1e-9
    case 'k' :
      fftSurface->setWaveScaleFactor(fftSurface->getWaveScaleFactor()-(1e-9), true);
      break;
    default:
      return false;
  }
  return true;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void Scene::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
  usage.addKeyboardMouseBinding("1","Select scene \"Clear Blue Sky\"");
  usage.addKeyboardMouseBinding("2","Select scene \"Dusk\"");
  usage.addKeyboardMouseBinding("3","Select scene \"Pacific Cloudy\"");
  usage.addKeyboardMouseBinding("4","Select scene \"Night\"");
  usage.addKeyboardMouseBinding("5","Select scene \"Custom\"");
  usage.addKeyboardMouseBinding("r","Toggle reflections (above water)");
  usage.addKeyboardMouseBinding("R","Toggle refractions (underwater)");
  usage.addKeyboardMouseBinding("o","Toggle Depth of Field (DOF) (underwater)");
  usage.addKeyboardMouseBinding("g","Toggle Glare (above water)");
  usage.addKeyboardMouseBinding("Backspace","Dirty the scene");
  usage.addKeyboardMouseBinding("G","Toggle God rays (underwater)");
  usage.addKeyboardMouseBinding("u","Toggle silt (underwater)");
  usage.addKeyboardMouseBinding("d","Toggle scattering (underwater)");
  usage.addKeyboardMouseBinding("H","Toggle Height lookup for shoreline foam and sine shape (above water)");
  usage.addKeyboardMouseBinding("Page Up","Raise ocean surface");
  usage.addKeyboardMouseBinding("Page Down","Lower ocean surface");
  usage.addKeyboardMouseBinding("End","Remove ocean surface");
  usage.addKeyboardMouseBinding("Home","Reset ocean surface at z=0");

  usage.addKeyboardMouseBinding("f","Toggle crest foam");
  usage.addKeyboardMouseBinding("p","Toggle choppy waves (dirties geometry if autoDirty is active)");
  usage.addKeyboardMouseBinding("k","Decrease wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
  usage.addKeyboardMouseBinding("K","Increase wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
  usage.addKeyboardMouseBinding("-","Decrease wind speed by 0.5 (dirties geometry if autoDirty is active)");
  usage.addKeyboardMouseBinding("+","Increase wind speed by 0.5 (dirties geometry if autoDirty is active)");
}

Scene::Scene(const SceneParams &params)
{
  initCoralResources();

  scene = new osg::Group;
  loadCubeMapTextures();

  // build ocean surface
  FFToceanSurface = new osgOcean::FFTOceanSurface(64, 256, 17, params.windDirection, params.windSpeed, params.depth,
                                                  params.reflectionDamping, params.waveScale, params.isChoppy(), params.choppyFactor,
                                                  10.f, 256);
  FFToceanSurface->setEnvironmentMap(cubemap.get());
  FFToceanSurface->setFoamBottomHeight(2.2f);
  FFToceanSurface->setFoamTopHeight(3.0f);
  FFToceanSurface->enableCrestFoam(false);
  FFToceanSurface->enableEndlessOcean(true);

  // build ocean scene
  oceanScene = new OceanScene(FFToceanSurface.get(), params);
  oceanScene->setLightID(0);
  oceanScene->setCylinderSize(1900.f, 4000.f);

  // create sky dome and add to ocean scene
  // set masks so it appears in reflected scene and normal scene
  skyDome = new SkyDome(1900.f, 16, 16, cubemap.get());
  skyDome->setNodeMask(	  Mask::reflection | Mask::normal | Mask::refraction);

  // add a pat to track the camera
  /*osg::MatrixTransform* transform = new osg::MatrixTransform;
  transform->setDataVariance(osg::Object::DYNAMIC);
  transform->setMatrix(osg::Matrixf::translate(osg::Vec3f(0.f, 0.f, 0.f)));
  transform->setCullCallback(new CameraTrackCallback);

  transform->addChild(skyDome.get());*/

  oceanScene->addChild(skyDome);

  {
    // Create and add fake texture for use with nodes without any texture
    // since the OceanScene default scene shader assumes that texture unit
    // 0 is used as a base texture map.
    osg::Image * image = new osg::Image;
    image->allocateImage(1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    *(osg::Vec4ub*)image->data() = osg::Vec4ub(0xFF, 0xFF, 0xFF, 0xFF);

	osg::Texture2D* fakeTex = new osg::Texture2D(image);
	fakeTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	fakeTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
	fakeTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
	fakeTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);

	osg::StateSet* stateset = oceanScene->getOrCreateStateSet();
	stateset->setTextureAttribute(0, fakeTex, osg::StateAttribute::ON);
	stateset->setTextureMode(0, GL_TEXTURE_1D, osg::StateAttribute::OFF);
	stateset->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
	stateset->setTextureMode(0, GL_TEXTURE_3D, osg::StateAttribute::OFF);
  }

  osg::LightSource* lightSource = new osg::LightSource;
  lightSource->setNodeMask(lightSource->getNodeMask() & ~DrawMask::CAST_SHADOW & ~DrawMask::RECEIVE_SHADOW);
  lightSource->setLocalStateSetModes();

  sun = lightSource->getLight();
  sun->setLightNum(0);
  sun->setAmbient(oceanScene->weather.sunAmbient);
  sun->setDiffuse(oceanScene->weather.sunDiffuse);
  sun->setSpecular(osg::Vec4d(0.1f, 0.1f, 0.1f, 1.0f));
  sun->setPosition(osg::Vec4f(oceanScene->weather.sunPosition, 0.0));

  scene->addChild(lightSource);
  scene->addChild(oceanScene.get());

  changeMood(Weather::from(params.scene_type), true);

  oceanScene->setDefaultSceneShader(Shader::create("scene_shader"));

  root = new osg::Group;
  root->getOrCreateStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
  root->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
  root->getOrCreateStateSet()->addUniform(new osg::Uniform("sls_projector", false));
  root->addChild(scene);
 /* root->getStateSet()->addUniform(new osg::Uniform("SLStex", 3));
  root->getStateSet()->addUniform(new osg::Uniform("SLStex2", 4));
  root->getStateSet()->addUniform( new osg::Uniform("stddev", 0.0f ) );
  root->getStateSet()->addUniform( new osg::Uniform("mean", 0.0f ) );
  root->getStateSet()->addUniform( new osg::Uniform("light", (float)config.lightRate ) );*/
}

void Scene::changeMood(const Weather::Mood &mood, bool force)
{
  if(!force && this->mood == mood)
    return;

  this->mood = mood;
  // ocean
  const auto &[weather,water] = oceanScene->changeMood(mood);

  // sky
  loadCubeMapTextures(weather.cubemap);
  skyDome->setCubeMap(cubemap.get());

  // surface
  FFToceanSurface->setEnvironmentMap(cubemap.get());
  FFToceanSurface->setLightColor(weather.lightColor);

  // sun
  sun->setPosition(osg::Vec4f(weather.sunPosition, 0.f));
  sun->setDiffuse(weather.sunDiffuse);
  sun->setAmbient(weather.sunAmbient);

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
