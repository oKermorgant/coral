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
*
*
* Modified for coral package
* Copyright (C) 2021 Olivier Kermorgant
*/

#include <osgOcean/ShaderManager>
#include <osgOcean/FFTOceanTechnique>
#include <osgOcean/FFTOceanSurface>
#include <osg/Depth>
#include <osg/Switch>
#include <osg/TextureCubeMap>
#include <osgText/Text>
#include <osg/LightSource>
#include <osg/Camera>

#include <coral/OceanScene.h>
#include <coral/resource_helpers.h>

using namespace coral;

namespace
{

enum DrawMask
{
  CAST_SHADOW             = (0x1<<30),
  RECEIVE_SHADOW          = (0x1<<29),
};


// CameraTrackCallback used by osgOcean with the undersea cylinder.
// Note: only set on MatrixTransform.
class CameraTrackCallback: public osg::NodeCallback
{
public:
  explicit CameraTrackCallback(OceanScene* oceanScene)
    : _oceanScene (oceanScene)
    , _currentMatrix(0)
    , _traversalNumber(0)
  {
  }

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
      osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);

      if (nv->getTraversalNumber() >= _traversalNumber)
      {
        // Rendering new frame, reuse matrices used in the last frame.
        _currentMatrix = 0;
      }

      osg::MatrixTransform* mt = static_cast<osg::MatrixTransform*>(node);
      bool follow = true;

      if (_oceanScene->surface() && !_oceanScene->surface()->isEndlessOceanEnabled())
      {
        // Do not follow the eye when the ocean is constrained
        // to an area.
        follow = false;

        // The ocean cylinder should not be visible if the ocean
        // is constrained to an area, since the ocean will often
        // be in a pool or a river which already has walls.
        mt->getChild(0)->setNodeMask(0);
      }

      if (follow)
      {
        osg::Vec3f centre,up,eye;
        // get MAIN camera eye,centre,up
        cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye,centre,up);

        bool eyeAboveWater = _oceanScene->isEyeAboveWater(eye);
        float mult = 1.0;
        if (eyeAboveWater) mult = -1.0;

        // Translate the ocean cylinder down by the surface height
        // if the eye went from below to above the surface (so the
        // cylinder doesn't peek through the waves) and inversely
        // when the eye goes from above to below the surface (so
        // we don't see cracks between the cylinder's edge and the
        // waves).

        // Note, we set the ocean cylinder's own matrixTransform
        // to identity, and push the relevant matrix onto the
        // modelView matrix stack, because it is viewpoint
        // dependent.
        mt->setMatrix(osg::Matrix::identity());

        osg::Camera* currentCamera = cv->getCurrentRenderBin()->getStage()->getCamera();

        double z = -_oceanScene->getOceanCylinder()->getHeight() +                      // So the cylinder is underwater
            _oceanScene->getOceanSurfaceHeight() +                              // Follow the ocean surface's height
            mult * _oceanScene->surface()->getMaximumHeight();        // Offset either up or down by a bit.
        z *= 0;
        osg::RefMatrix* cylinderMatrix = createOrReuseMatrix(osg::Matrix::translate(eye.x(), eye.y(), z) * currentCamera->getViewMatrix());
        cv->pushModelViewMatrix(cylinderMatrix, osg::Transform::ABSOLUTE_RF);
      }

      traverse(node, nv);

      if (follow)
      {
        cv->popModelViewMatrix();
      }
    }
    else
    {
      traverse(node, nv);
    }
  }

  // See osg::CullStack::createOrReuseMatrix()
  osg::RefMatrix* createOrReuseMatrix(const osg::Matrix& value)
  {
    // skip of any already reused matrix.
    while (_currentMatrix < _matrices.size() &&
           _matrices[_currentMatrix]->referenceCount()>1)
    {
      ++_currentMatrix;
    }

    // if still within list, element must be singularly referenced
    // there return it to be reused.
    if (_currentMatrix < _matrices.size())
    {
      osg::RefMatrix* matrix = _matrices[_currentMatrix++].get();
      matrix->set(value);
      return matrix;
    }

    // otherwise need to create new matrix.
    osg::RefMatrix* matrix = new osg::RefMatrix(value);
    _matrices.push_back(matrix);
    ++_currentMatrix;
    return matrix;
  }

  OceanScene* _oceanScene;
  std::vector< osg::ref_ptr<osg::RefMatrix> > _matrices;
  unsigned int _currentMatrix;

  unsigned int _traversalNumber;
};

}


bool OceanScene::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*)
{
  if (ea.getHandled()) return false;

  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
  {
    const auto key{ea.getKey()};
    // Reflections
    if (key == 'r')
    {
      _oceanScene->switchParam(_oceanScene->params.reflections);
      return true;
    }
    // Refractions
    if (key == 'R')
    {
      _oceanScene->switchParam(_oceanScene->params.refractions);
      return true;
    }
    // DOF
    if (key == 'o')
    {
      _oceanScene->switchParam(_oceanScene->params.underwaterDOF);
      return true;
    }
    // Glare
    if (key == 'g')
    {
      _oceanScene->switchParam(_oceanScene->params.glare);
      return true;
    }
    // God rays
    if (key == 'G')
    {
      _oceanScene->switchParam(_oceanScene->params.godrays);
      return true;
    }
    // Silt
    if (key == 'u')
    {
      _oceanScene->switchParam(_oceanScene->params.silt);
      return true;
    }
    // Underwater scattering
    if (key == 'T')
    {
      _oceanScene->switchParam(_oceanScene->params.underwaterScattering);
      return true;
    }
    // Height map
    if (key == 'H')
    {
      _oceanScene->switchParam(_oceanScene->params.heightmap);
      _oceanScene->surface()->dirty();      // Make it reload shaders.
      return true;
    }
    // Ocean surface height
    if (key == osgGA::GUIEventAdapter::KEY_Page_Up)
    {
      _oceanScene->setOceanSurfaceHeight(_oceanScene->getOceanSurfaceHeight() + .5);
      return true;
    }
    if (key == osgGA::GUIEventAdapter::KEY_Page_Down)
    {
      _oceanScene->setOceanSurfaceHeight(_oceanScene->getOceanSurfaceHeight() - .5);
      return true;
    }
    else if (key == osgGA::GUIEventAdapter::KEY_Home)
    {
      _oceanScene->setOceanSurfaceHeight(0.);
      return true;
    }
    else if (key == osgGA::GUIEventAdapter::KEY_End)
    {
      _oceanScene->setOceanSurfaceHeight(-1000.);
      return true;
    }
    // Ocean surface shape, override FFTOceanTechnique's event handler
    auto fftSurface{_oceanScene->surface()};

    // Crest foam
    if (key == 'f' )
    {
        fftSurface->enableCrestFoam(!fftSurface->isCrestFoamEnabled());
        return true;
    }
    // isChoppy
    if( key == 'p' )
    {
        fftSurface->setIsChoppy(!fftSurface->isChoppy(), true);
        return true;
    }
    // Wind speed + 0.5
    if (key == '+')
    {
        fftSurface->setWindSpeed(fftSurface->getWindSpeed() + 0.5, true);
        return true;
    }
    // Wind speed - 0.5
    if (key == '-')
    {
        fftSurface->setWindSpeed(fftSurface->getWindSpeed() - 0.5, true);
        return true;
    }
    // Scale factor + 1e-9
    if(key == 'K' )
    {
        float waveScale = fftSurface->getWaveScaleFactor();
        fftSurface->setWaveScaleFactor(waveScale+(1e-9), true);
        return true;
    }
    // Scale factor - 1e-9
    if(key == 'k' )
    {
        float waveScale = fftSurface->getWaveScaleFactor();
        fftSurface->setWaveScaleFactor(waveScale-(1e-9), true);
        return true;
    }
  }
  return false;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void OceanScene::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
  usage.addKeyboardMouseBinding("r","Toggle reflections (above water)");
  usage.addKeyboardMouseBinding("R","Toggle refractions (underwater)");
  usage.addKeyboardMouseBinding("o","Toggle Depth of Field (DOF) (underwater)");
  usage.addKeyboardMouseBinding("g","Toggle glare (above water)");
  usage.addKeyboardMouseBinding("G","Toggle God rays (underwater)");
  usage.addKeyboardMouseBinding("u","Toggle silt (underwater)");
  usage.addKeyboardMouseBinding("T","Toggle scattering (underwater)");
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

void OceanScene::loadCubeMapTextures( const std::string& dir )
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

void OceanScene::changeMood(const Weather::Mood &mood)
{
  weather.switchTo(mood);

  base_water_color = weather.underwaterFogColor;

  loadCubeMapTextures(weather.cubemap);
  skyDome->setCubeMap( cubemap.get() );

  oceanSurface->setEnvironmentMap( cubemap.get() );
  oceanSurface->setLightColor( weather.lightColor);

  refreshAboveWaterFog();
  refreshUnderwaterFog();

  sun->setPosition( osg::Vec4f(-weather.sunDirection, 0.f));
  sun->setAmbient( weather.sunAmbient);
  sun->setDiffuse( weather.sunDiffuse);

  _isDirty = true;
}

OceanScene::OceanScene(const SceneParams &params) : params{params}
{
  initCoralResources();

  osgOcean::ShaderManager::instance().enableShaders(true);

  loadCubeMapTextures(Weather(params.scene_type).cubemap);

  // Set up surface
  oceanSurface = osg::make_ref<osgOcean::FFTOceanSurface>(
        64, 256, 17,
        //128, 128, 17, // some tests
        params.windDirection,
        params.windSpeed, params.depth,
        params.reflectionDamping,
        params.waveScale, params.isChoppy(),
        params.choppyFactor, 10.f, 256 );

  oceanSurface->setEnvironmentMap( cubemap.get() );
  oceanSurface->setFoamBottomHeight( 2.2f );
  oceanSurface->setFoamTopHeight( 3.0f );
  oceanSurface->enableCrestFoam( true );
  // Make the ocean surface track with the main camera position, giving the illusion
  // of an endless ocean surface.
  oceanSurface->enableEndlessOcean(false);

  // Set up scene, with surface
  //ocean_scene->createDefaultSceneShader();
  setLightID(0);

  // Set the size of _oceanCylinder which follows the camera underwater.
  // This cylinder prevents the clear from being visible past the far plane
  // instead it will be the fog color.
  // The size of the cylinder should be changed according the size of the ocean surface.
  setCylinderSize( 1900.f, 4000.f );

  setGlareAttenuation(0.8f);
  fitToSize(params.width, params.height);

  // create sky dome and add to ocean scene
  // set masks so it appears in reflected scene and normal scene
  skyDome = osg::make_ref<SkyDome>( 1900.f, 16, 16, cubemap.get() );
  registerVisual(skyDome);

  // add a pat to track the camera
  auto transform = osg::make_ref<osg::MatrixTransform>();
  transform->setDataVariance( osg::Object::DYNAMIC );
  transform->setMatrix( osg::Matrixf::translate( osg::Vec3f(0.f, 0.f, 0.f) ));
  //transform->setCullCallback( new CameraTrackCallback );

  //transform->addChild( skyDome.get() );

  //addChild( transform );
  addChild(skyDome);

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

  auto stateset = getOrCreateStateSet();
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
  changeMood(params.scene_type);

  addChild(lightSource);
  auto ss{getOrCreateStateSet()};
  ss->addUniform(new osg::Uniform("uOverlayMap", 1));
  ss->addUniform(new osg::Uniform("uNormalMap", 2));
  ss->addUniform(new osg::Uniform("SLStex", 3));
  ss->addUniform(new osg::Uniform("SLStex2", 4));
  ss->addUniform( new osg::Uniform("stddev", 0.0f ) );
  ss->addUniform( new osg::Uniform("mean", 0.0f ) );
  ss->addUniform( new osg::Uniform("light", 1.f ) );

  setOceanSurfaceHeight(0);


  //-----------------------------------------------------------------------
  // _oceanCylinder follows the camera underwater, so that the clear
  // color is not visible past the far plane - it will be the fog color.
  _oceanCylinder->setColor( weather.underwaterFogColor );
  _oceanCylinder->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  _oceanCylinder->getOrCreateStateSet()->setMode(GL_FOG, osg::StateAttribute::OFF);

  auto cylinderGeode = osg::make_ref<osg::Geode>();
  cylinderGeode->addDrawable( _oceanCylinder.get() );

  _oceanCylinderMT->setMatrix( osg::Matrix::translate(0, 0, -OCEAN_CYLINDER_HEIGHT) );
  _oceanCylinderMT->setDataVariance( osg::Object::DYNAMIC );
  _oceanCylinderMT->setCullCallback( osg::make_ref<CameraTrackCallback>(this));
  _oceanCylinderMT->setNodeMask( Mask::normal | Mask::refraction );
  _oceanCylinderMT->addChild( cylinderGeode );

  _oceanTransform->addChild( _oceanCylinderMT.get() );

  _oceanTransform->setNodeMask( Mask::normal | Mask::surface );
  addChild( _oceanTransform.get() );

  oceanSurface->setNodeMask( Mask::surface );
  _oceanTransform->addChild( oceanSurface.get() );

  setNumChildrenRequiringUpdateTraversal(1);

  osgOcean::ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", _lightID);
  _defaultSceneShader = createDefaultSceneShader();
}


void OceanScene::init( void )
{
  osg::notify(osg::INFO) << "OceanScene::init()" << std::endl;

  _godrayPreRender  = NULL;
  _godrayPostRender = NULL;

  if( _reflectionClipNode.valid() ){
    removeChild( _reflectionClipNode.get() );
    _reflectionClipNode = NULL;
  }

  _dofPasses.clear();
  _dofStateSet = NULL;

  _glarePasses.clear();
  _glareStateSet = NULL;

  _distortionSurface = NULL;

  if( _siltClipNode.valid() ){
    removeChild( _siltClipNode.get() );
    _siltClipNode = NULL;
  }

  if( oceanSurface.valid() )
  {
    const float LOG2E = 1.442695;

    _globalStateSet = new osg::StateSet;

    // This is now a #define, added by the call to
    // ShaderManager::setGlobalDefinition() in the constructors above.
    // Note that since _lightID can change, we will need to change the
    // global definition and reload all the shaders that depend on its
    // value when it does. This is not implemented yet.
    //_globalStateSet->addUniform( new osg::Uniform("osgOcean_LightID", _lightID ) );

    _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableDOF", params.underwaterDOF ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableGlare", params.glare ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableUnderwaterScattering", params.underwaterScattering ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_WaterHeight", float(getOceanSurfaceHeight()) ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterFogColor", weather.underwaterFogColor ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_AboveWaterFogColor", weather.fogColor ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterFogDensity", -weather.underwaterFogDensity*weather.underwaterFogDensity*LOG2E ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_AboveWaterFogDensity", -weather.aboveWaterFogDensity*weather.aboveWaterFogDensity*LOG2E ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterDiffuse", weather.underwaterDiffuse ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterAttenuation", weather.underwaterAttenuation ) );

    if(_enableDefaultShader)
    {
      _globalStateSet->setAttributeAndModes( _defaultSceneShader.get(), osg::StateAttribute::ON );
    }

    if( params.reflections )
    {
      osg::ClipPlane* reflClipPlane = new osg::ClipPlane();
      reflClipPlane->setClipPlaneNum(0);
      reflClipPlane->setClipPlane( 0.0, 0.0, 1.0, -getOceanSurfaceHeight() );
      _reflectionClipNode = new osg::ClipNode;
      _reflectionClipNode->addClipPlane( reflClipPlane );

      addChild( _reflectionClipNode.get() );
    }

    if( params.godrays )
    {
      osg::TextureRectangle* godRayTexture = createTextureRectangle( _screenDims/2, GL_RGB );

      _godrays = osg::make_ref<osgOcean::GodRays>(10,weather.sunDirection, getOceanSurfaceHeight() );

      _godrayPreRender=renderToTexturePass( godRayTexture );
      _godrayPreRender->setClearColor( osg::Vec4(0.0745098, 0.10588235, 0.1529411, 1.0) );
      _godrayPreRender->addChild( _godrays.get() );

      _godRayBlendSurface = osg::make_ref<osgOcean::GodRayBlendSurface>( osg::Vec3f(-1.f,-1.f,-1.f), osg::Vec2f(2.f,2.f), godRayTexture );

      _godRayBlendSurface->setSunDirection(weather.sunDirection);
      _godRayBlendSurface->setEccentricity(0.3f);
      _godRayBlendSurface->setIntensity(0.1f);

      _godrayPostRender=godrayFinalPass();
      _godrayPostRender->addChild( _godRayBlendSurface.get() );
    }

    if( params.underwaterDOF )
    {
      _dofPasses.clear();

      osg::Vec2s lowResDims = _screenDims/4;

      _dofStateSet = new osg::StateSet;
      _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Near",  _dofNear ) );
      _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Far",   _dofFar ) );
      _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Clamp", _dofFarClamp ) );
      _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Focus", _dofFocus ) );

      // First capture screen color buffer and a luminance buffer used for a custom depth map
      osg::TextureRectangle* fullScreenTexture   = createTextureRectangle( _screenDims, GL_RGBA );
      osg::TextureRectangle* fullScreenLuminance = createTextureRectangle( _screenDims, GL_LUMINANCE );

      osg::Camera* fullPass = multipleRenderTargetPass( fullScreenTexture,   osg::Camera::COLOR_BUFFER0,
                                                        fullScreenLuminance, osg::Camera::COLOR_BUFFER1 );

      fullPass->setCullCallback( new PrerenderCameraCullCallback(this) );
      fullPass->setStateSet(_dofStateSet.get());
      _dofPasses.push_back( fullPass );

      // Downsize image
      osg::TextureRectangle* downsizedTexture = createTextureRectangle( lowResDims, GL_RGBA );
      _dofPasses.push_back( downsamplePass( fullScreenTexture, NULL, downsizedTexture, false ) );

      // Gaussian blur 1
      osg::TextureRectangle* gaussianTexture_1 = createTextureRectangle( lowResDims, GL_RGBA );
      _dofPasses.push_back( gaussianPass(downsizedTexture, gaussianTexture_1, true ) );

      // Gaussian blur 2
      osg::TextureRectangle* gaussianTexture_2 = createTextureRectangle( lowResDims, GL_RGBA );
      _dofPasses.push_back( gaussianPass(gaussianTexture_1, gaussianTexture_2, false ) );

      // Combiner
      osg::TextureRectangle* combinedTexture = createTextureRectangle( _screenDims, GL_RGBA );
      _dofPasses.push_back( dofCombinerPass(fullScreenTexture, fullScreenLuminance, gaussianTexture_2, combinedTexture ) );

      // Post render pass
      _dofPasses.push_back( dofFinalPass( combinedTexture ) );
    }

    if( params.glare )
    {
      _glarePasses.clear();

      osg::Vec2s lowResDims = _screenDims/4;

      _glareStateSet = new osg::StateSet;
      _glareStateSet->addUniform( new osg::Uniform("osgOcean_EnableGlare", params.glare ) );

      // First capture screen
      osg::TextureRectangle* fullScreenTexture = createTextureRectangle( _screenDims, GL_RGBA );
      osg::TextureRectangle* luminanceTexture  = createTextureRectangle( _screenDims, GL_LUMINANCE );

      osg::Camera* fullPass = multipleRenderTargetPass(
            fullScreenTexture, osg::Camera::COLOR_BUFFER0,
            luminanceTexture,  osg::Camera::COLOR_BUFFER1 );

      fullPass->setCullCallback( new PrerenderCameraCullCallback(this) );
      fullPass->setStateSet(_glareStateSet.get());
      _glarePasses.push_back( fullPass );

      // Downsize image
      osg::TextureRectangle* downsizedTexture = createTextureRectangle( lowResDims, GL_RGBA );
      _glarePasses.push_back( downsamplePass( fullScreenTexture, luminanceTexture, downsizedTexture, true ) );

      // Streak filter top Right
      osg::TextureRectangle* streakBuffer1 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer1, 1, osg::Vec2f(0.5f,0.5f) ) );

      osg::TextureRectangle* streakBuffer2 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(streakBuffer1,streakBuffer2, 2, osg::Vec2f(0.5f,0.5f) ) );

      // Streak filter Bottom left
      osg::TextureRectangle* streakBuffer3 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer3, 1, osg::Vec2f(-0.5f,-0.5f) ) );

      osg::TextureRectangle* streakBuffer4 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(streakBuffer3,streakBuffer4, 2, osg::Vec2f(-0.5f,-0.5f) ) );

      // Streak filter Bottom right
      osg::TextureRectangle* streakBuffer5 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer5, 1, osg::Vec2f(0.5f,-0.5f) ) );

      osg::TextureRectangle* streakBuffer6 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(streakBuffer5,streakBuffer6, 2, osg::Vec2f(0.5f,-0.5f) ) );

      // Streak filter Top Left
      osg::TextureRectangle* streakBuffer7 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer7,1, osg::Vec2f(-0.5f,0.5f) ) );

      osg::TextureRectangle* streakBuffer8 = createTextureRectangle( lowResDims, GL_RGB );
      _glarePasses.push_back( glarePass(streakBuffer7,streakBuffer8, 2, osg::Vec2f(-0.5f,0.5f) ) );

      // Final pass - combine glare and blend into scene.
      _glarePasses.push_back( glareCombinerPass(fullScreenTexture, streakBuffer2, streakBuffer4, streakBuffer6, streakBuffer8 ) );
    }

    if( params.silt )
    {
      auto silt = osg::make_ref<osgOcean::SiltEffect>();
      // Clip silt above water level
      silt->getOrCreateStateSet()->setMode( GL_CLIP_PLANE0+1, osg::StateAttribute::ON );
      silt->setIntensity(0.07f);
      silt->setParticleSpeed(0.025f);
      silt->setNodeMask(Mask::silt);

      osg::ClipPlane* siltClipPlane = new osg::ClipPlane();
      siltClipPlane->setClipPlaneNum(1);
      siltClipPlane->setClipPlane( 0.0, 0.0, -1.0, -getOceanSurfaceHeight() );

      _siltClipNode = new osg::ClipNode;
      _siltClipNode->addClipPlane( siltClipPlane );
      _siltClipNode->addChild( silt );

      addChild(_siltClipNode);
    }
  }
  _isDirty = false;
}

OceanScene::ViewData* OceanScene::getViewDependentData( osgUtil::CullVisitor * cv )
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_viewDataMapMutex);
  return _viewDataMap[ cv ].get();
}

void OceanScene::setViewDependentData( osgUtil::CullVisitor * cv, ViewData * data )
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_viewDataMapMutex);
  _viewDataMap[ cv ] = data;
}

OceanScene::ViewData * OceanScene::initViewDependentData( osgUtil::CullVisitor *cv, OceanScene::ViewData * vd )
{
  ViewData* viewData = vd;
  if (!viewData) viewData = new ViewData;
  viewData->init( this, cv );
  return viewData;
}

void OceanScene::ViewData::dirty( bool flag )
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
  _dirty = flag;
}

void OceanScene::fitToSize(int width, int height)
{
  _screenDims.x() = width;
  _screenDims.y() = height;
  _isDirty = true;
}

#include <osgOcean/shaders/osgOcean_heightmap_vert.inl>
#include <osgOcean/shaders/osgOcean_heightmap_frag.inl>

void OceanScene::ViewData::init( OceanScene *oceanScene, osgUtil::CullVisitor * cv )
{
  _cv = cv;
  _oceanScene = oceanScene;

  _globalStateSet = new osg::StateSet;
  _surfaceStateSet = new osg::StateSet;

  _globalStateSet->addUniform( new osg::Uniform("osgOcean_EyeUnderwater", false ) );
  _globalStateSet->addUniform( new osg::Uniform("osgOcean_Eye", osg::Vec3f() ) );

  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableReflections",  _oceanScene->params.reflections ) );
  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_ReflectionMap",      _oceanScene->_reflectionUnit ) );

  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableRefractions",  _oceanScene->params.refractions ) );
  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_RefractionMap",      _oceanScene->_refractionUnit ) );
  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_RefractionDepthMap", _oceanScene->_refractionDepthUnit ) );

  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableHeightmap",    _oceanScene->params.heightmap ) );
  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_Heightmap",          _oceanScene->_heightmapUnit ) );

  _surfaceStateSet->addUniform( new osg::Uniform(osg::Uniform::FLOAT_MAT4, "osgOcean_RefractionInverseTransformation") );
  _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_ViewportDimensions", osg::Vec2(_oceanScene->_screenDims.x(), _oceanScene->_screenDims.y()) ) );

  _fog = new osg::Fog;
  _fog->setMode(osg::Fog::EXP2);
  _fog->setDensity(_oceanScene->weather.aboveWaterFogDensity);
  _fog->setColor(_oceanScene->weather.fogColor);
  _globalStateSet->setAttributeAndModes(_fog.get(), osg::StateAttribute::ON);

  if( _oceanScene->params.reflections )
  {
    // Update the reflection matrix's translation to take into account
    // the ocean surface height. The translation we need is 2*h.
    // See http://www.gamedev.net/columns/hardcore/rnerwater1/page3.asp
    _reflectionMatrix = osg::Matrixf(  1,  0,  0,  0,
                                       0,  1,  0,  0,
                                       0,  0, -1,  0,
                                       0,  0,  2 * _oceanScene->getOceanSurfaceHeight(),  1 );

    osg::ref_ptr<osg::Texture2D> reflectionTexture = _oceanScene->createTexture2D( _oceanScene->_reflectionTexSize, GL_RGBA );

    // clip everything below water line
    _reflectionCamera = _oceanScene->renderToTexturePass( reflectionTexture.get() );
    _reflectionCamera->setClearColor( osg::Vec4( 0.0, 0.0, 0.0, 0.0 ) );
    _reflectionCamera->setComputeNearFarMode( osg::Camera::DO_NOT_COMPUTE_NEAR_FAR );
    _reflectionCamera->setCullMask( Mask::reflection );
    _reflectionCamera->setCullCallback( new CameraCullCallback(_oceanScene.get()) );
    _reflectionCamera->getOrCreateStateSet()->setMode( GL_CLIP_PLANE0+0, osg::StateAttribute::ON );
    _reflectionCamera->getOrCreateStateSet()->setMode( GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

    _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_reflectionUnit, reflectionTexture.get(), osg::StateAttribute::ON );
  }

  if( _oceanScene->params.refractions )
  {
    osg::Texture2D* refractionTexture = _oceanScene->createTexture2D( _oceanScene->_refractionTexSize, GL_RGBA );
    osg::Texture2D* refractionDepthTexture = _oceanScene->createTexture2D( _oceanScene->_refractionTexSize, GL_DEPTH_COMPONENT );

    refractionTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST );
    refractionTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST );

    _refractionCamera = _oceanScene->multipleRenderTargetPass(
          refractionTexture, osg::Camera::COLOR_BUFFER,
          refractionDepthTexture, osg::Camera::DEPTH_BUFFER );

    _refractionCamera->setClearDepth( 1.0 );
    _refractionCamera->setClearColor( osg::Vec4( 0.0, 0.0, 0.0, 0.0 ) );
    _refractionCamera->setComputeNearFarMode( osg::Camera::DO_NOT_COMPUTE_NEAR_FAR );
    _refractionCamera->setCullMask( Mask::refraction );
    _refractionCamera->setCullCallback( new CameraCullCallback(_oceanScene.get()) );

    _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_refractionUnit, refractionTexture, osg::StateAttribute::ON );
    _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_refractionDepthUnit, refractionDepthTexture, osg::StateAttribute::ON );
  }

  if ( _oceanScene->params.heightmap )
  {
    osg::Texture2D* heightmapTexture = _oceanScene->createTexture2D( _oceanScene->_refractionTexSize, GL_DEPTH_COMPONENT );

    _heightmapCamera = new osg::Camera;

    _heightmapCamera->setClearMask( GL_DEPTH_BUFFER_BIT );
    _heightmapCamera->setClearDepth( 1.0 );
    _heightmapCamera->setComputeNearFarMode( osg::Camera::DO_NOT_COMPUTE_NEAR_FAR );
    _heightmapCamera->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LESS, 0.0f, 1.0f, true));

    _heightmapCamera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    _heightmapCamera->setViewport( 0,0, heightmapTexture->getTextureWidth(), heightmapTexture->getTextureHeight() );
    _heightmapCamera->setRenderOrder(osg::Camera::PRE_RENDER);
    _heightmapCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    _heightmapCamera->attach( osg::Camera::DEPTH_BUFFER, heightmapTexture );

    _heightmapCamera->setCullMask( Mask::heightmap );
    _heightmapCamera->setCullCallback( new CameraCullCallback(_oceanScene.get()) );

    static const char osgOcean_heightmap_vert_file[] = "osgOcean_heightmap.vert";
    static const char osgOcean_heightmap_frag_file[] = "osgOcean_heightmap.frag";

    osg::ref_ptr<osg::Program> program = osgOcean::ShaderManager::instance().createProgram( "heightmap",
                                                                                            osgOcean_heightmap_vert_file, osgOcean_heightmap_frag_file,
                                                                                            osgOcean_heightmap_vert,      osgOcean_heightmap_frag );

    if(program.valid())
      _heightmapCamera->getOrCreateStateSet()->setAttributeAndModes( program.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_heightmapUnit, heightmapTexture, osg::StateAttribute::ON );
  }

  dirty( false );
}

void OceanScene::ViewData::updateStateSet( bool eyeAboveWater )
{
  osg::Camera* currentCamera = _cv->getCurrentRenderBin()->getStage()->getCamera();

  _globalStateSet->getUniform("osgOcean_EyeUnderwater")->set(!eyeAboveWater);
  _globalStateSet->getUniform("osgOcean_Eye")->set( _cv->getEyePoint() );

  // Switch the fog state from underwater to above water or vice versa if necessary.
  const auto requiredFogDensity = eyeAboveWater ? _oceanScene->weather.aboveWaterFogDensity : _oceanScene->weather.underwaterFogDensity;
  const auto requiredFogColor = eyeAboveWater ? _oceanScene->weather.fogColor   : _oceanScene->weather.underwaterFogColor;
  if (requiredFogDensity != _fog->getDensity() || requiredFogColor != _fog->getColor())
  {
    _fog->setDensity(requiredFogDensity);
    _fog->setColor  (requiredFogColor);
  }

  // Update viewport dimensions
  osg::Viewport* viewport = currentCamera->getViewport();
  _surfaceStateSet->getUniform("osgOcean_ViewportDimensions")->set( osg::Vec2(viewport->width(), viewport->height()) );

  // Check if the RTT passes should be enabled for this view.
  bool enabled = (_oceanScene->_viewsWithRTTEffectsDisabled.find(currentCamera->getView()) == _oceanScene->_viewsWithRTTEffectsDisabled.end());

  bool reflectionEnabled = _oceanScene->params.reflections && eyeAboveWater && enabled &&
      ( _cv->getEyePoint().z() < _oceanScene->_eyeHeightReflectionCutoff - _oceanScene->getOceanSurfaceHeight() );
  _surfaceStateSet->getUniform("osgOcean_EnableReflections")->set(reflectionEnabled);

  if (reflectionEnabled)
  {
    // Update the reflection matrix's translation to take into account
    // the ocean surface height. The translation we need is 2*h.
    // See http://www.gamedev.net/columns/hardcore/rnerwater1/page3.asp
    _reflectionMatrix = osg::Matrixf(  1,  0,  0,  0,
                                       0,  1,  0,  0,
                                       0,  0, -1,  0,
                                       0,  0,  2 * _oceanScene->getOceanSurfaceHeight(),  1 );
  }

  // Refractions need to be calculated even when the eye is above water
  // for the shoreline foam effect and translucency.
  bool refractionEnabled = _oceanScene->params.refractions && enabled;
  _surfaceStateSet->getUniform("osgOcean_EnableRefractions")->set(refractionEnabled);

  bool heightmapEnabled = _oceanScene->params.heightmap && eyeAboveWater && enabled;
  _surfaceStateSet->getUniform("osgOcean_EnableHeightmap")->set(heightmapEnabled);
}

void OceanScene::ViewData::cull(bool surfaceVisible )
{
  // Assume _stateSet has been pushed before we get here.
  osg::Camera* currentCamera = _cv->getCurrentRenderBin()->getStage()->getCamera();

  bool reflectionEnabled;
  _surfaceStateSet->getUniform("osgOcean_EnableReflections")->get(reflectionEnabled);
  bool refractionEnabled;
  _surfaceStateSet->getUniform("osgOcean_EnableRefractions")->get(refractionEnabled);
  bool heightmapEnabled;
  _surfaceStateSet->getUniform("osgOcean_EnableHeightmap")->get(heightmapEnabled);

  _cv->pushStateSet(_oceanScene->_globalStateSet.get());

  // Render refraction if ocean surface is visible.
  if( surfaceVisible && refractionEnabled && _refractionCamera )
  {
    // update refraction camera and render refracted scene
    _refractionCamera->setViewMatrix( currentCamera->getViewMatrix() );
    _refractionCamera->setProjectionMatrix( currentCamera->getProjectionMatrix() );

    _refractionCamera->accept( *_cv );

    // Update inverse view and projection matrix
    osg::Matrix viewMatrix = _refractionCamera->getViewMatrix();
    osg::Matrix projectionMatrix = _refractionCamera->getProjectionMatrix();
    osg::Matrix inverseViewProjectionMatrix = osg::Matrix::inverse(viewMatrix * projectionMatrix);
    _surfaceStateSet->getUniform("osgOcean_RefractionInverseTransformation")->set(inverseViewProjectionMatrix);
  }

  // Render reflection if ocean surface is visible.
  if( surfaceVisible && reflectionEnabled && _reflectionCamera )
  {
    // update reflection camera and render reflected scene
    _reflectionCamera->setViewMatrix( _reflectionMatrix * currentCamera->getViewMatrix() );
    _reflectionCamera->setProjectionMatrix( currentCamera->getProjectionMatrix() );

    _reflectionCamera->accept( *_cv );
  }

  // Render height map if ocean surface is visible.
  if ( surfaceVisible && heightmapEnabled && _heightmapCamera )
  {
    // update refraction camera and render refracted scene
    _heightmapCamera->setViewMatrix( currentCamera->getViewMatrix() );
    _heightmapCamera->setProjectionMatrix( currentCamera->getProjectionMatrix() );

    _heightmapCamera->accept( *_cv );
  }

  _cv->popStateSet();
}


void OceanScene::enableRTTEffectsForView(osg::View* view, bool enable)
{
  ViewSet::iterator it = _viewsWithRTTEffectsDisabled.find(view);
  if (enable)
  {
    // Default is enabled for all views, so if we find it we
    // remove it, if not then nothing to do.
    if (it != _viewsWithRTTEffectsDisabled.end())
      _viewsWithRTTEffectsDisabled.erase(it);
  }
  else
  {
    if (it == _viewsWithRTTEffectsDisabled.end())
      _viewsWithRTTEffectsDisabled.insert(view);
  }
}

void OceanScene::traverse( osg::NodeVisitor& nv )
{

  if( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
  {
    if( _isDirty )
      init();

    update(nv);

    osg::Group::traverse(nv);
  }
  else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
  {
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

    if (cv)
    {
      osg::Camera* currentCamera = cv->getCurrentRenderBin()->getStage()->getCamera();
      if (currentCamera->getName() == "ShadowCamera" ||
          currentCamera->getName() == "AnalysisCamera" )
      {
        // Do not do reflections and everything if we're in a shadow pass.
        osg::Group::traverse(nv);
      }
      else
      {
        bool eyeAboveWater  = isEyeAboveWater(cv->getEyePoint());

        // Push the stateset for view-dependent RTT effects.
        ViewData * vd = getViewDependentData( cv );

        if ( !vd || vd->_dirty || vd->_cv != cv || vd->_oceanScene != this )
        {
          vd = initViewDependentData( cv, vd );
          setViewDependentData( cv, vd );
        }

        if (vd)
        {
          vd->updateStateSet(eyeAboveWater);
          cv->pushStateSet( vd->_globalStateSet.get() );
        }

        bool surfaceVisible = oceanSurface->isVisible(*cv, eyeAboveWater);

#if OSG_VERSION_LESS_THAN(3,3,9)
        (*_oceanSurface->getCullCallback())(_oceanSurface.get(), &nv);
#else
        (*oceanSurface->getCullCallback()).run(oceanSurface.get(), &nv);
#endif

        preRenderCull(*cv, eyeAboveWater, surfaceVisible);     // reflections/refractions

        // Above water
        if( eyeAboveWater )
        {
          if(!params.glare)
            cull(*cv, eyeAboveWater, surfaceVisible);        // normal scene render
        }
        // Below water passes
        else
        {
          if(!params.underwaterDOF)
            cull(*cv, eyeAboveWater, surfaceVisible);        // normal scene render
        }

        postRenderCull(*cv, eyeAboveWater);    // god rays/dof/glare

        if (vd)
        {
          cv->popStateSet();
        }
      }
    }
    else
      osg::Group::traverse(nv);
  }
  else
    osg::Group::traverse(nv);
}

void OceanScene::update( osg::NodeVisitor& nv )
{
  if( params.godrays && _godrays.valid() )
    _godrays->accept(nv);

  if( params.godrays && _godRayBlendSurface.valid() )
    _godRayBlendSurface->accept(nv);

  if( params.distortion && _distortionSurface.valid() )
    _distortionSurface->accept(nv);
}

void OceanScene::preRenderCull( osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible )
{
  osg::Camera* currentCamera = cv.getCurrentRenderBin()->getStage()->getCamera();

  // Render all view-dependent RTT effects.
  ViewData * vd = getViewDependentData( &cv );

  if( vd )
  {
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(vd->_mutex);
    vd->cull(surfaceVisible);
  }

  // Now render other effects.

  // Above water
  if( eyeAboveWater )
  {
    if( params.glare )
    {
      // set view and projection to match main camera
      _glarePasses.at(0)->setViewMatrix( currentCamera->getViewMatrix() );
      _glarePasses.at(0)->setProjectionMatrix( currentCamera->getProjectionMatrix() );

      for( unsigned int i=0; i<_glarePasses.size()-1; ++i )
      {
        _glarePasses.at(i)->accept(cv);
      }
    }
  }
  // Below water
  else
  {
    if( params.godrays && _godrayPreRender.valid() )
    {
      // Render the god rays to texture
      _godrayPreRender->setViewMatrix( currentCamera->getViewMatrix() );
      _godrayPreRender->setProjectionMatrix( currentCamera->getProjectionMatrix() );
      _godrayPreRender->accept( cv );
    }

    if( params.underwaterDOF )
    {
      // set view and projection to match main camera
      _dofPasses.at(0)->setViewMatrix( currentCamera->getViewMatrix() );
      _dofPasses.at(0)->setProjectionMatrix( currentCamera->getProjectionMatrix() );

      // pass the cull visitor down the chain
      for(unsigned int i = 0; i<_dofPasses.size()-1; ++i)
      {
        _dofPasses.at(i)->accept(cv);
      }
    }
  }
}

void OceanScene::postRenderCull( osgUtil::CullVisitor& cv, bool eyeAboveWater)
{
  if( eyeAboveWater )
  {
    if( params.glare )
    {
      _glarePasses.back()->accept(cv);
    }
  }
  else
  {
    // dof screen first
    if( params.underwaterDOF )
    {
      _dofPasses.back()->accept(cv);
    }
    // blend godrays ontop
    if( params.godrays )
    {
      _godrayPostRender->accept(cv);
    }
  }
}

void OceanScene::cull(osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible)
{
  unsigned int mask = cv.getTraversalMask();

  cv.pushStateSet(_globalStateSet.get());

  if ( oceanSurface.valid() && oceanSurface->getNodeMask() != 0 && surfaceVisible )
  {
    // HACK: Make sure masks are set correctly on children... This
    // assumes that the ocean surface is the only child that should have
    // the _surfaceMask bit set, and the silt node is the only child that
    // should have the _siltMask bit set. Otherwise other children will be
    // rendered twice.
    for (unsigned int i = 0; i < getNumChildren(); ++i)
    {
      osg::Node* child = getChild(i);
      if (child->getNodeMask() != 0 && child != _oceanTransform.get() && child != _siltClipNode.get())
        child->setNodeMask((child->getNodeMask() & ~Mask::surface & ~Mask::silt) | Mask::normal | Mask::reflection | Mask::refraction);
    }

    // Push the view-dependent surface stateset.
    ViewData * vd = getViewDependentData( &cv );
    if (vd)
    {
      cv.pushStateSet( vd->_surfaceStateSet.get() );
    }

    cv.setTraversalMask( mask & Mask::surface );
    osg::Group::traverse(cv);

    if (vd)
    {
      cv.popStateSet();
    }
  }

  // render rest of scene
  cv.setTraversalMask( mask & Mask::normal );
  osg::Group::traverse(cv);

  // pop globalStateSet
  cv.popStateSet();

  if( !eyeAboveWater )
  {
    if( params.silt )
    {
      cv.setTraversalMask( mask & Mask::silt );
      osg::Group::traverse(cv);
    }
  }

  // put original mask back
  cv.setTraversalMask( mask );
}

bool OceanScene::isEyeAboveWater( const osg::Vec3& eye )
{
  return (eye.z() >= getOceanSurfaceHeight());
}

osg::Camera* OceanScene::renderToTexturePass( osg::Texture* textureBuffer )
{
  osg::Camera* camera = new osg::Camera;

  camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
  camera->setClearDepth( 1.0 );
  camera->setClearColor( osg::Vec4f(0.f, 0.f, 0.f, 1.f) );
  camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
  camera->setViewport( 0,0, textureBuffer->getTextureWidth(), textureBuffer->getTextureHeight() );
  camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
  camera->setRenderOrder(osg::Camera::PRE_RENDER, 1);
  camera->attach( osg::Camera::COLOR_BUFFER, textureBuffer );

  return camera;
}

osg::Camera* OceanScene::multipleRenderTargetPass(osg::Texture* texture0, osg::Camera::BufferComponent buffer0,
                                                  osg::Texture* texture1, osg::Camera::BufferComponent buffer1 )
{
  osg::Camera* camera = new osg::Camera;

  camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
  camera->setClearDepth( 1.0 );
  camera->setClearColor( osg::Vec4f(0.f, 0.f, 0.f, 1.f) );
  camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
  camera->setViewport( 0,0, texture0->getTextureWidth(), texture0->getTextureHeight() );
  camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
  camera->setRenderOrder(osg::Camera::PRE_RENDER, 1);
  camera->attach( buffer0, texture0 );
  camera->attach( buffer1, texture1 );

  return camera;
}

osg::Camera* OceanScene::godrayFinalPass( void )
{
  osg::Camera* camera = new osg::Camera;

  camera->setClearMask(GL_DEPTH_BUFFER_BIT);
  camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
  camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
  camera->setProjectionMatrixAsOrtho( -1.f, 1.f, -1.f, 1.f, 1.0, 500.f );
  camera->setViewMatrix(osg::Matrix::identity());
  camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );

  return camera;
}

#include <osgOcean/shaders/osgOcean_downsample_vert.inl>
#include <osgOcean/shaders/osgOcean_downsample_frag.inl>
#include <osgOcean/shaders/osgOcean_downsample_glare_frag.inl>

osg::Camera* OceanScene::downsamplePass(osg::TextureRectangle* colorBuffer,
                                        osg::TextureRectangle* auxBuffer,
                                        osg::TextureRectangle* outputTexture,
                                        bool isGlareEffect )
{
  static const char osgOcean_downsample_vert_file[]       = "osgOcean_downsample.vert";
  static const char osgOcean_downsample_frag_file[]       = "osgOcean_downsample.frag";
  static const char osgOcean_downsample_glare_frag_file[] = "osgOcean_downsample_glare.frag";

  osg::Vec2s lowResDims = _screenDims/4;

  osg::StateSet* ss = new osg::StateSet;

  if (isGlareEffect)
  {
    ss->setAttributeAndModes(
          osgOcean::ShaderManager::instance().createProgram("downsample_glare",
                                                            osgOcean_downsample_vert_file, osgOcean_downsample_glare_frag_file,
                                                            osgOcean_downsample_vert,      osgOcean_downsample_glare_frag),
          osg::StateAttribute::ON );

    ss->setTextureAttributeAndModes( 1, auxBuffer,   osg::StateAttribute::ON );

    ss->addUniform( new osg::Uniform("osgOcean_GlareThreshold", _glareThreshold ) );
    ss->addUniform( new osg::Uniform("osgOcean_LuminanceTexture", 1 ) );
  }
  else
  {
    ss->setAttributeAndModes(
          osgOcean::ShaderManager::instance().createProgram("downsample",
                                                            osgOcean_downsample_vert_file,  osgOcean_downsample_frag_file,
                                                            osgOcean_downsample_vert,       osgOcean_downsample_frag ),
          osg::StateAttribute::ON );
  }

  ss->setTextureAttributeAndModes( 0, colorBuffer, osg::StateAttribute::ON );
  ss->addUniform( new osg::Uniform( "osgOcean_ColorTexture", 0 ) );

  osg::Geode* downSizedQuad = createScreenQuad( lowResDims, _screenDims );
  downSizedQuad->setStateSet(ss);

  osg::Camera* RTTCamera = renderToTexturePass( outputTexture );
  RTTCamera->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0, lowResDims.y(), 1, 10 );
  RTTCamera->setViewMatrix(osg::Matrix::identity());
  RTTCamera->addChild( downSizedQuad );

  return RTTCamera;
}

#include <osgOcean/shaders/osgOcean_gaussian_vert.inl>
#include <osgOcean/shaders/osgOcean_gaussian1_frag.inl>
#include <osgOcean/shaders/osgOcean_gaussian2_frag.inl>

osg::Camera* OceanScene::gaussianPass( osg::TextureRectangle* inputTexture, osg::TextureRectangle* outputTexture, bool isXAxis )
{
  static const char osgOcean_gaussian_vert_file[]  = "osgOcean_gaussian.vert";
  static const char osgOcean_gaussian1_frag_file[] = "osgOcean_gaussian1.frag";
  static const char osgOcean_gaussian2_frag_file[] = "osgOcean_gaussian2.frag";

  osg::Vec2s lowResDims = _screenDims/4.f;

  osg::StateSet* ss = new osg::StateSet;

  if (isXAxis)
  {
    ss->setAttributeAndModes(
          osgOcean::ShaderManager::instance().createProgram("gaussian1",
                                                            osgOcean_gaussian_vert_file, osgOcean_gaussian1_frag_file,
                                                            osgOcean_gaussian_vert,      osgOcean_gaussian1_frag),
          osg::StateAttribute::ON );
  }
  else
  {
    ss->setAttributeAndModes(
          osgOcean::ShaderManager::instance().createProgram("gaussian2",
                                                            osgOcean_gaussian_vert_file, osgOcean_gaussian2_frag_file,
                                                            osgOcean_gaussian_vert,      osgOcean_gaussian2_frag),
          osg::StateAttribute::ON );
  }

  ss->setTextureAttributeAndModes( 0, inputTexture, osg::StateAttribute::ON );
  ss->addUniform( new osg::Uniform( "osgOcean_GaussianTexture", 0 ) );

  osg::Geode* gaussianQuad = createScreenQuad( lowResDims, lowResDims );
  gaussianQuad->setStateSet(ss);

  osg::Camera* dofGaussianPass = renderToTexturePass( outputTexture );
  dofGaussianPass->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0, lowResDims.y(), 1, 10 );
  dofGaussianPass->addChild(gaussianQuad);

  return dofGaussianPass;
}


#include <osgOcean/shaders/osgOcean_dof_combiner_vert.inl>
#include <osgOcean/shaders/osgOcean_dof_combiner_frag.inl>

osg::Camera* OceanScene::dofCombinerPass(osg::TextureRectangle* fullscreenTexture,
                                         osg::TextureRectangle* fullDepthTexture,
                                         osg::TextureRectangle* blurTexture,
                                         osg::TextureRectangle* outputTexture )
{
  static const char osgOcean_dof_combiner_vert_file[] = "osgOcean_dof_combiner.vert";
  static const char osgOcean_dof_combiner_frag_file[] = "osgOcean_dof_combiner.frag";

  osg::Vec2f screenRes( (float)_screenDims.x(), (float)_screenDims.y() );
  osg::Vec2f invScreenRes( 1.f / (float)_screenDims.x(), 1.f / (float)_screenDims.y() );
  osg::Vec2f lowRes( float(_screenDims.x())*0.25f, float(_screenDims.y())*0.25f );

  auto ss = osg::make_ref<osg::StateSet>();
  ss->setTextureAttributeAndModes( 0, fullscreenTexture, osg::StateAttribute::ON );
  ss->setTextureAttributeAndModes( 1, fullDepthTexture,  osg::StateAttribute::ON );
  ss->setTextureAttributeAndModes( 2, blurTexture,       osg::StateAttribute::ON );

  ss->setAttributeAndModes(
        osgOcean::ShaderManager::instance().createProgram("dof_combiner",
                                                          osgOcean_dof_combiner_vert_file, osgOcean_dof_combiner_frag_file,
                                                          osgOcean_dof_combiner_vert,      osgOcean_dof_combiner_frag),
        osg::StateAttribute::ON );

  ss->addUniform( new osg::Uniform( "osgOcean_FullColourMap", 0 ) );
  ss->addUniform( new osg::Uniform( "osgOcean_FullDepthMap",  1 ) );
  ss->addUniform( new osg::Uniform( "osgOcean_BlurMap",       2 ) );
  ss->addUniform( new osg::Uniform( "osgOcean_ScreenRes",     screenRes ) );
  ss->addUniform( new osg::Uniform( "osgOcean_ScreenResInv",  invScreenRes ) );
  ss->addUniform( new osg::Uniform( "osgOcean_LowRes",        lowRes ) );

  osg::Geode* combinedDOFQuad = createScreenQuad( _screenDims, osg::Vec2s(1,1) );
  combinedDOFQuad->setStateSet(ss);

  osg::Camera* dofCombineCamera = renderToTexturePass( outputTexture );
  dofCombineCamera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0, _screenDims.y(), 1, 10 );
  dofCombineCamera->addChild( combinedDOFQuad );

  return dofCombineCamera;
}

osg::Camera* OceanScene::dofFinalPass( osg::TextureRectangle* combinedTexture )
{
  _distortionSurface = osg::make_ref<osgOcean::DistortionSurface>(osg::Vec3f(0,0,-1), osg::Vec2f(_screenDims.x(),_screenDims.y()), combinedTexture);

  osg::Camera* camera = new osg::Camera;
  camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
  camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
  camera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0.f, _screenDims.y(), 1.0, 500.f );
  camera->setViewMatrix(osg::Matrix::identity());
  camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );
  camera->addChild(_distortionSurface.get());

  return camera;
}

#include <osgOcean/shaders/osgOcean_streak_vert.inl>
#include <osgOcean/shaders/osgOcean_streak_frag.inl>

osg::Camera* OceanScene::glarePass(osg::TextureRectangle* streakInput,
                                   osg::TextureRectangle* steakOutput,
                                   int pass,
                                   osg::Vec2f direction )
{
  static const char osgOcean_streak_vert_file[] = "osgOcean_streak.vert";
  static const char osgOcean_streak_frag_file[] = "osgOcean_streak.frag";

  osg::Vec2s lowResDims = _screenDims / 4;

  osg::Camera* glarePass = renderToTexturePass( steakOutput );
  glarePass->setClearColor( osg::Vec4f( 0.f, 0.f, 0.f, 0.f) );
  glarePass->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0.f, lowResDims.y(), 1.0, 500.f );
  glarePass->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
  {
    auto program =
        osgOcean::ShaderManager::instance().createProgram( "streak_shader",
                                                           osgOcean_streak_vert_file, osgOcean_streak_frag_file,
                                                           osgOcean_streak_vert,      osgOcean_streak_frag );

    osg::Geode* screenQuad = createScreenQuad(lowResDims, lowResDims);
    screenQuad->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    screenQuad->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON );
    screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Buffer", 0) );
    screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Pass", float(pass)) );
    screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Direction", direction) );
    screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Attenuation", _glareAttenuation ) );
    screenQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0,streakInput,osg::StateAttribute::ON);
    glarePass->addChild( screenQuad );
  }

  return glarePass;
}

#include <osgOcean/shaders/osgOcean_glare_composite_vert.inl>
#include <osgOcean/shaders/osgOcean_glare_composite_frag.inl>

osg::Camera* OceanScene::glareCombinerPass( osg::TextureRectangle* fullscreenTexture,
                                            osg::TextureRectangle* glareTexture1,
                                            osg::TextureRectangle* glareTexture2,
                                            osg::TextureRectangle* glareTexture3,
                                            osg::TextureRectangle* glareTexture4 )
{
  osg::Camera* camera = new osg::Camera;

  camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
  camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
  camera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0.f, _screenDims.y(), 1.0, 500.f );
  camera->setViewMatrix(osg::Matrix::identity());
  camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );

  osg::Geode* quad = createScreenQuad( _screenDims, _screenDims );

  static const char osgOcean_glare_composite_vert_file[] = "osgOcean_glare_composite.vert";
  static const char osgOcean_glare_composite_frag_file[] = "osgOcean_glare_composite.frag";

  auto program =
      osgOcean::ShaderManager::instance().createProgram( "glare_composite",
                                                         osgOcean_glare_composite_vert_file, osgOcean_glare_composite_frag_file,
                                                         osgOcean_glare_composite_vert,      osgOcean_glare_composite_frag );

  osg::StateSet* ss = quad->getOrCreateStateSet();
  ss->setAttributeAndModes(program, osg::StateAttribute::ON);
  ss->setTextureAttributeAndModes(0, fullscreenTexture, osg::StateAttribute::ON );
  ss->setTextureAttributeAndModes(1, glareTexture1, osg::StateAttribute::ON );
  ss->setTextureAttributeAndModes(2, glareTexture2, osg::StateAttribute::ON );
  ss->setTextureAttributeAndModes(3, glareTexture3, osg::StateAttribute::ON );
  ss->setTextureAttributeAndModes(4, glareTexture4, osg::StateAttribute::ON );
  ss->addUniform( new osg::Uniform("osgOcean_ColorBuffer",   0 ) );
  ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer1", 1 ) );
  ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer2", 2 ) );
  ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer3", 3 ) );
  ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer4", 4 ) );

  camera->addChild( quad );

  return camera;
}

osg::Texture2D* OceanScene::createTexture2D( const osg::Vec2s& size, GLint format )
{
  osg::Texture2D* texture = new osg::Texture2D;
  texture->setTextureSize(size.x(), size.y());
  texture->setInternalFormat(format);
  texture->setFilter(osg::Texture2D::MIN_FILTER, format == GL_DEPTH_COMPONENT ? osg::Texture2D::NEAREST : osg::Texture2D::LINEAR );
  texture->setFilter(osg::Texture2D::MAG_FILTER, format == GL_DEPTH_COMPONENT ? osg::Texture2D::NEAREST : osg::Texture2D::LINEAR );
  texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
  texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
  texture->setDataVariance(osg::Object::DYNAMIC);
  return texture;
}

osg::TextureRectangle* OceanScene::createTextureRectangle( const osg::Vec2s& size, GLint format )
{
  osg::TextureRectangle* texture = new osg::TextureRectangle();
  texture->setTextureSize(size.x(), size.y());
  texture->setInternalFormat(format);
  texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
  texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
  texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
  texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
  texture->setDataVariance(osg::Object::DYNAMIC);
  return texture;
}

osg::Geode* OceanScene::createScreenQuad( const osg::Vec2s& dims, const osg::Vec2s& texSize )
{
  osg::Geode* geode = new osg::Geode;

  osg::Geometry* quad = osg::createTexturedQuadGeometry(
        osg::Vec3f(0.f,0.f,0.f),
        osg::Vec3f(dims.x(), 0.f, 0.f),
        osg::Vec3f( 0.f,dims.y(), 0.0 ),
        (float)texSize.x(),
        (float)texSize.y() );

  geode->addDrawable(quad);

  return geode;
}

#include <osgOcean/shaders/osgOcean_ocean_scene_vert.inl>
#include <osgOcean/shaders/osgOcean_ocean_scene_frag.inl>

osg::Program* OceanScene::createDefaultSceneShader(void)
{
  static const char osgOcean_ocean_scene_vert_file[] = "coral_scene.vert";//"default_scene.vert";
  static const char osgOcean_ocean_scene_frag_file[] = "coral_scene.frag";//"default_scene.frag";

  return osgOcean::ShaderManager::instance().createProgram("scene_shader",
                                                           osgOcean_ocean_scene_vert_file, osgOcean_ocean_scene_frag_file,
                                                           osgOcean_ocean_scene_vert,      osgOcean_ocean_scene_frag);
}

// -------------------------------
//     Callback implementations
// -------------------------------

OceanScene::CameraCullCallback::CameraCullCallback(OceanScene* oceanScene):
  _oceanScene(oceanScene)
{
}

void OceanScene::CameraCullCallback::operator()(osg::Node*, osg::NodeVisitor* nv)
{
  _oceanScene->osg::Group::traverse(*nv);
}

OceanScene::PrerenderCameraCullCallback::PrerenderCameraCullCallback(OceanScene* oceanScene):
  _oceanScene(oceanScene)
{
}

void OceanScene::PrerenderCameraCullCallback::operator()(osg::Node*, osg::NodeVisitor* nv)
{
  osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*> (nv);

  if(cv)
  {
    bool eyeAboveWater  = _oceanScene->isEyeAboveWater(cv->getEyePoint());
    bool surfaceVisible = _oceanScene->surface()->isVisible(*cv, eyeAboveWater);
    _oceanScene->cull(*cv, eyeAboveWater, surfaceVisible);
  }
}
/*
REGISTER_DOTOSGWRAPPER(OceanScene)
(
    new osgOcean::OceanScene,
    "OceanScene",
    "Object Node OceanScene Group",
    NULL,
    NULL
    );
*/
