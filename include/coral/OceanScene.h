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

#ifndef CORAL_OCEAN_SCENE_H
#define CORAL_OCEAN_SCENE_H

#include <osgOcean/Export>
#include <osgOcean/OceanTechnique>
#include <osgOcean/GodRayBlendSurface>
#include <osgOcean/DistortionSurface>
#include <osgOcean/GodRays>
#include <osgOcean/SiltEffect>
#include <osgOcean/Cylinder>

#include <osg/Group>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osg/Uniform>
#include <osgUtil/CullVisitor>
#include <osg/MatrixTransform>
#include <osg/ClipNode>
#include <osg/ClipPlane>
#include <osgGA/GUIEventHandler>

#include <coral/masks.h>
#include <coral/scene_params.h>
#include <coral/weather.h>
#include <coral/water.h>

#include <map>

namespace coral
{
/**
    * Controls the render passes required for the ocean scene effects.
    * Uses a series of traversal masks to control which passes a child is subjected to.
    * @note Requires an \c OceanTechnique to be added.
    */
class OSGOCEAN_EXPORT OceanScene : public osg::Group
{
public:
  SceneParams params;
  Weather weather;
  Water water;

  typedef std::set< osg::observer_ptr<osg::View> > ViewSet;

private:
  osg::ref_ptr<osgOcean::OceanTechnique> _oceanSurface;

  bool _isDirty{true};

  osg::Vec2s _reflectionTexSize{512,512};
  osg::Vec2s _refractionTexSize{512,512};
  osg::Vec2s _screenDims{1920,1080};

  int _reflectionUnit{1};
  int _refractionUnit{2};
  int _refractionDepthUnit{3};
  int _heightmapUnit{7};

  osg::ref_ptr<osg::Camera> _godrayPreRender;
  osg::ref_ptr<osg::Camera> _godrayPostRender;

  std::vector< osg::ref_ptr<osg::Camera> > _dofPasses;
  std::vector< osg::ref_ptr<osg::Camera> > _glarePasses;

  osg::ref_ptr<osg::StateSet> _dofStateSet;
  osg::ref_ptr<osg::StateSet> _glareStateSet;
  osg::ref_ptr<osg::StateSet> _globalStateSet;

  osg::ref_ptr<osg::Program> _defaultSceneShader;

  osg::ref_ptr<osgOcean::GodRayBlendSurface> _godRayBlendSurface;
  osg::ref_ptr<osgOcean::DistortionSurface> _distortionSurface;
  osg::ref_ptr<osgOcean::GodRays> _godrays;
  osg::ref_ptr<osg::ClipNode> _siltClipNode;
  osg::ref_ptr<osg::ClipNode> _reflectionClipNode;

  unsigned int _lightID{ 0 };

  float _dofNear{ 0.f };
  float _dofFar{ 160.f };
  float _dofFarClamp{ 1.f };
  float _dofFocus{ 30.f };
  float _glareThreshold{ 0.9f };
  float _glareAttenuation{ 0.8f };

  float _eyeHeightReflectionCutoff{ FLT_MAX};
  float _eyeHeightRefractionCutoff{ -FLT_MAX};

  float _surfaceHeight{0};
  osg::ref_ptr<osg::MatrixTransform>  _oceanTransform{ new osg::MatrixTransform };
  osg::ref_ptr<osg::MatrixTransform>  _oceanCylinderMT{ new osg::MatrixTransform };
  osg::ref_ptr<osgOcean::Cylinder>    _oceanCylinder{ new osgOcean::Cylinder(1900.f, 4000.f, 16, false, true) };

  ViewSet                             _viewsWithRTTEffectsDisabled;

  struct ViewData : public osg::Referenced
  {
	/// Method called upon ViewData instance to initialize internal variables
	virtual void init( OceanScene* oceanScene, osgUtil::CullVisitor* cv );

	virtual void updateStateSet( bool eyeAboveWater );

	/// Method called by OceanScene to allow ViewData
	/// do the hard work computing reflections/refractions for its associated view
	virtual void cull( bool eyeAboveWater, bool surfaceVisible );

	/// Dirty is called by parent OceanScene to force
	/// update of resources after some of them were modified in parent scene
	virtual void dirty( bool flag );

	/// View's CullVisitor associated with this ViewData instance
	osg::observer_ptr< osgUtil::CullVisitor > _cv;

	/// Parent OceanScene
	osg::observer_ptr< OceanScene > _oceanScene;

	/// Mutex used to guard _dirty flag from override in case when parent technique calls
	/// dirty() simultaneously with ViewData while it is updating resources inside init method.
	OpenThreads::Mutex _mutex;

	/// Dirty flag tells this instance to update its resources
	bool _dirty{true};

	osg::Matrixf _reflectionMatrix;
	osg::ref_ptr<osg::Camera> _reflectionCamera;
	osg::ref_ptr<osg::Camera> _refractionCamera;
	osg::ref_ptr<osg::Camera> _heightmapCamera;

	osg::ref_ptr<osg::Fog> _fog;
	bool _eyeAboveWaterPreviousFrame{true};

	osg::ref_ptr<osg::StateSet> _globalStateSet;
	osg::ref_ptr<osg::StateSet> _surfaceStateSet;

    friend class OceanScene;
  };

  /// Map of view dependent data per view cull visitor (CVs are used as indices)
  /// ViewDependentShadowTechnique uses this map to find VieData for each cull vitior
  typedef std::map< osg::observer_ptr< osgUtil::CullVisitor >,
                   osg::ref_ptr< ViewData > > ViewDataMap;

  ViewDataMap                                       _viewDataMap;


  /// Mutex used to serialize accesses to ViewDataMap
  OpenThreads::Mutex                             _viewDataMapMutex;

  /// Return view dependent data for the cull visitor
  OceanScene::ViewData * getViewDependentData( osgUtil::CullVisitor * cv );

  /// Define view dependent data for the cull visitor
  void setViewDependentData( osgUtil::CullVisitor * cv, OceanScene::ViewData * data );

  ViewData * initViewDependentData( osgUtil::CullVisitor *cv, OceanScene::ViewData * vd );

  // NOTE: Remember to add new variables to the copy constructor.
public:
  OceanScene(osgOcean::OceanTechnique* technique, const SceneParams &params );
  //OceanScene( const OceanScene& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

  std::pair<Weather,Water> changeMood(const Weather::Mood &mood);

  virtual const char* libraryName() const { return "osgOcean"; }
  virtual const char* className() const { return "OceanScene"; }
  virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const OceanScene*>(obj) != 0; }

  /// Sets up statesets and render passes based on current settings
  /// Called in the update traversal if dirty flag is set.
  void init( void );

  /// Check if the eye is above water or not.
  bool isEyeAboveWater( const osg::Vec3& eye );

  /// switch a bool param
  inline void switchParam( bool& param )
  {
    param = !param;
    _isDirty = true;
  }

  /// force dirty the scene
  inline void dirty()
  {
    _isDirty = true;
  }

  /// Set ocean surface height in world space (default is 0.0)
  void setOceanSurfaceHeight(float height){
    _surfaceHeight = height;
    _oceanTransform->setMatrix(osg::Matrix::translate(0,0,_surfaceHeight - _oceanSurface->getSurfaceHeight()));
    _isDirty = true;
  }

  /// Get ocean surface average height in world space.
  inline auto getOceanSurfaceHeight() const{
    return _surfaceHeight + _oceanSurface->getSurfaceHeight();
  }

  /// Get height of given (x,y) point in world space. Optionally returns the normal.
  float getOceanSurfaceHeightAt(float x, float y, osg::Vec3* normal = 0)
  {
    return _surfaceHeight +
           _oceanSurface->getSurfaceHeightAt(x, y, normal);
  }

  /// Set the ocean surface world-space position. Note that the (x,y)
  /// components of the translation are of no consequence if the ocean
  /// surface is infinite, since the surface will follow the eye.
  void setOceanSurfaceTransform(const osg::Matrix& transform)
  {
    _surfaceHeight = transform.getTrans().z();
    _oceanTransform->setMatrix(osg::Matrix::translate(transform.getTrans().x(),
                                                      transform.getTrans().y(),
                                                      transform.getTrans().z() - _oceanSurface->getSurfaceHeight()));
    _isDirty = true;
  }

  /// Returns the world-space position of the ocean surface. Note that
  /// the (x,y) components of the translation are of no consequence if
  /// the ocean surface is infinite, since the surface will follow the
  /// eye.
  osg::Matrix getOceanSurfaceTransform() const
  {
    return osg::Matrix::translate(_oceanTransform->getMatrix().getTrans().x(),
                                  _oceanTransform->getMatrix().getTrans().y(),
                                  _surfaceHeight + _oceanSurface->getSurfaceHeight());
  }

  /// Set whether the ocean surface is visible or not.
  void setOceanVisible(bool visible){
    _oceanTransform->setNodeMask( visible ? Mask::normal | Mask::surface : 0 );
  }

  /// Check whether the ocean surface is visible or not.
  bool isOceanVisible() const { return _oceanTransform->getNodeMask() != 0; }

  /// Get the ocean cylinder.
  osgOcean::Cylinder* getOceanCylinder() const{
    return _oceanCylinder.get();
  }

  /// Get the ocean cylinder's transform node.
  osg::MatrixTransform* getOceanCylinderTransform() const{
    return _oceanCylinderMT.get();
  }

  /// Set the size of _oceanCylinder which follows the camera underwater, so that the clear
  /// color is not visible past the far plane - it will be the fog color.
  /// Height is a positive number which represents depth.
  /// Default values are Radius: 1900 and Height: 4000
  inline void setCylinderSize( float radius, float height ){
    _oceanCylinder->build( radius, height, 16, false, true );
    _oceanCylinderMT->setMatrix( osg::Matrixf::translate(osg::Vec3f(0.f,0.f,-height-0.2f)) );
  }

  /// Get the radius of the ocean cylinder.
  inline  float getCylinderRadius( void ) const{
    return _oceanCylinder->getRadius();
  }

  /// Get the height of the ocean cylinder.
  inline  float getCylinderHeight( void ) const{
    return _oceanCylinder->getHeight();
  }

  /// Enable/disable RTT effects (reflection, refraction, height map)
  /// for the given view.
  void enableRTTEffectsForView(osg::View* view, bool enable);


  /// Get the list of views where RTT effects (reflection, refraction,
  /// height map) are disabled (enabled for all other views).
  ViewSet getViewsWithRTTEffectsDisabled() const
  {
    return _viewsWithRTTEffectsDisabled;
  }

  /// Enable reflections (one RTT pass when the eye is above the ocean
  /// surface).
  inline void enableReflections( bool enable ){
    params.reflections = enable;
    _isDirty = true;
  }

  /// Check whether reflections are enabled.
  inline bool areReflectionsEnabled() const{
    return params.reflections;
  }

  /// If the eye is higher than this value above the ocean surface,
  /// reflections will not be rendered. Set to a very large value to
  /// disable this feature. Default is FLT_MAX.
  inline void setEyeHeightReflectionCutoff( float cutoff ){
    _eyeHeightReflectionCutoff = cutoff;
  }

  /// Get the eye height reflection cutoff.
  inline float getEyeHeightReflectionCutoff() const{
    return _eyeHeightReflectionCutoff;
  }

  /// Set reflection texture size (must be 2^n)
  inline void setReflectionTextureSize( const osg::Vec2s& size ){
    if( size.x() != size.y() )
      return;
    _reflectionTexSize = size;
    _isDirty = true;
  }

  /// Enable refractions (one RTT pass whether the eye is above or
  /// below the ocean surface).
  inline void enableRefractions( bool enable ){
    params.refractions = enable;
    _isDirty = true;
  }

  /// Check whether refractions are enabled.
  inline bool areRefractionsEnabled() const{
    return params.refractions;
  }

  /// If the eye is lower than this value below the ocean surface,
  /// refractions will not be rendered. Set to a very large negative
  /// value to disable this feature. Default is -FLT_MAX.
  inline void setEyeHeightRefractionCutoff( float cutoff ){
    _eyeHeightRefractionCutoff = cutoff;
  }

  /// Get the eye height refraction cutoff.
  inline float getEyeHeightRefractionCutoff() const{
    return _eyeHeightRefractionCutoff;
  }

  /// Set refraction texture size (must be 2^n)
  inline void setRefractionTextureSize( const osg::Vec2s& size){
    if( size.x() != size.y() )
      return;
    _refractionTexSize = size;
    _isDirty = true;
  }

  /// Enable the height map pass (one RTT pass when the eye is above
  /// the ocean surface - uses the same texture size as refractions).
  inline void enableHeightmap( bool enable ){
    params.heightmap = enable;
    _isDirty = true;
  }

  /// Check whether the height map pass is enabled.
  inline bool isHeightmapEnabled() const {
    return params.heightmap;
  }

  /// Enable underwater God Rays.
  inline void enableGodRays( bool enable ){
    params.godrays = enable;
    _isDirty = true;
  }

  /// Check whether God Rays are enabled.
  inline bool areGodRaysEnabled() const{
    return params.godrays;
  }

  /// Enable underwater silt.
  inline void enableSilt( bool enable ){
    params.silt = enable;
    _isDirty = true;
  }

  /// Check whether silt is enabled.
  inline bool isSiltEnabled() const{
    return params.silt;
  }

  /// Sets the current screen size, needed to initialise the God Ray
  /// and DOF frame buffers. Default is 1024x768.
  inline void setScreenDims( short width, short height )
  {
    _screenDims.x() = width;
    _screenDims.y() = height;
    _isDirty = true;
  }

  /// Enable underwater depth of field.
  inline void enableUnderwaterDOF( bool enable ){
    params.underwaterDOF = enable;
    _isDirty = true;
  }

  /// Check if underwater depth of field is enabled.
  inline bool isUnderwaterDOFEnabled() const{
    return params.underwaterDOF;
  }

  /// Set near DOF blur distance.
  inline void setDOFNear( float dofNear ) {
    _dofNear = dofNear;

	if( _dofStateSet.valid() )
	  _dofStateSet->getUniform("osgOcean_DOF_Near")->set(_dofNear);
  }

  /// Get near DOF blur distance.
  inline float getDOFNear() const{
    return _dofNear;
  }

  /// Set far DOF blur distance
  inline void setDOFFar( float dofFar ) {
    _dofFar = dofFar;

	if( _dofStateSet.valid() )
	  _dofStateSet->getUniform("osgOcean_DOF_Far")->set(_dofFar);
  }

  /// Get far DOF blur distance.
  inline float getDOFFar() const{
    return _dofFar;
  }

  /// Set far clamp value.
  inline void setDOFFarClamp( float farClamp ){
    _dofFarClamp = farClamp;

	if( _dofStateSet.valid() )
	  _dofStateSet->getUniform("osgOcean_DOF_Clamp")->set(_dofFarClamp);
  }

  /// Get far clamp value.
  inline float getDOFFarClamp() const{
    return _dofFarClamp;
  }

  /// Set DOF focal distance.
  inline void setDOFFocalDistance( float focus ){
    _dofFocus = focus;

	if( _dofStateSet.valid() )
	  _dofStateSet->getUniform("osgOcean_DOF_Focus")->set(_dofFocus);
  }

  /// Get DOF focal distance.
  inline float getDOFFocalDistance() const{
    return _dofFocus;
  }

  /// Enable cross hatch glare.
  inline void enableGlare( bool flag )
  {
    params.glare = flag;

    _isDirty = true;
  }

  /// Check if glare is enabled.
  inline bool isGlareEnabled() const{
    return params.glare;
  }

  /// Set the luminance threshold for glare.
  /// Luminance value at which glare appears.
  /// Typical range: 0.75 < threshold < 1.0
  inline void setGlareThreshold( float threshold )
  {
    _glareThreshold = threshold;
    _isDirty = true;
  }

  /// Get the luminance threshold for glare.
  inline float getGlareThreshold() const{
    return _glareThreshold;
  }

  /// Set the glare attenuation.
  /// Controls the rate at which the glare drops off.
  /// Typical range: 0.75 < attenuation < 0.95
  inline void setGlareAttenuation( float attenuation )
  {
    _glareAttenuation = attenuation;
    _isDirty = true;
  }

  /// Get the current ocean technique.
  inline auto getOceanTechnique( void ) {
    return _oceanSurface.get();
  }

    /// Set the ID of the light source that should be used to light the ocean.
  inline void setLightID( unsigned int id ){
    _lightID = id;
    _isDirty = true;
  }

  /// Sets the fogging params for the above water scene.
  /// EXP fog
  inline void refreshAboveWaterFog()
  {
	const float LOG2E = 1.442695;
	if( _globalStateSet.valid() ){
	  _globalStateSet->getUniform("osgOcean_AboveWaterFogDensity")->set(-weather.fogDensity*weather.fogDensity*LOG2E);
	  _globalStateSet->getUniform("osgOcean_AboveWaterFogColor")->set(weather.fogColor);
	}
    _isDirty = true;
  }

  /// Sets the fogging params for the underwater scene.
  /// EXP2 fog
  inline void refreshUnderwaterFog()
  {
    _oceanCylinder->setColor(water.fogColor);

	const float LOG2E = 1.442695;
	if( _globalStateSet.valid() ){
	  _globalStateSet->getUniform("osgOcean_UnderwaterFogDensity")->set(-water.fogDensity*water.fogDensity*LOG2E);
	  _globalStateSet->getUniform("osgOcean_UnderwaterFogColor")->set(water.fogColor);
	}
    _isDirty = true;
  }

  /// Override the default scene shader for custom shaders.
  /// If custom shaders are required for individual nodes add them
  /// before adding to the OceanScene.
  void setDefaultSceneShader( osg::Program* program )
  {
    _defaultSceneShader = program;
    _isDirty = true;
  }

  /// Add a new path from which to search for library textures.
  /// Updates osgDB::Registry's data file path
  inline void addAlternativeTexturePath( const std::string& path )
  {
    osgDB::Registry::instance()->getDataFilePathList().push_back(path);
  }

  /// Add a new path from which to search for library shaders.
  /// Updates osgDB::Registry's data file path
  inline void addAlternativeShaderPath( const std::string& path )
  {
    osgDB::Registry::instance()->getDataFilePathList().push_back(path);
  }

private:
  osg::Texture2D* createTexture2D( const osg::Vec2s& size, GLint format );
  osg::TextureRectangle* createTextureRectangle( const osg::Vec2s& size, GLint format );

  /// Override OSG traversal function in order to do custom rendering.
  void traverse(osg::NodeVisitor& nv);

  /// Main cull traversal.
  /// Renders main scene, surface, silt.
  void cull( osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible );

  /// Render to texture passes for reflection/refractions/height map/god rays.
  void preRenderCull( osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible );

  /// Post render passes for DOF/god rays.
  void postRenderCull( osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible );

  /// Update god ray geometry and screen quad.
  void update(osg::NodeVisitor& nv);

  /// Post render pass for god rays. */
  osg::Camera* godrayFinalPass( void );

  /// Downsample (1/4 original size) pass for depth of field and glare effect.
  /// colorBuffer refers to the main frame buffer color image
  /// auxBuffer refers to the luminance buffer (glare) or depth buffer (dof).
  osg::Camera* downsamplePass(
      osg::TextureRectangle* colorBuffer,
      osg::TextureRectangle* auxBuffer,
      osg::TextureRectangle* outputTexture,
      bool isGlareEffect );

  /// Gaussian blur passes for the depth of field effect.
  /// One pass is required for each axis.
  osg::Camera* gaussianPass( osg::TextureRectangle* inputTexture, osg::TextureRectangle* outputTexture, bool isXAxis );

  /// Combine original FBO with downsampled blur image.
  osg::Camera* dofCombinerPass(
      osg::TextureRectangle* fullscreenTexture,
      osg::TextureRectangle* fullDepthTexture,
      osg::TextureRectangle* blurTexture,
      osg::TextureRectangle* outputTexture );

  /// Post render pass displays combined DOF buffer
  osg::Camera* dofFinalPass( osg::TextureRectangle* combinedTexture );

  /// Post render pass blends glare texture into main
  osg::Camera* glareCombinerPass(
      osg::TextureRectangle* fullscreenTexture,
      osg::TextureRectangle* glareTexture1,
      osg::TextureRectangle* glareTexture2,
      osg::TextureRectangle* glareTexture3,
      osg::TextureRectangle* glareTexture4  );

  /// Pre render pass adds streak filter to image
  osg::Camera* glarePass(osg::TextureRectangle* streakInput,
                         osg::TextureRectangle* steakOutput,
                         int pass,
                         osg::Vec2f direction );

  /// Sets up a camera for a render to FBO pass.
  osg::Camera* renderToTexturePass( osg::Texture* textureBuffer );

  /// Sets up a camera for a render MRT FBO pass.
  osg::Camera* multipleRenderTargetPass(
      osg::Texture* texture0, osg::Camera::BufferComponent buffer0,
      osg::Texture* texture1, osg::Camera::BufferComponent buffer1 );

  /// Create geometry for a screen aligned quad.
  osg::Geode* createScreenQuad( const osg::Vec2s& dims, const osg::Vec2s& texSize );

protected:

  inline ~OceanScene() {}

  class CameraCullCallback : public osg::NodeCallback
  {
  public:
    CameraCullCallback(OceanScene* oceanScene);
    virtual void operator()(osg::Node*, osg::NodeVisitor* nv);
  protected:
    OceanScene* _oceanScene;
  };

  class PrerenderCameraCullCallback : public osg::NodeCallback
  {
  public:
    PrerenderCameraCullCallback(OceanScene* oceanScene);
    virtual void operator()(osg::Node*, osg::NodeVisitor* nv);
  protected:
    OceanScene* _oceanScene;
  };
};
}

#endif
