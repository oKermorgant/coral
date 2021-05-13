#include <coral/scene_interface.h>
#include <osgOcean/ShaderManager>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ViewDependentShadowMap>

using namespace coral;

SceneInterface::SceneInterface(SceneParams params)
 : params(params), scene(params, "terrain"), root(new osg::Group)
{
  this->params = params;

  root = new osg::Group;

   // static const char model_vertex[] = "default_scene.vert";
  //  static const char model_fragment[] = "default_scene.frag";
  //  osg::Program* program = osgOcean::ShaderManager::instance().createProgram("object_shader", model_vertex,model_fragment, "", "");
  //  scene.oceanScene()->setDefaultSceneShader(program);
    scene.oceanScene()->enableSilt(true);

    root->getOrCreateStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
    root->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
    root->getStateSet()->addUniform(new osg::Uniform("SLStex", 3));
    root->getStateSet()->addUniform(new osg::Uniform("SLStex2", 4));
    root->getStateSet()->addUniform( new osg::Uniform("stddev", 0.0f ) );
    root->getStateSet()->addUniform( new osg::Uniform("mean", 0.0f ) );
    root->getStateSet()->addUniform( new osg::Uniform("light", 1.f ) );
    scene.oceanScene()->enableGodRays(params.godrays);

  scene.oceanScene()->setOceanSurfaceHeight(0);

  root->addChild( scene.fullScene());
}
