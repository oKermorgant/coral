#pragma once
#include <osg/Switch>
#include <osg/TextureCubeMap>

#include <osgText/Text>

#include <coral/OceanScene.h>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/FFTOceanSurfaceVBO>

#include "SkyDome.h"
#include <coral/scene_params.h>
#include <coral/scene_type.h>
#include <osg/Version>
namespace coral
{

enum DrawMask
{
    CAST_SHADOW             = (0x1<<30),
    RECEIVE_SHADOW          = (0x1<<29),
};


class Scene : public osg::Referenced
{
private:

    SceneType scene_type;

    bool useVBO;

    osg::ref_ptr<osg::Group> scene;

    osg::ref_ptr<osgOcean::OceanScene> ocean_scene;
    osg::ref_ptr<osgOcean::FFTOceanTechnique> ocean_surface;
    osg::ref_ptr<osg::TextureCubeMap> cubemap;
    osg::ref_ptr<SkyDome> skyDome;
    osg::ref_ptr<osg::Light> light;

    osg::ref_ptr<osg::Switch> island_switch;

public:
    Scene(const SceneParams &params, const std::string& terrain_shader_basename );

    void changeScene( SceneType::SCENE_TYPE type);

    // Load the islands model
    // Here we attach a custom shader to the model.
    // This shader overrides the default shader applied by OceanScene but uses uniforms applied by OceanScene.
    // The custom shader is needed to add multi-texturing and bump mapping to the terrain.
    osg::Node* loadIslands(const std::string& terrain_shader_basename);

    osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures( const std::string& dir );

    osg::Geode* sunDebug( const osg::Vec3f& position );


    inline osgOcean::OceanScene::EventHandler* getOceanSceneEventHandler()
    {
        return ocean_scene->getEventHandler();
    }

    inline osgOcean::OceanTechnique* oceanSurface()
    {
        return ocean_surface.get();
    }

    inline osg::Group* fullScene(void){
        return scene.get();
    }

    inline osgOcean::OceanScene* oceanScene()
    {
        return ocean_scene.get();
    }

    osg::Light* getLight() { return light.get(); }
};

}
