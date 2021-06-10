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
#include <mutex>
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
    SceneParams params;
    std::mutex scene_mtx;

    bool useVBO;

    osg::ref_ptr<osg::Group> scene, root;

    osg::ref_ptr<osgOcean::OceanScene> ocean_scene;
    osg::ref_ptr<osgOcean::FFTOceanTechnique> ocean_surface;
    osg::ref_ptr<osg::TextureCubeMap> cubemap;
    osg::ref_ptr<SkyDome> skyDome;
    osg::ref_ptr<osg::Light> light;

public:
    Scene(const SceneParams &params);

    const SceneParams & parameters() const {return params;}
    inline std::mutex* mutex() {return &scene_mtx;}

    inline void lock() {scene_mtx.lock();}
    inline void unlock() {scene_mtx.unlock();}

    void changeScene( SceneType::SCENE_TYPE type);

    osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures( const std::string& dir );

    osg::Geode* sunDebug( const osg::Vec3f& position);

    inline osgOcean::OceanTechnique* oceanSurface()
    {
        return ocean_surface.get();
    }

    inline osg::Group* fullScene(){
        return root.get();
    }

    inline osgOcean::OceanScene* oceanScene()
    {
        return ocean_scene.get();
    }

    inline void setupMeshNode(osg::Node *mesh)
    {
      mesh->setNodeMask(ocean_scene->getNormalSceneMask() |
                        ocean_scene->getReflectedSceneMask() |
                        ocean_scene->getRefractedSceneMask());
    }

    osg::Light* getLight() { return light.get(); }
};

}
