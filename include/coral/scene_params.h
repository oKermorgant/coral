#ifndef CORAL_SCENE_PARAMS_H
#define CORAL_SCENE_PARAMS_H

#include <osg/Vec2f>
#include <osg/Vec3>

namespace coral
{

struct SceneParams
{

  // written from node params
  osg::Vec2f windDirection;
  float windSpeed;

  float depth, reflectionDamping;

  float waveScale, choppyFactor, crestFoamHeight;

  bool useVBO, isChoppy, godrays;
  osg::Vec3 initialCameraPosition;

  // hard-coded to default values in osgocean demo


};

}

#endif // CORAL_SCENE_PARAMS_H
