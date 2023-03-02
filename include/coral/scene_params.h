#ifndef CORAL_SCENE_PARAMS_H
#define CORAL_SCENE_PARAMS_H

#include <osg/Vec2f>
#include <osg/Vec3>
#include <vector>
#include <string>

namespace coral
{

struct SceneParams
{
  static inline std::vector<double> asVector(const osg::Vec2f &vec)
  {
    return {vec.x(), vec.y()};
  }
  static inline std::vector<double> asVector(const osg::Vec3 &vec)
  {
    return {vec.x(), vec.y(), vec.z()};
  }    

  // will be updated from node param if any
  osg::Vec2f windDirection        {1,1};
  float windSpeed                 {3.3};

  float depth                     {1000};
  float depth_attn                {300};
  float reflectionDamping         {0.35f};

  float waveScale                 {1e-8f};
  float choppyFactor              {2.5f};
  float crestFoamHeight           {2.2f};

  bool godrays                    {false};  // godrays do not work on Intel GPUs
  bool glare                      {true};
  bool underwaterDof              {false};  // do not work on Intel GPUs
  osg::Vec3 initialCameraPosition {-10.,0.,10.};

  int width         {1024};
  int height        {768};

  inline bool isChoppy() const {return std::abs(choppyFactor) > 1e-3;}

  std::string scene_type;

};

}

#endif // CORAL_SCENE_PARAMS_H
