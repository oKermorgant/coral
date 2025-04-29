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
  float reflectionDamping         {0.35f};

  float waveScale                 {1e-8f};
  float choppyFactor              {2.5f};
  float crestFoamHeight           {2.2f};

  bool refractions                {true};
  bool reflections                {true};
  bool godrays                    {false};  // godrays do not work on Intel GPUs
  bool glare                      {true};
  bool underwaterDOF              {true};  // creates a dark effect on close objects?
  bool distortion                 {true};
  bool silt                       {true};
  bool underwaterScattering       {true};
  bool heightmap                  {false};
  osg::Vec3 initialCameraPosition {-10.,0.,5.};

  // display
  int width         {1024};
  int height        {768};
  bool surface_keys{true};
  bool stats_keys{true};
  bool stateset_keys{false};


  // light and weather conditions
  std::string scene_type{"clear"};
  double jerlov{0.2};
  double elev{60.};
  double azim{20.};
  double fogDensity{0.002f};

  inline bool isChoppy() const {return std::abs(choppyFactor) > 1e-3;}



};

}

#endif // CORAL_SCENE_PARAMS_H
