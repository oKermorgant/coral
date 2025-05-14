#ifndef CORAL_MASKS_H
#define CORAL_MASKS_H

#include <osg/Node>
#include <osgOcean/ShaderManager>

namespace coral
{

namespace Mask
{
constexpr static unsigned int reflection  {0x1};
constexpr static unsigned int refraction  {0x2};
constexpr static unsigned int normal      {0x4};
constexpr static unsigned int surface     {0x8};
constexpr static unsigned int silt        {0x10};
constexpr static unsigned int heightmap   {0x20};
constexpr static unsigned int marker      {0x40};
}

namespace DrawMask
{
constexpr uint CAST_SHADOW = (0x1 << 30);
constexpr uint RECEIVE_SHADOW = (0x1 << 29);
}

namespace Shader
{
#define CORAL_SHADERS

#ifdef CORAL_SHADERS
constexpr static auto ocean_scene_vert_file = "coral_scene.vert";
constexpr static auto ocean_scene_frag_file = "coral_scene.frag";
#else
constexpr static auto ocean_scene_vert_file = "default_scene.vert";
constexpr static auto ocean_scene_frag_file = "default_scene.frag";
#endif

inline auto create(const std::string &name, const std::string &vert_file = ocean_scene_vert_file, const std::string &frag_file = ocean_scene_frag_file)
{
  return osgOcean::ShaderManager::instance().createProgram(name, vert_file, frag_file, "", "");
}
}
}

#endif // CORAL_MASKS_H
