#ifndef CORAL_URDF_PARSER_H
#define CORAL_URDF_PARSER_H

#include <coral/link.h>
#include <coral/camera.h>

namespace coral
{

namespace urdf_parser
{

std::tuple<std::vector<Link>, std::vector<CameraInfo> > parse(const std::string &description, bool with_thrusters, Link &world_link);

}

}

#endif // CORAL_URDF_PARSER_H
