#ifndef CORAL_RESOURCE_HELPERS_H
#define CORAL_RESOURCE_HELPERS_H

#include <string>
#include <filesystem>

namespace coral
{

void initCoralResources();
inline void addResourcePath(const std::string &path);
std::filesystem::path resolvePath(const std::string &file);

}

#endif // CORAL_RESOURCE_HELPERS_H
