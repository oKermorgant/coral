#include <coral/camera.h>
#include <coral/coral_node.h>
#include <tinyxml.h>

using namespace std::chrono_literals;

namespace coral
{

std::vector<CameraInfo> CameraInfo::extractFrom(const std::string &description)
{
  std::vector<CameraInfo> cameras;

  // tediously parse the description XML to extract gazebo sensors and plugins

  TiXmlDocument doc;
  doc.Parse(description.c_str());
  auto root = doc.RootElement();

  for(auto gazebo_elem = root->FirstChildElement("gazebo");
      gazebo_elem != nullptr;
      gazebo_elem = gazebo_elem->NextSiblingElement("gazebo"))
  {
    for(auto sensor_elem = gazebo_elem->FirstChildElement("sensor");
        sensor_elem != nullptr;
        sensor_elem =sensor_elem->NextSiblingElement("sensor"))
    {
      std::cout << sensor_elem->Attribute("type") << std::endl;
    }
  }

  return cameras;
}


Camera::Camera(CoralNode* node, const CameraInfo &info)
{
  cam = new osg::Camera;
  pose = new osg::MatrixTransform(info.pose); // TODO check if Gazebo and OSG use same convention for cameras

  pose->addChild(cam);
  info.parent_node->addChild(pose);

  publisher = node->image_transport().advertise(info.topic, 1);

  publish_timer = node->create_wall_timer(std::chrono::milliseconds(info.period_ms),
                                          [&](){publish();});
}

void Camera::publish()
{
  // TODO build image from camera -> sensor_msgs


}


}
