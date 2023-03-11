#ifndef CORAL_CAMERA_SIM_H
#define CORAL_CAMERA_SIM_H

#include <osg/Camera>
#include <coral/urdf_parser.h>
#include <image_transport/image_transport.hpp>

namespace coral
{

class CoralNode;

class Camera
{
  static std::unique_ptr<image_transport::ImageTransport> transport;
  static rclcpp::Node::SharedPtr node;
  static std::vector<Camera> cameras;

  Camera(const urdf_parser::CameraInfo &info);

public:

  static void addCameras(osg::Group* parent, const std::vector<urdf_parser::CameraInfo> &infos);

private:
  osg::ref_ptr<osg::Camera> cam;
  osg::ref_ptr<osg::Image> image;
  osg::ref_ptr<osg::MatrixTransform> pose;

  sensor_msgs::msg::Image msg;
  image_transport::Publisher publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;

  void publish();


};
}
#endif

