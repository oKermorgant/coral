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
  static inline std::unique_ptr<image_transport::ImageTransport> transport;
  static inline rclcpp::Node::SharedPtr node;  
  static inline osg::Group* scene;
  static inline std::vector<std::unique_ptr<Camera>> cameras;

public:

  static inline void observe(osg::Group* scene) {Camera::scene = scene;}
  static void addCameras(osg::Group* parent, const std::vector<urdf_parser::CameraInfo> &new_cameras);

private:
  osg::ref_ptr<osg::Camera> cam;
  osg::ref_ptr<osg::Image> image;
  osg::ref_ptr<osg::MatrixTransform> pose;

  sensor_msgs::msg::Image msg;
  image_transport::Publisher publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;

  Camera(const urdf_parser::CameraInfo &info);
  void publish();


};
}
#endif

