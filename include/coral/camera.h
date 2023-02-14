#ifndef CORAL_CAMERA_SIM_H
#define CORAL_CAMERA_SIM_H

#include <string>
#include <vector>
#include <osg/Camera>
#include <image_transport/image_transport.hpp>
#include <coral/urdf_parser.h>


namespace coral
{

class CoralNode;

class Camera
{
public:
  Camera(CoralNode *node, const urdf_parser::CameraInfo &info);


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

