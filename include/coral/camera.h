#ifndef CORAL_CAMERA_SIM_H
#define CORAL_CAMERA_SIM_H

#include <string>
#include <vector>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <image_transport/image_transport.hpp>
#include <tinyxml.h>

namespace coral
{

class CoralNode;


// temporary structure to extract the camera info before building an actual simulated camera
struct CameraInfo
{
  int height, width;
  double fov;
  double clip_near, clip_far;
  std::string topic;
  std::string link_name;
  int period_ms;
  osg::MatrixTransform* pose = new osg::MatrixTransform;

  CameraInfo(std::string link, const TiXmlElement* sensor_elem);

  static std::vector<CameraInfo> extractFrom(const std::string &description);

};

class Camera
{
public:
  Camera(CoralNode *node, const CameraInfo &info);


private:
  osg::ref_ptr<osg::Camera> cam;
  osg::ref_ptr<osg::Image> image;
  osg::ref_ptr<osg::MatrixTransform> pose;

  sensor_msgs::msg::Image msg;
  image_transport::Publisher publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;

  void publish(CoralNode *node);


};

}
#endif

