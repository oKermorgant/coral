#ifndef CORAL_CAMERA_SIM_H
#define CORAL_CAMERA_SIM_H

#include <string>
#include <vector>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <image_transport/image_transport.hpp>
namespace coral
{

class CoralNode;


// temporary structure to extract the camera info before building an actual simulated camera
struct CameraInfo
{
  int height, width;
  float fov;
  float clip_near, clip_far;
  std::string topic;
  std::string link_name;
  osg::Matrixd pose;
  int period_ms;
  mutable osg::MatrixTransform* parent_node;

  inline const CameraInfo& attachedTo(osg::MatrixTransform* parent) const
  {
    parent_node = parent;
    return *this;
  }

  static std::vector<CameraInfo> extractFrom(const std::string &description);

};




class Camera
{
public:
  Camera(CoralNode *node, const CameraInfo &info);


private:
  osg::ref_ptr<osg::Camera> cam;
  osg::ref_ptr<osg::MatrixTransform> pose;
  image_transport::Publisher publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;

  void publish();


};

}
#endif

