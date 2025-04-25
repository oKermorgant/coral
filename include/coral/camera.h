#ifndef CORAL_CAMERA_SIM_H
#define CORAL_CAMERA_SIM_H

#include <osg/Camera>
#include <coral/OceanScene.h>
#include <coral/urdf_parser.h>
#include <image_transport/image_transport.hpp>
#include <osg/NodeTrackerCallback>

namespace coral
{

class CoralNode;

class Camera
{

  static inline std::unique_ptr<image_transport::ImageTransport> transport;
  static inline rclcpp::Node::SharedPtr node;
  static inline osg::Group* world;
  static inline coral::OceanScene* scene;
  static inline std::vector<std::unique_ptr<Camera>> cameras;
  static inline rclcpp::CallbackGroup::SharedPtr cam_cb_group;
  static inline rclcpp::executors::MultiThreadedExecutor::SharedPtr cam_executor;

public:

  static inline void observe(osg::Group *world, coral::OceanScene* scene)
  {
    Camera::world = world;
    Camera::scene = scene;
  }
  static void addCameras(osg::Group* parent, const std::vector<urdf_parser::CameraInfo> &new_cameras);

  inline static void reloadShaders()
  {
    std::cout << "Reloading shaders for cameras" << std::endl;
    for(auto &cam : cameras)
      cam->loadShaders();
  }

  inline void saveBuffer()
  {
    const auto lock{std::lock_guard(mtx)};
    image = new osg::Image(*buffer.get());
  }

  void loadShaders();

private:
  osg::ref_ptr<osg::Camera> cam;
  osg::ref_ptr<osg::Image> image, buffer;
  osg::ref_ptr<osg::MatrixTransform> pose;
  std::mutex mtx;

  sensor_msgs::msg::Image msg;
  image_transport::Publisher publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;

  Camera(const urdf_parser::CameraInfo &info);
  void publish();
};
}
#endif

