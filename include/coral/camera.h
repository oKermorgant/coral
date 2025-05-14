#ifndef CORAL_CAMERA_SIM_H
#define CORAL_CAMERA_SIM_H

#include <osg/Camera>
#include <coral/scene.h>
#include <coral/urdf_parser.h>
#include <image_transport/image_transport.hpp>
#include <osg/NodeTrackerCallback>
#include <osgWidget/Box>
#include <osgWidget/Window>
#include <coral/viewer.h>

namespace coral
{

class CoralNode;

class Camera
{

  static inline std::unique_ptr<image_transport::ImageTransport> transport;
  static inline rclcpp::Node::SharedPtr node;
  static inline coral::Scene* scene;
  static inline std::vector<std::unique_ptr<Camera>> cameras;  
  static inline rclcpp::executors::MultiThreadedExecutor::SharedPtr cam_executor;

public:

  static inline void observe(rclcpp::Node::SharedPtr node, Scene* scene)
  {
    Camera::node = node;
    Camera::transport = std::make_unique<image_transport::ImageTransport>(node);
    Camera::scene = scene;
  }
  static void addCameras(osg::Group* parent,
                         const std::vector<urdf_parser::CameraInfo> &new_cameras,
                         Viewer* viewer);

  inline static void reloadShaders()
  {
    if(cameras.empty())
      return;
    std::cout << "Reloading shaders for cameras" << std::endl;
    for(auto &cam : cameras)
      cam->loadShaders();
  }

  void saveBuffer();
  void loadShaders();

  osg::ref_ptr<osgWidget::Window> getWidgetWindow()
  {
    osg::ref_ptr < osgWidget::Box > box = new osgWidget::Box("VirtualCameraBox", osgWidget::Box::HORIZONTAL, true);
    osg::ref_ptr < osgWidget::Widget > widget = new osgWidget::Widget("VirtualCameraWidget", msg.width, msg.height);
    widget->setImage(image.get(), true, false);
    box->addWidget(widget.get());
    box->getBackground()->setColor(1.0f, 0.0f, 0.0f, 0.8f);
    box->attachMoveCallback();
    box->attachScaleCallback();
    return box;
  }

  void publish();

private:
  osg::ref_ptr<osg::Camera> cam;
  osg::ref_ptr<osg::Image> image;
  osg::ref_ptr<osg::MatrixTransform> pose;  
  std::mutex mtx;

  rclcpp::CallbackGroup::SharedPtr cam_cb_group;
  sensor_msgs::msg::Image msg;
  image_transport::Publisher publisher;
  double period_s;
  rclcpp::Time last_save;
  rclcpp::TimerBase::SharedPtr publish_timer;

  Camera(const urdf_parser::CameraInfo &info);

};
}
#endif

