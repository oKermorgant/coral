#ifndef CORAL_NODE_H
#define CORAL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rosgraph_msgs/msg/clock.hpp>

#include <coral/srv/spawn.hpp>
#include <coral/srv/surface.hpp>
#include <coral/OceanScene.h>
#include <coral/viewer.h>
#include <coral/link.h>
#include <coral/camera.h>

namespace coral
{



using geometry_msgs::msg::Pose;
using coral::srv::Spawn;
using coral::srv::Surface;

class CoralNode : public rclcpp::Node
{
 using GeometryType = decltype(urdf::Geometry::MESH);

public:
  CoralNode();
  inline Viewer* getViewer()
  {
    return &viewer;
  }

  SceneParams parameters();

  image_transport::ImageTransport& image_transport() const {return *it;}

  void findModels();

private:
  Link world_link{"world"};
  osg::ref_ptr<OceanScene> scene;
  Viewer viewer;

  // tf interface
  rclcpp::TimerBase::SharedPtr pose_update_timer;
  void refreshLinkPoses();
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // ground truth subscribers from Gazebo or other Pose topic
  std::vector<rclcpp::Subscription<Pose>::SharedPtr> pose_subs;

  // links and their meshes
  std::vector<std::string> known_model_namespaces;
  inline bool hasModel(const std::string &model) const
  {
    return std::find(known_model_namespaces.begin(), known_model_namespaces.end(), model) != known_model_namespaces.end();
  }
  bool display_thrusters = false;
  std::vector<Link> links;
  std::vector<Camera> cameras;
  std::unique_ptr<image_transport::ImageTransport> it;
  void addCameras(const std::vector<urdf_parser::CameraInfo> &cams);

  // how to get them
  rclcpp::Service<Spawn>::SharedPtr spawn_srv;  
  void spawnModel(const std::string &model_ns, const std::string &pose_topic = "", const std::string &world_model = "");
  void parseModel(const std::string &model);
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;

  // camera view point
  const std::string coral_cam_link = "coral_cam_view";
  bool has_cam_view = false;
  Link* getKnownCamParent();
};

}

#endif // CORAL_NODE_H
