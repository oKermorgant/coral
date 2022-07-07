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
#include <coral/Scene.h>
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
  inline void manage(Scene &scene, Viewer &viewer)
  {
    this->scene = &scene;
    this->viewer = &viewer;
    world_link.attachTo(this->scene);
  }

  SceneParams parameters();

  image_transport::ImageTransport& image_transport() const {return *it;}

  void findModels();

private:
  Scene * scene;
  Viewer * viewer;

  // tf interface
  rclcpp::TimerBase::SharedPtr pose_update_timer;
  void refreshWorldParams();
  void refreshLinkPoses();
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  inline bool hasLink(const std::string& name, const std::vector<std::string> &tf_links) const
  {
    return std::find(tf_links.begin(), tf_links.end(), name) != tf_links.end();
  }

  // ground truth subscribers from Gazebo
  std::vector<rclcpp::Subscription<Pose>::SharedPtr> pose_subs;

  // links and their meshes
  std::vector<std::string> models;
  inline bool hasModel(const std::string &model) const
  {
    return std::find(models.begin(), models.end(), model) != models.end();
  }
  bool display_thrusters = false;
  std::vector<Link> links;
  std::vector<Camera> cameras;
  std::unique_ptr<image_transport::ImageTransport> it;

  // how to get them
  rclcpp::Service<Spawn>::SharedPtr spawn_srv;  
  void spawnModel(const std::string &model_ns, const std::string &pose_topic);
  void parseModel(const std::string &model);
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;

  // camera view point
  Link world_link{"world"};
  const std::string coral_cam_link = "coral_cam_view";
  bool has_cam_view = false;
  Link* getKnownCamParent();

  // surface height
  rclcpp::Service<Surface>::SharedPtr surface_srv;
  void computeSurface(const Surface::Request &req, Surface::Response &res);
};

}

#endif // CORAL_NODE_H
