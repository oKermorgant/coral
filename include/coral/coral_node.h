#ifndef CORAL_NODE_H
#define CORAL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <coral/srv/spawn.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <coral/Scene.h>
#include <coral/viewer.h>
#include <coral/link.h>
#include <coral/camera.h>

namespace coral
{

using nav_msgs::msg::Odometry;
using coral::srv::Spawn;

class CoralNode : public rclcpp::Node
{
 using GeometryType = decltype(urdf::Geometry::MESH);


public:
  CoralNode();
  inline void manage(Scene &scene, Viewer &viewer)
  {
    this->scene = &scene;
    this->viewer = &viewer;
  }

  SceneParams parameters();

  image_transport::ImageTransport& image_transport() const {return *it;}

private:
  Scene * scene;
  Viewer * viewer;

  // tf interface
  rclcpp::TimerBase::SharedPtr world_sim_timer, pose_update_timer;
  void refreshWorldParams();
  void refreshLinkPoses();
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  inline bool hasLink(const std::string& name, const std::vector<std::string> &tf_links) const
  {
    return std::find(tf_links.begin(), tf_links.end(), name) != tf_links.end();
  }

  std::vector<rclcpp::Subscription<Odometry>::SharedPtr> odom_subs;  
  void odomCallback(const std::string &link_name, const geometry_msgs::msg::Pose &pose);

  // links and their meshes
  bool display_thrusters = false;
  bool world_is_parsed = false;
  Link world_link;
  std::vector<Link> abs_links, rel_links;
  std::vector<Camera> cameras;
  std::unique_ptr<image_transport::ImageTransport> it;

  // how to get them
  rclcpp::Service<Spawn>::SharedPtr spawn_srv;
  void spawnModel(const std::string &model_ns, const std::string &pose_topic);
  void parseModel(const std::string &model, bool moving);
  void parseWorld();
  void addWorldVisual(rclcpp::SyncParametersClient::SharedPtr pwm_client, const std::string &name, GeometryType geometry);

  // camera view point
  const std::string coral_cam_link = "coral_cam_view";
  bool has_cam_view = false;
  Link* getKnownCamParent();
};

}

#endif // CORAL_NODE_H
