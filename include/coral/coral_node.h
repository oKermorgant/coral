#ifndef CORAL_NODE_H
#define CORAL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <coral/Scene.h>
#include <coral/viewer.h>
#include <coral/link.h>
#include <coral/camera.h>

namespace coral
{

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
  void guessSimTime();
  rclcpp::TimerBase::SharedPtr tree_parser_timer, pose_update_timer;
  void parseTFTree();
  void refreshLinkPoses();
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // links and their meshes
  bool display_thrusters = false;
  bool world_is_parsed = false;
  Link world_link;
  std::set<std::string> parsed_links, parsed_models;
  std::vector<Link> links;
  std::vector<Camera> cameras;
  std::unique_ptr<image_transport::ImageTransport> it;

  // how to get them
  void parseModelFromLink(const std::string &link);
  void parseModel(const std::string &model);
  void parseWorld();
  void addWorldVisual(rclcpp::SyncParametersClient::SharedPtr pwm_client, const std::string &name, GeometryType geometry);

  // camera view point
  const std::string coral_cam_link = "coral_cam_view";
  bool has_cam_view = false;
};

}

#endif // CORAL_NODE_H
