#ifndef CORALNODE_H
#define CORALNODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <coral/scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <coral/visual_link.h>
#include <std_msgs/msg/string.hpp>

namespace coral
{
using std_msgs::msg::String;

class CoralNode : public rclcpp::Node
{
public:
  CoralNode();
  void setInterface(SceneInterface &scene);
  SceneParams nodeParams();

private:
  SceneInterface * scene;

  rclcpp::TimerBase::SharedPtr tree_parser_timer, pose_update_timer;
  void parseTFTree();
  void refreshLinkPoses();
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // links and their meshes
  bool world_is_parsed = false;
  std::vector<std::string> parsed_links;
  std::vector<VisualLink> links;
  void parseLink(const std::string &link);
  void parseModel(const std::string &model);
  void parseWorld();

  // camera view point
  osg::Matrixd cam_view;
  const std::string coral_cam_link = "coral_cam_view";
  bool has_cam_view = false;
};

}

#endif // CORALNODE_H
