#ifndef CORAL_NODE_H
#define CORAL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <coral/srv/spawn.hpp>
#include <coral/scene_params.h>
#include <coral/viewer.h>
#include <coral/link.h>

namespace coral
{

class OceanScene;
class Viewer;

using geometry_msgs::msg::Pose;
using coral::srv::Spawn;

class CoralNode : public rclcpp::Node
{
public:
  CoralNode();

  void manage(osg::ref_ptr<OceanScene> scene, Viewer &viewer);

  SceneParams parameters();

  void findModels();

private:
  Link world_link{"world"};
  OceanScene* scene;
  Viewer* viewer;

  // tf interface
  void refreshLinkPoses();
  Buffer tf_buffer;

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

  // how to get them
  void spawnModel(const std::string &model_ns, const std::string &pose_topic = "", const std::string &world_model = "");
  void parseModel(const std::string &model);
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;

  // camera view point 
  bool has_cam_view = false;
  Link* getKnownCamParent();
};

}

#endif // CORAL_NODE_H
