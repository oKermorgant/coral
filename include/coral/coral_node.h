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

  struct ScopedTimer
  {
    using Clock = std::chrono::high_resolution_clock;
    std::string msg;
    Clock::time_point start;
    Node* node;
    inline explicit ScopedTimer(const std::string &msg, Node * node) : msg{msg}, start{Clock::now()}, node{node}
    {}
    inline ~ScopedTimer()
    {
      RCLCPP_INFO(node->get_logger(), "%s... took %ld μs", msg.c_str(),
                  std::chrono::duration_cast<std::chrono::microseconds>(Clock::now()-start).count());
    }
  };



  Link world_link{WORLD_NAME};
  OceanScene* scene;
  Viewer* viewer;

  // tf interface
  void refreshLinkPoses();
  Buffer tf_buffer{this};

  // ground truth subscribers from Gazebo or other Pose topic
  std::vector<rclcpp::Subscription<Pose>::SharedPtr> pose_subs;

  // links and their meshes
  std::vector<std::string> known_model_namespaces;
  inline bool hasModel(const std::string &model) const
  {
    return std::find(known_model_namespaces.begin(), known_model_namespaces.end(), model) != known_model_namespaces.end();
  }
  bool display_thrusters = false;
  std::vector<std::unique_ptr<Link>> links;
  inline Link* findLink(const std::string &name) const
  {
    auto link{std::find_if(links.begin(), links.end(), [&](auto &link){return link->name() == name;})};
    return link == links.end() ? nullptr : link->get();
  }

  // how to get them
  void spawnModel(const std::string &model_ns, const std::string &pose_topic = "", const std::string &world_model = "");
  /// add a model and returns the root link
  Link *parseModel(const std::string &model);
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;

  // camera view point 
  bool has_cam_view = false;
  Link* getKnownCamParent();
};

}

#endif // CORAL_NODE_H
