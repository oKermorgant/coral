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

class Scene;
class Viewer;

using geometry_msgs::msg::Pose;
using coral::srv::Spawn;
using rcl_interfaces::msg::SetParametersResult;

class CoralNode : public rclcpp::Node
{
public:
  CoralNode();

  void manage(Scene& scene, Viewer &viewer);

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
  Scene* scene;
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
  std::vector<std::unique_ptr<Link>> links;
  inline Link* findLink(const std::string &name) const
  {
    auto link{std::find_if(links.begin(), links.end(), [&](auto &link){return link->name() == name;})};
    return link == links.end() ? nullptr : link->get();
  }

  // how to get them
  void spawnModel(const std::string &model_ns, const std::string &pose_topic = "", const std::string &world_model = "");
  /// add a model and returns the root link
  Link* addModel(const std::string &model);
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub;

  // camera view point if requested
  bool updateViewerPose();

  // helper functions to declare and describe parameters
  template <typename ParamType>
  inline void declareParamDescription(std::string name,
                                           ParamType &value,
                                           std::string description = "")
  {
    if(has_parameter(name))
    {
      get_parameter(name, value);
      return;
    }
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    value = declare_parameter<ParamType>(name, value, descriptor);
    //value = declare_parameter<ParamType>(name, value);
  }

  inline void declareParamDescription(std::string name,
                                     int &value,
                                     int lower,
                                     int upper,
                                     int step = 1,
                                      std::string description = "")
  {
    if(has_parameter(name))
    {
      get_parameter(name, value);
      return;
    }
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()
                                .set__from_value(lower)
                                .set__to_value(upper)
                                .set__step(step)};
    value = declare_parameter<int>(name, value, descriptor);
  }

  inline void declareParamDescription(std::string name,
                                     double &value,
                                     double lower,
                                     double upper,
                                      std::string description = "")
  {
    if(has_parameter(name))
    {
      get_parameter(name, value);
      return;
    }
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                .set__from_value(lower)
                                .set__to_value(upper)};
    value = declare_parameter<double>(name, value, descriptor);
  }

  OnSetParametersCallbackHandle::SharedPtr param_change;
  SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};

}

#endif // CORAL_NODE_H
