// an executable to call a service from a launch file

#include <rclcpp/rclcpp.hpp>
#include <coral/srv/spawn.hpp>

using coral::srv::Spawn;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("coral_spawner");

  // arguments reflect service call
  auto request = std::make_shared<Spawn::Request>();

  request->robot_namespace = node->declare_parameter("namespace", "");
  request->pose_topic = node->declare_parameter("pose_topic", "pose_gt");
  request->urdf_model = node->declare_parameter("urdf_model", "");

  std::map<std::string, double> pose;
  for(auto key: {"x","y","z","R","P","Y"})
    pose[key] = node->declare_parameter(key, 0.);

  request->urdf_pose.position.x = pose["x"];
  request->urdf_pose.position.y = pose["y"];
  request->urdf_pose.position.z = pose["z"];

  const auto cy{cos(pose["Y"]/2)};
  const auto sy{sin(pose["Y"]/2)};
  const auto cp{cos(pose["P"]/2)};
  const auto sp{sin(pose["P"]/2)};
  const auto cr{cos(pose["R"]/2)};
  const auto sr{sin(pose["R"]/2)};

  request->urdf_pose.orientation.w = cr * cp * cy + sr * sp * sy;
  request->urdf_pose.orientation.x = sr * cp * cy - cr * sp * sy;
  request->urdf_pose.orientation.y = cr * sp * cy + sr * cp * sy;
  request->urdf_pose.orientation.z = cr * cp * sy - sr * sp * cy;

  auto client = node->create_client<Spawn>("/coral/spawn");

  client->wait_for_service();

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }
  [[maybe_unused]] auto result = result_future.get();
  if(request->robot_namespace.empty())
    RCLCPP_INFO(node->get_logger(), "successfully spawned new descriptions");
  else
    RCLCPP_INFO(node->get_logger(), "successfully spawned robot in namespace %s", request->robot_namespace.c_str());
  rclcpp::shutdown();
  return 0;
}
