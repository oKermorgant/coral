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
    RCLCPP_INFO(node->get_logger(), "successfully spawned robot in namespace ", request->robot_namespace.c_str());
  rclcpp::shutdown();
  return 0;
}
