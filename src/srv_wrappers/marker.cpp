// an executable to call a service from a launch file

#include <rclcpp/rclcpp.hpp>
#include <coral/srv/marker.hpp>
#include "random_node.h"

using coral::srv::Marker;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node{random_node("coral_marker")};

  // arguments reflect service call
  auto request = std::make_shared<Marker::Request>();

  request->topic = node->declare_parameter<std::string>("topic", "");
  auto rgb{node->declare_parameter<std::vector<double>>("color", {1.,0.,0.})};
  rgb.resize(3, 1.);
  std::copy(rgb.begin(), rgb.end(), request->rgb.begin());

  if(!node->declare_parameter("delete", false))
  {    
    // wait for this topic to spawn and identify the message type
    while(true)
    {
      const auto topics{node->get_topic_names_and_types()};
      const auto here{topics.find(request->topic)};
      if(here != topics.end() && !here->second.empty())
      {
       request->message = here->second[0];
       RCLCPP_INFO(node->get_logger(), "Spawn marker for %s on topic %s",
                   request->message.c_str(),
                   request->topic.c_str());
       break;
      }
      rclcpp::spin_some(node);
      sleep(1);
    }
  }

  auto client = node->create_client<Marker>("/coral/marker");

  client->wait_for_service();

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }
  [[maybe_unused]] auto result = result_future.get();
  rclcpp::shutdown();
  return 0;
}
