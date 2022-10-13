#include <coral/coral_node.h>

using namespace coral;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  osg::setNotifyLevel(osg::WARN);

  auto node(std::make_shared<CoralNode>());
  auto viewer{node->createViewer()};

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  [[maybe_unused]] auto future = std::async([&node]()
  {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    node->findModels();
  });

  std::thread ros([&](){exec.spin();});

  while(!viewer->done() && rclcpp::ok())
  {
    viewer->frame();
  }

  if(rclcpp::ok())
    rclcpp::shutdown();
  ros.join();

  return 0;
}
