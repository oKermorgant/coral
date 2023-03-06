#include <coral/coral_node.h>

using namespace coral;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  osg::setNotifyLevel(osg::WARN);

  auto node{std::make_shared<CoralNode>()};
  auto viewer{node->getViewer()};

  std::thread ros([&](){rclcpp::spin(node);});

  while(!viewer->done() && rclcpp::ok())
    viewer->frame();

  if(rclcpp::ok())
    rclcpp::shutdown();
  ros.join();

  return 0;
}
