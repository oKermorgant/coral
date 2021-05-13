#include <coral/scene_interface.h>
#include <coral/coral_node.h>
#include <coral/viewer.h>

using namespace coral;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  osg::setNotifyLevel(osg::FATAL);

  auto node(std::make_shared<CoralNode>());
  SceneInterface interface(node->nodeParams());
  node->setInterface(interface);

  Viewer viewer(interface);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while(rclcpp::ok() && !viewer.done())
  {
    exec.spin_once();
    viewer.frame();
  }

  if(rclcpp::ok())
    rclcpp::shutdown();

  return 0;
}
