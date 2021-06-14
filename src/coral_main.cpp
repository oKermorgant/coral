#include <coral/coral_node.h>
#include <coral/Scene.h>
#include <coral/viewer.h>

using namespace coral;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  osg::setNotifyLevel(osg::FATAL);

  auto node(std::make_shared<CoralNode>());

  Scene scene(node->parameters());
  Viewer viewer(scene);

  node->manage(scene, viewer);

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
