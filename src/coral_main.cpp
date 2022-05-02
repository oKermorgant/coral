#include <coral/coral_node.h>
#include <coral/Scene.h>
#include <coral/viewer.h>

using namespace coral;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  osg::setNotifyLevel(osg::WARN);

  auto node(std::make_shared<CoralNode>());

  Scene scene(node->parameters());
  Viewer viewer(scene);

  node->manage(scene, viewer);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  [[maybe_unused]] auto future = std::async([&node]()
  {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    node->findModels();
  });

  while(rclcpp::ok() && !viewer.done())
  {
    exec.spin_once();
    viewer.frame();
  }

  if(rclcpp::ok())
    rclcpp::shutdown();

  return 0;
}
