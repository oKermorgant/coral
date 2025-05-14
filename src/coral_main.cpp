#include <sensor_msgs/msg/image.hpp>
#include <coral/coral_node.h>
#include <coral/scene.h>
#include <coral/viewer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <coral/scene_lock.h>
#include <osgWidget/WindowManager>

using namespace coral;

//#define TEST_CAM

#ifdef TEST_CAM

#include "../tests/moving_cam.h"
#endif



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  osg::setNotifyLevel(osg::WARN);

  auto node{std::make_shared<CoralNode>()};
  Scene scene(node->parameters());
  auto viewer = Viewer(scene);

#ifdef TEST_CAM

  CamTrackerCallback::ocean_scene = scene.getOceanScene();
  MovingCam cam;
  auto sub = node->create_subscription<geometry_msgs::msg::Pose>("camera_pose", 1,
                                                                 [&cam](const geometry_msgs::msg::Pose::SharedPtr pose)
                                                                 {
                                                                   cam.update(pose);
                                                                 });

  cam.publisher = node->create_publisher<sensor_msgs::msg::Image>("camera", 1);
  cam.reloadShaders(scene.getOceanScene());

  scene.getOceanScene()->addChild(cam.cam_pose);
  cam.view->setSceneData(scene.getRoot());
  //cam.view->setUpViewInWindow( 150, 150, 800,600, 0 );
  viewer.addView(cam.view);

  auto widget = cam.getWidgetWindow();

  widget->setX(0);
  widget->setY(0);
 // viewer.wm->addChild(widget);
  //widget->show();

  osg::ref_ptr < osg::Group > appgroup = new osg::Group();
  osg::ref_ptr < osg::Camera > appcamera = viewer.wm->createParentOrthoCamera();
  appgroup->addChild(appcamera);
    appgroup->addChild(scene.getRoot());
  viewer.main_view->setSceneData(appgroup);


  cam.cam->addChild(scene.getScene());
//scene.getRoot()->addChild(cam.cam.get());

//scene->connect(cam.get());*/
#else
  node->manage(scene, viewer);
#endif

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  std::thread ros([&](){exec.spin();});

  //std::this_thread::sleep_for(std::chrono::seconds(10));

  while(!viewer.done() && rclcpp::ok())
  {
    viewer.frame();
  }

  if(rclcpp::ok())
    rclcpp::shutdown();
  ros.join();

  return 0;
}
