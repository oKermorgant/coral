# Coral - osgOcean-based viewer for ROS 2

This ROS 2 package allows displaying simulated scenes with osgOcean. It is inspired from the well-known UWSim in ROS 1.

Contrary to UWSim, Coral is not a simulator at all and only renders available models found on the TF tree. It is compatible with Gazebo simulation, if the poses of the objects are published and bridges to ROS.

 - links on the TF tree are parsed only if they have a name of the form `<robot>/link`. In this case, Coral will try to find the corresponding model from the `/<robot>/robot_state_publisher` node. It this does not exist, the link will be definitely ignored
 - a special link is `coral_cam_node`: if this link is found in the TF tree then Coral will use this frame to place its camera. If this link is not refreshed for 1 sec then the camera will be free-flying again
 
 `coral_gui` is the main node and has parameters to tune the scene rendering: wind, waves and foam, initial camera position, water surface reflection and refraction, or toggle glare or godrays.
 
## Spawning a robot

Coral will provide the `/coral/spawn` service defined as:
```
string robot_namespace
string pose_topic "pose_gt"
string urdf_model
---
```
If `urdf_model` is defined, assume it is the content of some URDF file where `world` is the root frame (typically a world mesh).
Otherwise, will use the `robot_namespace` to listen for a `robot_description` topic and spawn the corresponding robot.
If `pose_topic` is define, Coral will use it to update the pose of the root frame of the robot. Otherwise, it will rely on `/tf`.

Note that Coral will always rely on `/tf` to get the relative pose of the links that belong to a given robot (typically published by `robot_state_publisher`). Only the root link can be chosen as being updated either from a topic, or from `/tf`.

The `spawn` executable is a wrapper around the `Spawn` service that is used to add a new model in the simulation
 
## Controlling the point of view

A helper launch file is `track_launch.py` and simply publishes a static transform between the given `link` and the `coral_cam_node` link, in order to automatically track the corresponding link inside Coral.
