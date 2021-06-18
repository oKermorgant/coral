# coral - osgOcean-based gui for Plankton

This ROS 2 package allows displaying Plankton-simulated scenes with osgOcean.

It is inspired from the well-known UWSim in ROS 1.

Contrary to UWSim, Coral is not a simulator at all and only renders available models found on the TF tree. It also renders world models published by Plankton.

 - links on the TF tree are parsed only if they have a name of the form `<robot>/link`. In this case, Coral will try to find the corresponding model from the `/<robot>/robot_state_publisher` node. It this does not exist, the link will be definitely ignored
 - world links are assumed to be a `meshes` parameter set by the `/publish_world_models` node. Run a Plankton tutorial to see how to write these parameters
 - a special link is `coral_cam_node`: if this link is found in the TF tree then Coral will use this frame to place its camera. If this link is not refreshed for 1 sec then the camera will be free-flying again
 
## Nodes and parameters

The only node is `coral` and has parameters to tune the scene rendering: wind, waves and foam, initial camera position, water surface reflection and refraction, or toggle glare or godrays.

A helper launch file is `track_launch.py` and simply publishes a static transform between the given `link` and the `coral_cam_node` link, in order to automatically track a 
