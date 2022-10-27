## Vision Based Manipulation RBE450x
This course teaches 
* basics of manipulation
* Grasp Matrix
* Grasp Stability
* Image Processing basics
* Visual Servoing
  * Image Jacboian
---

This repository contains obsidian notes, reference pdfs, and tutorials


To run the homework4 in docker container created from [here](https://github.com/saikrn112/rbe450x_ros2)
```
python start_docker_instance.py --display --it
rbe450HW4 #sources files for HW4
ros2 launch rrbot_gazebo rrbot_world.launch.py
ros2 run rrbot_gazebo publisher
ros2 run rrbot_gazebo switch
ros2 run visual_servo_controller visual_servo_controller
```
