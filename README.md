Description of this package
---
This package is an interface to MoveIt motion planning library. You will learn to use ros service to send commands to MoveIt, thus making the manipulator moving around.


Some useful links:
---
**_This is a pretty good tutorial for learning ROS._**
**What is ROS?**: Please look @ the tutorial here: http://wiki.ros.org/ROS/Tutorials.

**_It would be better if you can understand some basic concepts like workspace, package, message, service... before you start PS2, so the following are some useful links that get you prepared_**
**What is a workspace?** Here: http://wiki.ros.org/catkin/workspaces
**What is a ROS package?** Here: http://wiki.ros.org/Packages
**What is a ROS message?** Here: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
**What is ROS topics?**: Here: http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
**What is ROS service?** Here: http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams
**How to write a simple service client?** Here: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

**_Some extra resources_**
**Parameter server** Here: http://wiki.ros.org/Parameter%20Server
**Parameter namespace** Here: http://wiki.ros.org/Names

Run the package:
---
1. Download the tar file ‘denso_robot_plugin.tar’
2. Open a terminal (terminator recommended)
3. Untar it: 
    ``tar -xvf denso_robot_plugin.tar``
4. Go into the folder: 
    ``cd denso_robot_plugin``
5. Compile the code: 
    ``catkin_make``
6. Source the resulting ros packages: 
    ``source devel/setup.bash``
7. Run the denso_robot_ros package with MoveItRviz & Gazebo: 
    ``roslaunch denso_robot_bringup denso_robot_bringup.launch``
8. Open another terminal (Ctrl+Shft+E in Terminator) and run the denso_robot_interface: 
    ``roslaunch denso_robot_interface denso_robot_interface.launch``
9. You should be able to two GUIs: One is the GAZEBO interface and another is the MoveIt window. If either of them didn't show up, try restarting the processed by CTRL-C and repeat step 7 and 8
10. Now you should be able to start working on PS2 using ros service provided by denso_robot_interface node. Details of services can be found in the following section.


Services provided:
---
**ROS provides a useful commandline tool for checking and calling currently available services, see here for more details: http://wiki.ros.org/rosservice.**
**Basically, the syntax for calling a rosservice is:
    ``rosservice call [service_name] [*kargs...]``**

**Here we listed the services you might need for this Problem Set:**
|-------------------------------------------|--------------------|-----------------------|------------------------------------------|
| service_name                              | arguments          | return _value         | description                              |
|-------------------------------------------|--------------------|-----------------------|------------------------------------------|
| /denso_robot_interface/mark_state         | State Name         | Success? [Reason]     | mark down the current robot state        |
| /denso_robot_interface/show_states        | Empty              | Success? [Reason]     | show all the states that is marked down  |
| /denso_robot_interface/clear_states       | Empty              | Success? [Reason]     | clear all the states that is marked down |
| /denso_robot_interface/upload_states      | File Name          | Success? [Reason]     | save the marked-down states to YAML file |
| /denso_robot_interface/execute_trajectory | Empty              | Success? [Reason]     | execute the marked-down states in order  |
| /denso_robot_interface/go_to              | geometry_msgs/Pose | Success? [Reason]     | go to a Pose target                      |
| /denso_robot_interface/translation        | float64[]          | Success? [Percentage] | follow a translation-only trajectory     |
|-------------------------------------------|--------------------|-----------------------|------------------------------------------|
**Note that you can always use [TAB] to perform command auto-completion (including arguments). If you pressed [TAB], waited and nothing happened, do ``source devel/setup.bash`` in the current command window and try again**
**mark_state ONLY save the states dynamically, that is, if you restart the ROS, everything you've been working on will be lost. Use upload_states to save the states on your hard drive. The default folder for your saved YAML file is "denso_robot_interface/yaml/your_file.yaml"**_


Hints for this Problem Set:
---
**For PS2-1**: All you need to do is to move the manipulator to the ideal pose, mark down its current pose. This will, in the end, give you a "trajectory" that you can execute.

**For PS2-2**: Please take a look @ "denso_robot_interface/nodes/ps2-2.py". What you need to do is to compose a trajectory using geometry_msgs/Pose, you can find the details of this ROS message here: http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html

TIPS:
---
1. Well, all of these are Open Source Software, so don't expect them to be as robust as your Microsoft Office. RViz and GAZEBO tends to crash for no reason(this does not mean that it should take the blame everytime it creashes). Here is a trouble shooting page for RViz: http://wiki.ros.org/rviz/Troubleshooting and GAZEBO: http://wiki.ros.org/simulator_gazebo/Troubleshooting
2. Often, when you exit the program, it will emit ERRORs like **[vs060/controller_spawner-5] escalating to SIGTERM**, this is fine, ignore it.
3. The virtual box is kinds of slow... So, be patient...
4. If you are not familiar with Quaternion, here is a description: http://mathworld.wolfram.com/Quaternion.html. And there are many converters online, too.
5. Google is always your best friend when developping!
