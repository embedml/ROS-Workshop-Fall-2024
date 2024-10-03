# Workshop 6


The goals of this workshop are to learn how to create a ROS Package, create ROS Actions, and use ROS Action Servers.


## Milestone 0: Set up workshop and create a package.

```
git clone git@github.ncsu.edu:software-engineering-for-robotics/workshop-6-actions.git
```


* [Open the form in a new tab](https://forms.gle/swg13ZHJA6EH7UxE6)

* Build the workshop.

* Start the container.


Now let's create a new __package__ for our action.
```
mkdir -p action_ws/src
cd action_ws/src
```


```
ros2 pkg create action_workshop
```
You should see something like:

```
going to create a new package
package name: action_tutorials
destination directory: /root/action_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['root <root@todo.todo>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
creating folder ./action_tutorials
creating ./action_tutorials/package.xml
creating source and include folder
creating folder ./action_tutorials/src
creating folder ./action_tutorials/include/action_tutorials
creating ./action_tutorials/CMakeLists.txt

[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identifiers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0
```

Notice the `version 0.0.0` -- This is the default version number in "[Semantic Versioning Number](https://en.wikipedia.org/wiki/Software_versioning)", where the first number is the major version, the second number is the minor version, and the third number is the smallest increment.

* Open `package.xml` with an editor (like `nano`) and bump the version number ("bump" means increase it by the smallest increment).  
* Also in `package.xml`, change the `license` to some license like `MIT` (see list above).
* Also in `package.xml`, change the maintainer to your UNITY_ID
* Also in `package.xml`, set up the maintainer email address.

Take a screen shot of the changed package information for the workshop form.


## Milestone 1: Create a "Action"

ROS "Actions" are a way to organize your robot's capabilites into a higher level that just publishing and receiving messages.  Actions are interruptable tasks that take more than an instant to complete.  Actions are also a data structure, containing three parts:

Action File
---

* Goal / Request
* Result
* Feedback

In the `action_workshop` folder, create a folder `action` and a file called `VerboseMoveBase.action` (This is a modified version of the [MoveBase](http://docs.ros.org/api/move_base_msgs/html/action/MoveBase.html) action message).  Add you UNITY_ID to the first line of the file.


```
# UNITY_ID
geometry_msgs/PoseStamped target_pose    # GOAL
---
geometry_msgs/PoseStamped final_pose  #  RESULT
---
geometry_msgs/PoseStamped base_pose  #  FEEDBACK
```

Notice that the Goal, Result, and Feedback are the same type: a stamped pose message.  Answer the question in the form about this action message type.


Now edit the `CMakeLists.txt` file in the `action_workshop` package and make the following 2 changes:

1) add `find_package(geometry_msgs REQUIRED)` at the end of the `# find dependencies` section.

2) add the following lines immediately before `ament_package()` :

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/VerboseMoveAction.action"
  DEPENDENCIES geometry_msgs
)
```

Now edit `package.xml` and add the following after `<buildtool_depend>ament_cmake</buildtool_depend>`:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

(_warning: this next command is going to fail and you'll have to figure out why_.) Now we can build our package. Navigate to the `action_ws` folder, then:
```
colcon build
```

Oh no! something is wrong....  Read the error message from `colcon build` and examine the `CMakeLists.txt` and `Package.xml` files to determine what needs to be fixed.  Pay close attention to the names of files.

When you have made the change, you should be able to run `colcon build` successfully, which will look like this:
```
Starting >>> action_workshop
Finished <<< action_workshop [5.46s]                     

Summary: 1 package finished [5.59s]
```

For the activity form, describe what you fixed.


Congratulations now you have created a ROS2 action message which communicates both a request for an action and the result.

Now apply the `overlay` for this package:
```
COMMAND FOR SOURCING THE OVERLAY
```
and check that it worked:
```
ros2 interface show action_workshop/action/VerboseMoveBase
```
Take a screen shot of the results of the previous command and upload it to the activity form and make sure you've edited the `action` file so it shows your UNITY_ID.



## Milestone 2: Create an Action Server.



Copy the file `sample_action_server.py` into the container (how can you get code into the container if you didn't link the volume?.  You could copy it into the `/root` directory although you could put it anywhere and it should still work after the overlay (why?).

Now run the action server:
```
python3 sample_action_server.py
```

In a new terminal window apply the `action_ws` overlay. Then, try sending a goal to the action server that will command a pose to move to x=10, y=20, z=30:
```
ros2 action send_goal --feedback move_base action_workshop/action/VerboseMoveBase "{target_pose: {pose: {position: {x: 10, y: 20, z: 30}}}}"
```

Watch how the action `Feedback` shows an `x`, `y`, and `z` `position` (part of the pose message) changes over time.  You might notice that it changes position very slowly.  The goal is published but it failes to converge to the goal pose quickly. 

Stop the action server with `ctrl-c ctrl-c`. Examine the `sample_action_server.py` code.  The sample action server includes an implementation of a simple kind of "controller" (from ["Control Theory"](https://en.wikipedia.org/wiki/Control_theory)) called a ["Proportional-Integral-Derivative" ("PID") controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller), but with only the proportional ("P") part. 


Why is the action server moving the `xyz` co-ordinates so slowly?  Give your answer in the activity form.  
  
igure out how to make work faster by editing `sample_action_server.py` and look especially for the `kP` term --- if this takes more than 10 minutes to figure it out, ask for help!

Now that you've changed the `sample_action_server.py` code to make the PID controller action move the system more quickly, try invoking the action server again with the same commend:
```
ros2 action send_goal --feedback move_base action_workshop/action/VerboseMoveBase "{target_pose: {pose: {position: {x: 10, y: 20, z: 30}}}}"
```

After making your change, re-issue the goal and take a screen shot of the action server successfully reaching the goal.

How does the function `is_close_enough()` work?



## Milestone 3:  Create your own Action Client.

Use [this tutorial as a model](https://docs.ros.org/en/iron/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html) and create your own `action_client.py` file that imports the `VerboseMoveBase` action and executes that action with a goal `x=20, y=30, z=40`.   Run the client from the command line and take a screen shot with your UNITY_ID typed into the terminal.


## Milestone 4: using and mock robot aka "dummy robot".

This is not related to actions, and is an introduction to some material we'll be covering in the upcoming weeks.  This akin to ['mocking'](https://en.wikipedia.org/wiki/Mock_object), where we create a fake object to test and build the interfaces.  In this case, we're using a fake robot, aka "Dummy Robot."


```
apt-get update
apt-get install -y --fix-missing ros-$ROS_DISTRO-dummy-robot-bringup
```

Start up the `dummy_robot`:
```
ros2 REMEMBER_THIS_COMMAND?  dummy_robot_bringup dummy_robot_bringup_launch.py
```

In a new terminal, recall the command to determine the message rate for the `/scan` message and take a screen shot.  

Upload the screen shot to the form :)


Last question for the form:  What are the 3 parts of an action message?

## END

Remember to complete the google form.

## Evaluation

This workshop is worth 5 points.

You'll be graded on the following rubric:

| ITEM | POINTS |
|--|--|
|Screen shots | 2.5 |
|Questions   | 2.5 |
