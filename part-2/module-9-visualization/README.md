# Workshop 7 - Foxglove


The goals of this workshop are to learn how learn how to find and modify an existing URDF file and to use a visualization tool, Foxglove, to view runtime changes to ROS2 data running inside a container.


## Milestone 0: Set up workshop 

```
git clone git@github.ncsu.edu:software-engineering-for-robotics/workshop-7-foxglove.git
```


* [Open the form in a new tab](https://forms.gle/2JvniEZGgRBGEkoX8)

* Build the workshop.

* Start the container.



## Milestone 1: Find the URDF for the Dummy Robot

In this milestone, the goal is to find the `robot_description` information in the running `dummy robot` and visualize this URDF using an online URDF editor ([https://mymodelrobot.appspot.com](https://mymodelrobot.appspot.com))


Start up the `dummy_robot`:
```
ros2 REMEMBER_THIS_COMMAND?  dummy_robot_bringup dummy_robot_bringup_launch.py
```

This should start the `dummy robot` system.

As mentioned in class, the `URDF` representation of a robot is often created dynamically at runtime. The running `dummy robot` provides 2 ways to get the URDF:

1. URDF is available in **a parameter.**
2. URDF is available as **a ROS message**.

From a new terminal window connected to the running docker container, modify the `ros2 param ...` command to show the URDF of the dummy robot.  If necessary, look back at [the workshop on parameters](https://github.ncsu.edu/software-engineering-for-robotics/workshop-4-parameters-and-bagging) to refresh your memory or use `ros2 param -h` for detailed command line help.

FORM: What `ros2` command did you use to get the URDF from a parameter?

Now, let's get that same dummy robot URDF information from a ROS topic.  Using the `ros2 topic ...` command as a starting point, find a command that will show the `robot_description` **including the entire URDF** (not truncated).  You might need to look up how to show the `data` field of the `robot_description` topic.

FORM: What `ros2` command did you use to get the full URDF from a topic?

Now copy the URDF of the dummy robot that you accessed either from a param or a topic and put the URDF into this online visualizer, `MyModelRobot` [https://mymodelrobot.appspot.com](https://mymodelrobot.appspot.com).

Once you've copied the URDF into the online editor, click the `load robot URDF` button show the robot. 

Now use the online editor to significantly modify both the color and lengths of all 3 of the robot's links.

FORM: Take a screenshot of the modified robot in the visualization window of `MyModelRobot`.


## Milestone 2: Connecting Foxglove to the dummy robot.

* Download and install [Foxglove](https://foxglove.dev/download).

Stop the docker container that is running the dummy robot, and relaunch the docker container but connect port `8765` in the container to port `8765` on your host machine by adding the `-p 8765:8765` command option to your `docker run ...` command.

Open the `foxglove` program and select the `open connection` option.  Choose the option `foxglove websocket` using the `open` command.

Now that foxglove is set up to talk to our docker container, we need to start 2 things:

1. Launch the dummy robot inside your container.
2. Launch the `foxglove_bridge` from within your container (you might need to connect another terminal to your running container), using the command: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

Hopefully, you should now be seeing a visualization of the moving `dummy robot` within foxglove.  If you do not, you might need to add a `3D` panel inside foxglove.


FORM: Take a brief (5-10 seconds) screen capture **video** of the foxglove visualization showing the `dummy robot` arm swinging back and forth for the form


# Stretch Goal:
Quit and restart `foxglove` and select the `Explore sample data` option from the splash screen.   Play the sample data of a driverless car in Singapore. and try to find out how many humans are being detected (the "human detector" data is available in `markers/annotations`).



## END

Remember to complete the google form.

## Evaluation

This workshop is worth 5 points.

You'll be graded on the following rubric:

| ITEM | POINTS |
|--|--|
|Screen shots | 3 |
|Questions   | 2 |
