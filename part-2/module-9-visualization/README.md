# Module 9 - Visualization

The goals of this module are to learn how learn how to find and modify an existing URDF file and to use a visualization tool, Foxglove, to view runtime changes to ROS2 data running inside a container.

## Milestone 0: Set up module

* Build the module.

* Start the container.

## Milestone 1: Find the URDF for the Dummy Robot

In this milestone, the goal is to find the `robot_description` information in the running `dummy robot` and visualize this URDF using an online URDF editor ([https://mymodelrobot.appspot.com](https://mymodelrobot.appspot.com))

Start up the `dummy_robot`:

```bash
ros2 REMEMBER_THIS_COMMAND?  dummy_robot_bringup dummy_robot_bringup.launch.py
```

This should start the `dummy robot` system.

The `URDF` representation of a robot is often created dynamically at runtime. The running `dummy robot` provides 2 ways to get the URDF:

1. URDF is available in **a parameter.**
2. URDF is available as **a ROS message**.

From a new terminal window connected to the running docker container, modify the `ros2 param ...` command to show the URDF of the dummy robot.  If necessary, look back at [the module on parameters](../../part-1/module-4-parameters-and-bagging/) to refresh your memory or use `ros2 param -h` for detailed command line help.

Now, let's get that same dummy robot URDF information from a ROS topic.  Using the `ros2 topic ...` command as a starting point, find a command that will show the `robot_description` **including the entire URDF** (not truncated).  You might need to look up how to show the `data` field of the `robot_description` topic.

FORM: What `ros2` command did you use to get the full URDF from a topic?

Now copy the URDF of the dummy robot that you accessed either from a param or a topic and put the URDF into this online visualizer, `MyModelRobot` [https://mymodelrobot.appspot.com](https://mymodelrobot.appspot.com).

Once you've copied the URDF into the online editor, click the `load robot URDF` button show the robot.

Now use the online editor to significantly modify both the color and lengths of all 3 of the robot's links.

## Milestone 2: Connecting Foxglove to the dummy robot.

You should have Foxglove installed from the [installation instructions](../../README.md).

Stop the docker container that is running the dummy robot, and relaunch the docker container but connect port `8765` in the container to port `8765` on your host machine by adding the `-p 8765:8765` command option to your `docker run ...` command.

Open the `foxglove` program outside of docker and select the `open connection` option.  Choose the option `foxglove websocket` using the `open` command.

Now that foxglove is set up to talk to our docker container, we need to start 2 things:

1. Launch the dummy robot inside your container.
2. Launch the `foxglove_bridge` from within your container (you might need to connect another terminal to your running container), using the command: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

Hopefully, you should now be seeing a visualization of the moving `dummy robot` within foxglove.  If you do not, you might need to add a `3D` panel inside foxglove.

## Extra Exploration

Quit and restart `foxglove` and select the `Explore sample data` option from the splash screen.   Play the sample data of a driverless car in Singapore. and try to find out how many humans are being detected (the "human detector" data is available in `markers/annotations`).

Other tools in Foxglove include a panel for calling ROS Services and publishing directly to topics. If you want to try exposing the Foxglove port from the Docker container in [module 7](../module-7-services/) and install the `foxglove_bridge` package (HINT: Look at the [Dockerfile](./Dockerfile) for this module), then you can call the service server. Pass the Request as JSON representing the msg structure. Examples given [here](https://answers.ros.org/question/196365/is-there-a-general-way-to-convert-ros-messages-into-json-format/). To publish directly to a topic, use the Publish panel and again specify the message format in JSON. Inside the container, you can echo the messages on the topic and see Foxglove giving you manual input to the robotic system.

# End of 2024 EML ROS Workshop!

Hopefully, this workshop gave plenty of practical experience in using Docker and the Robot Operating System. Please fill out the [attendance form](https://forms.gle/C99wPJC1sv7WEzTA6) if you haven't already.

Thanks again to Dr. Ore for the bulk of the material. I highly recommend his class ECE 591: Software for Robotics.
