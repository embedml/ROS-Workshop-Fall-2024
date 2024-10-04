# Multiple Language Support

This will be a relatively quick module focused on exploring the usage of ROS2 with nodes implemented in multiple languages and with a custom interface. A key feature of ROS is that it is language agnostic and nodes function identically regardless of implementation language.

## Milestone 0: Set up module

Build and run the Docker container for the module, where `embedml/ros-humble-workshop:module-5` is the image name. (Again, if you use Powershell, just use the run command. It will automatically download a compatible Docker image from the Docker Hub and run the container.)

## Milestone 1: Exploring Custom Message

The `ros_entrypoint.sh` script should have built the workspace when the container was set-up, however, you can double-check it built properly.

```bash
cd /root/workspace_folder
colcon build
```

Source the overlay

```bash
source /root/workspace_folder/install/setup.bash
```

__Optional__: automatically source the overlay in every new terminal you connect to the container.

```bash
echo 'source /root/workspace_folder/install/setup.bash' >> /root/.bashrc
```

Look at the packages in the source folder of the workspace, using the tree command from a prior module:

```bash
tree -L 1 ./src
```

You should see the following.

```
.
├── example_pkg_cpp
├── example_pkg_py
└── example_test_interfaces
```

These are three example packages demonstrating a simple publisher and subscriber, one each in Python and C++. There is also an example of a package defining a custom message interface in `example_test_interfaces`. Look at the message definition there (HINT: Find the `.msg` file in the folder).

```bash
more path/to/msg/file/ExampleTest.msg
```

What do you see? Notice how the message definition is a collection of named message fields. These fields are standard message types provided by ROS. Look for the two types in the custom message [here](https://github.com/ros2/common_interfaces/tree/rolling/std_msgs/msg).

Open a second terminal and connect to the docker container. Source the workspace overlay if you did not set it up to do so automatically.

Now, run the `testpub` executable from the `example_pkg_py` package in one terminal and use the appropriate `ros2` to view the output on `/test_topic` in the other terminal. Compare this output to the message definition file. Notice how the field names are kept and displayed with the message. Ctrl-C both terminals to stop the current processes but stay connected to the container.

## Milestone 2: Language Agnostic Communication between Nodes

This packages in this workspace have two different build systems. One is called ament_python, the other is ament_cmake. As you might infer, the former is for Python packages, and the latter is for C/C++ packages. Technically, it is possible to have both in the same package, however, I (Daniel) personally find it simpler to split my packages by language.

### Python to C++ Communication

We are going to start a Python publisher and a C++ subscriber and observe that the data is handled in an expected manner.

In one terminal, run:

```bash
ros2 run example_pkg_py testpub
```

In the other terminal, run:

```bash
ros2 run example_pkg_cpp testsub
```

What do you notice? Is the subscriber node properly receiving what the publisher is sending?

From here, start and stop nodes in both Python and C++ to see how they behave and pay attention to whether the behavior between the respective implementations is identical.

Here are the nodes available in each of the packages. Notice how they can have identical names in different packages because each namespace is package-specific.

```
ros2 run ...
├── example_pkg_cpp
│   ├── testpub
│   └── testsub
└── example_pkg_py 
    ├── testpub
    └── testsub
```

Once you have experimented with various combinations and compared the source code in the package folders (`src` in the C++ package and the folder with the same name as the package for Python), ctrl-C any processes running in the two terminal windows but leave the container connected.

## Multiple Publishers on a Topic

Having multiple publishers competing on a topic is not necessarily a good development practice in most cases, but it is an interesting exercise to see how ROS handles it.

In `example_pkg_py`, there is a launch file that runs all 4 nodes listed above simultaneously. Launch it.

```bash
ros2 launch example_pkg_py all_nodes.yaml
```

You can ctrl-C it after a few seconds because it will generate a chaotic mess of log messages from all the different nodes. Look through the mess and follow the progression of data between the nodes. Notice how ROS handles multiple publishers just fine.

As an additional verification if you are curious, you can relaunch the nodes, and then in the other terminal, run the ROS command to print out the topic data. You should see messages coming from both nodes with no issues.

Nevertheless, refer back to the message definition. The way the subscriber nodes are able to differentiate between publisher nodes is because the message type explicitly includes a field to specify that.

It is usually better to make a new topic than to have multiple publishers on one topic because then the subscribers automatically know from exactly which node the information originated. Additionally, if you are dealing with movement commands for a robot, then you DO NOT want to have multiple conflicting commands reaching your actuators and potentially causing physical damage, which could happen if you have multiple publishers to a movement_goal or similar topic.
