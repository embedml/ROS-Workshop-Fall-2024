
# Workshop 4

The goals of this workshop are to learn about ROS parameter and a kind of Log File called a 'ROS Bag.'





## Milestone 0: Set up workshop.

Open the [workshop form](https://forms.gle/xtujxhZbrbPXUb6t5) in another tab so it's ready.


Download the workshop.
```
git clone git@github.ncsu.edu:software-engineering-for-robotics/workshop-4-parameters-and-bagging.git
```

Build and run the workshop, where `workshop-4` is the IMAGE_NAME:
```
HINT:  docker (DO YOU RECALL THIS COMMAND?) . -t IMAGE_NAME
HINT:  docker run -it IMAGE_NAME
```


## Milestone 1: Learning about message types during runtime.

Activate the "overlay"
```
source /root/workspace_folder/install/setup.bash
```


From the running container, launch the `pub` from `rclpy_launch_demo` package:
```
hint:   ros2 run PACKAGE_NAME NODE_NAME   
```

From a separate terminal, connect to the docker container.  Then list the available topics:

```
ros2 topic list
```

You should see at least the following topic name (called TOPIC_NAME) in the next command:
```
/goal_waypoint
```

To get more information about this topic:
```
ros2 topic info TOPIC_NAME
```

Take note of the `Type` of the message: The type of the message is in the form "MESSAGE_LIBRARY/MSG/MESSAGE_TYPE."
Record the message type **in the form**.

Now let's get information about **nodes**. This shows all of the current running nodes:

```
ros2 node list
```

Make a note of the `node` name that is returned by this command including the leading '/', called NODE_NAME in following command. 

```
ros2 node info NODE_NAME
```

Answer the corresponding Milestone #1 question in the activity form.


## Milestone 2: ROS Parameters


Run the `param` node from the `rclpy_launch_demo` package. 

```
ros2 run PACKAGE_NAME NODE_NAME
```

In a separate terminal window, view the available "parameters:"

```
ros2 param list
```

You should see:
```
/param_publisher:
  my_float
  start_type_description_service
  use_sim_time
```

This is in the form:
```
/NODE_NAME:
  PARAM_1
  PARAM_2
```

In ROS2, parameters are stored in each node (unlike ROS1, which stores them centrally in a parameter server).   Parameters are like external interfaces to object properties with `getters` and `setters`.    Let's view a parameter:

```
ros2 param get NODE_NAME my_float
```
You should see:
```
Double value is: 42.0
```

On the `param` node, this value is being published as part of the `/param_waypoint` topic, the TOPIC_NAME.   Echo the `/param_waypoint` to the terminal window

```
ros2 topic echo TOPIC_NAME
```

Now open another terminal window and connect to the docker container.  In this new window, set the `my_float` parameter (PARAMETER) of the `param_publisher` (NODE_NAME) to 3.14, the NEW_VALUE

 ```
ros2 param set NODE_NAME PARAMETER NEW_VALUE
```
 
Now go back to your 2nd terminal window and confirm that the `x` part of the `point` mess is `3.14`.


__Take a screen shot (#1)__ of your two terminal windows and upload it to the google form you opened at the beginning.




## Milestone 3: ROS Bags, Log Files, and 'Bagging'.

A "[Log File](https://en.wikipedia.org/wiki/Log_file)" is a critical tool for auditing, tracking, troublshooting, and document what happened during a program's execution.
**In ROS2, a log file is called a "ROS Bag"**.

"Bagging" means creating a log file for a running ROS system.    

Uses
---

* Recording sensor data (creating a dataset)
* Creating a record of actuation commands.
* Troubleshooting timing issues

Level of Introspection, aka Logging granularity
---

Logging (the more general form of 'bagging') can happen a many levels.
For example at a **low level,** a computer program might log every time a particular [interrupt vector](https://en.wikipedia.org/wiki/Interrupt_vector_table) is invoked.
At a higher level, your computer tracks every time a program is started or stopped.
ROS Bagging is somewhere in the middle, and you can control the level of granularity.


From your 3rd Docker terminal, try running the following:

```
ros2 bag record -a
```
The `-a` part tels ROS2 you want to record all published topics.

After a few seconds, cancel the bagging using `ctrl-c`.  Now list the contents of the directory.  You should see a new folder, something like:
```
rosbag2_2023_02_08-03_15_23
```

`cd` into this new folder and inspect the contents.

Answer this question in the form:  What files do you see?  What kinds of files are these?

Now that you've created a bag, you can play the bag back and `ros2 bag play` will "republish" the message contained in the bag:

```
ros2 bag play NAME_OF_BAG
```

Try playing the bag you just recorded. As you play the bag, open another window and look at the topics that are being published using `ros2 topic list`.  Try "echoing" a topic that is being played back, using the command `ros2 topic echo TOPIC_NAME` (where `TOPIC_NAME` is the name of a topic from your bag).



## END

Remember to complete the [google form](https://forms.gle/xtujxhZbrbPXUb6t5).


## Evaluation

This workshop is worth 4 points.

You'll be graded on the following rubric:

| ITEM | POINTS |
|--|--|
|Screen shot | 1 |
|Questions   | 3 |
