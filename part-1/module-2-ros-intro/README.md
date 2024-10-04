# ROS2 pub/sub intro + ROS2 Command-Line-Interface

Goals of this module:

* Run multiple ROS nodes and have them communicate (publish and subscribe) over a topic.
* Become familiar with some ros2 command-line tools useful for debugging and logging.

## Setup

* Launch `Docker` on your local machine.

## Milestone 1: Download and start a new docker container.

Open the terminal and navigate to this folder. For example, if your terminal is in the folder from the last module, this would necessitate the following.

```bash
cd ../module-2-ros-intro
```

This command reads the `Dockerfile` instructions and downloads code to build the environment (a docker "image") that we're going to use to run ros2. Build the ROS image:

```bash
docker build . -t ros-module-2
```

Now start this docker image, making a running docker "container":

```bash
docker run -it ros-module-2 
```

You should see:

```bash
----------------------
WELCOME TO MODULE 2!
----------------------
root@8a2a95d76a0c:~#
```

Note the hex code after `root@` and before `:~#`  (in the example above, `8a2a95d76a0c`, __your docker container hex id will be different__).  We'll use this code later on and refer to it as `YOUR_DOCKER_CONTAINER_HEX_ID`.

## Milestone 2: Simple Publisher and Subscriber.

Let's get two windows going so we can see 1 publisher and 1 subscriber at the same time. 

Open a second terminal window and connect to your running docker container through another interactive terminal and replace `YOUR_DOCKER_CONTAINER_HEX_ID` with the hex id of your running container from the first terminal:

```bash
docker exec -it YOUR_DOCKER_CONTAINER_HEX_ID bash
```

You should again see:

```bash
----------------------
WELCOME TO MODULE 2!
----------------------
root@8a2a95d76a0c:~#
```

Now, run the included ros2 publishing node:

```
python ./src/hello_world/pub.py 
```
In a few seconds, you should see something like:

```
[INFO] [1597781864.860547100] [simple_publisher]: Publishing: "Hello World: 0"
[INFO] [1597781865.342818900] [simple_publisher]: Publishing: "Hello World: 1"
[INFO] [1597781865.840738400] [simple_publisher]: Publishing: "Hello World: 2"
...
```
Cancel the running node by pressing `ctrl-c`

Let's look at that output in detail.

| ITEM | DESCRIPTION | 
|--|--|
| `[INFO]`| This is the "debug level."  `INFO` is the 2nd severity level in the debug levels `DEBUG`, `INFO`, `WARN`, `ERROR` or `FATAL.`  You can choose to supress messages less than a particual debug level, which can be useful if you only care about, for example, fatal errors.|
|`[1597781864.860547100]` |This is a timestamp in "epoch" or "unix" time format.  The number to the left of the "." is the number of seconds since Midnight, Dec. 31, 1969 ([really!](https://en.wikipedia.org/wiki/Unix_time)), and number to the right is in nanoseconds.|
|`[simple_publisher]` | The name of the node that is publishing this message.  This is useful because if you have messages from many nodes at once, it can be hard to know where the message came from. | 
|`Publishing: "Hello World: 0"`| The debug message itself. |



Lets modify the publisher code in `./src/hello_world/pub.py`.  You have several options for an editor, but it might be fastest to use the `nano` editor that is already build into this container.

(If you want to use some other editor, like `Sublime` or `notepad`, raise your hand and I'll help you set it up.  It involves restaring your container with the `-v ./src:/root/src` option enabled)

```bash
nano ./src/hello_world/pub.py 
```

Change line 17, adding your unity id after `Hello world`, so its now `Hello world, my id is UNITY_ID`.

After adding your unityid, quit `nano` using `ctrl-x`, confirm your change with a `Y` for yes, and then hit `Enter` to save it to the same file name.

Now restart your ros2 node:

```bash
python ./src/hello_world/pub.py 
```

And you should see something like:

```
...
[DEBUG] [1597781864.860547100] [simple_publisher]: UNITY_ID: "Hello World, my id is UNITY_ID: 32"
...
```

Switch to your first terminal window that is also running docker and run the subscriber node:

```bash
python ./src/hello_world/sub.py 
```

And you should see something like:

```
[INFO] [1597783642.273824000] [minimal_subscriber]: I heard: "Hello World, my id is djyanke: 27"
[INFO] [1597783642.778840400] [minimal_subscriber]: I heard: "Hello World, my id is djyanke: 28"
```

In general, ros2 nodes can __both publish and subscribe__, and usually that is the case.  A ros2 node takes information in, performs some computations with it, then republishes that information.  For example, a node might take in an image and output the locations of bounding boxes that contain cats.

## Milestone 3: Explore ROS2 command line tools.

Leave those 2 windows open, and open a third terminal and connect to your running docker container.  

```bash
docker exec -it YOUR_DOCKER_CONTAINER_HEX_ID bash
```

You should again see:

```bash
----------------------
WELCOME TO MODULE 2!
----------------------
root@8a2a95d76a0c:~#
```

You should currently have 2 ros nodes running.  ROS2 provides a command to see all the running nodes:

```bash
ros2 node list
```

Which should show:

```
/minimal_subscriber
/simple_publisher
```

Let's see all the topic that are currently being published:

```bash
ros2 topic list
```

which shows:

```
/hello_world_topic
/parameter_events
/rosout
```

As expected, we see `/hello_world_topic` but additionally, there are two topics we didn't publish but that are provided by default for every running ros2 system:  `/parameter_events`, which will show ros2 parameters when they change, and `rosout`, which is a "sink" in the ROS graph.  These topics have a leading "/" because we can nest topics, i.e. "/camera1/image" and "/camara1/parameters" both relate to data from "camera1". 

Let's look at the content of these topics using the `echo` command, which essentially creates a temporary ROS2 code that subscribes a topic and writes the output to the screen.

```bash
ros2 topic echo hello_world_topic
```

And you should see a stream of messages:

```bash
data: 'Hello World, my id is djyanke: 614'
```

Use `ctrl-c` to stop the `echo` command.

Now let's check how fast these messages are being published:

```bash
ros2 topic hz hello_world_topic
```

and you should see:

```
average rate: 1.999
	min: 0.498s max: 0.503s std dev: 0.00186s window: 3
average rate: 2.000
	min: 0.497s max: 0.503s std dev: 0.00180s window: 6
```

Stop the `hz` command with `ctrl-c`.

This is showing the approximate rate at which message are being received, 2 per second (as expected because our pub node is set to publish at 2 hertz.

Stop the publishing node and edit `./src/hello_world/pub.py` to make `timer_period` smaller to speed up the publisher (line 11), to a faster value, `0.01`.

Restart the publisher, then restart the `hz` command to measure the rate in your 3rd terminal window with

```bash
ros2 topic hz hello_world_topic
```

Stop the publisher and `hz` commands, and change the `timer_period` to `0.001` (line 11).

Restart the publisher and `hz` commands, then measure the rate in your 3rd terminal window with

```bash
ros2 topic hz hello_world_topic
```

If your computer was able to maintain the appropriate frequency in the last step and you want to see how high the frequency can go, then try reducing the period further. Depending on your machine's hardware and configuration, there will be wide variation in this maximum frequency. Compare with your neighbors to see who has the fastest configuration. Some computers may max out at a few hundred Hz, others will be able to reach a few kHz!

## That's it :)

### Notes

`docker exec...` is a very useful command. What it is doing is specifying a running container and passing a command to execute inside it. When you pass `bash` in this way, the container opens a bash terminal and the `-it` specifies to have the bash session be interactive, just like in the run command. Since `docker run` starts a new container that is isolated from the first, being able to connect to a running container with a new terminal is an essential function to be able to run multiple processes in the same environment. Whenever subsequent directions refer to opening a new terminal with the same container, this is the command that should be used. When opening a new container, then `run` should be used.
