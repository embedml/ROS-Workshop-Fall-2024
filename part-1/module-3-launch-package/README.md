# ROS2 Packages & Launch Files

Goals of this module:

* Learn about ros2's packaging system.
* Starting multiple nodes using launch files.

## Setup

* Launch `Docker` on your local machine.

## Milestone 1: Download and start a new docker container.

Open the terminal and navigate to the folder, build it, and run it.

```bash
#Skip this block on Powershell
cd module-3-launch-package
docker build . -t embedml/ros-humble-workshop:module-3
```

Now run the docker container:

```bash
docker run -it embedml/ros-humble-workshop:module-3
```

(NOTE:  Running this container this way will means changes inside the container will __not__ persist after the container is stopped.  If you want the changes and edits to persist, or if you want to use an editor on your "host" machine, you can instead run this command with the `-v` option enabled, like `-v LOCAL_FOLDER:CONTAINER_FOLDER`.  On Windows Powershell, this would be mean adding `-v ${PWD}:/root` to the `docker run -it workshop-3` command, like `docker -v ${PWD}:/root -it workshop-3`.    On linux or OSX, this means adding `-v $(pwd):/root` to the command)

You should see:

```bash
----------------------
WELCOME TO MODULE 3!
----------------------
root@8a2a95d76a0c:~#
```

(Your hex ID will be different, of course).


## Milestone 2: Packages

Package are collections of software that works together.  The ROS2 package you see today is rather simple, whereas usually packages are much more complicated, but hopefully this section will show your a little about how packages are organized and what they do.

```bash
cd workspace_folder
ls
```

To better understand the files in this directory, we'll use the unix `tree` command, but first we need to get it. To get software on Ubuntu-flavored linux systems, we use the `apt` command.  Firstly, we connect to the internet and __update__ the list of what software is available for our system and where it is located:

```bash
apt update
```

You should see a lot of things scroll.  Now that we have an updated list of available software we can install the `tree` command:

```bash
apt install tree
```

Now try the `tree` command from the current `workspace_folder`:

```bash
tree
```

The `workspace_folder` contains a ROS2 "package".  For more information, [see this link](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).

Now we'll "build" this package.  Ros2 has a special command to recursively find code in subdirectories and compile everything inside it.  This command is `colcon`, and it is specific to the ROS ecosystem.  `colcon` is a tool that support "[Build Automation (wiki)](https://en.wikipedia.org/wiki/Build_automation)", basically it helps create and manage the execuable code.  We run it like this:

```bash
colcon build
```

You should see a few messages and perhaps some warnings.  Do the warnings look serious?    Now examine the contents of the subfolders again:

```bash
tree
```

So much more!   `colcon` has created a lot of files.    So let's tell `tree` to not go as deep into the directory tree.   Using this command to access the help for tree:

```bash
tree --help
```

Look for the parameter in `tree` that limits the depth, "Descend only level directories deep."

Once you have found this parameter, run `tree` again and only go 2 levels deep.

Notice that there are now 4 directories in `workspace_folder` (tree shows more detail that just these 4):

```
build
install
log
src
```

The `install` directory that `colcon` built contains some special files that help ROS2 know what can be launched and run.  One of these special files creates something called an "overlay", which is a set of environment variables inside your terminal instance that help ROS2 know about the files inside this package.

You can see all the "environment variables" in your terminal (just like local variables in a program) with the command

```bash
env
```

Each line contains is in the form `VARIABLE_NAME=value`.   We can search these variables using `grep`

```bash
env | grep ROS_DISTRO
```

and you should see the version of ROS2 that we're running:

```bash
ROS_DISTRO=____ 
```

Now let's run the special file that creates the "overlay"  using the `source` command (which just executes the contents of the file):

```bash
source ./install/local_setup.bash
```

Now we have a new "overlay" of variables, which is an overly fancy ROS2 way of saying we set some variables.  Specifically, we have a new variable in our "environment variables:"

```bash
env | grep COLCON
```

and your should see:

```bash
COLCON_PREFIX_PATH=/root/workspace_folder/install
```

This is one of the places the `ros2` command line command will look for executable files.

Now that we have built our package and activated it using the `source` command, we're ready to run some ROS2 code.

This code runs the node `sub` from the package `rclpy_launch_demo` and uses `&` to put the process in the background.  Running a node in the background means that its still running even though you can still interact with the terminal (!)

```bash
ros2 run rclpy_launch_demo sub &
```

Using a ros2 command from workshop 1 that lists the current topics, list the ros topics that are active.

To see the current processes in the background, type:

```bash
jobs
```

This should show the current children processes running in this terminal.   Now return the process to the foreground (`fg`):

```bash
fg
```

and stop the node using `ctrl-c`.

## Milestone 3: Launch Files

Remember in previous workshops where we started ROS nodes directly with something like `python pub.py`?   We also started one ROS node in each terminal window.  This works OK for a few nodes, but when you have 50 nodes working together it does not scale.  Therefore we need a way to launch multiple ROS nodes at once.  This is where "ROS launch files" come in.

Change directories in the container to your $HOME directory:

```bash
cd ~
```

(aside: in unix, the `~` symbol is a shortcut for the `$HOME` environment variable.  You can check using `env | grep HOME` or `echo $HOME`)

Use an editor to open the launch file (*alternately:* use Visual Studio Code)

```bash
nano test.launch.py
```

There are 3 lines you'll need to uncomment to make the file runnable.  The comment symbol is `#`.  Remove the 3 comments.  Note that one of the of the lines to uncomment pertains to `LaunchDescription`.  This `LaunchDescription` is a ROS2 launch object that specifies which nodes to launch.  Save the changes to the file.

NOTE: copy the HEX_ID of your container so you'll have it ready.

Here's the ROS2 launch command:

```bash
ros2 launch test.launch.py
```

you should see something like:

```
root@bfbc906896a4:~# ros2 launch test.launch.py 
[INFO] [launch]: All log files can be found below /root/.ros/log/2020-08-26-01-05-21-442486-bfbc906896a4-373
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [pub-1]: process started with pid [375]
[INFO] [sub-2]: process started with pid [377]
[sub-2] [INFO] [1598403922.839647200] [my_sub_name]: x:0.00, y:0.00, z:0.00
[sub-2] [INFO] [1598403923.817893000] [my_sub_name]: x:0.00, y:0.00, z:0.00
```

Open a new terminal window and attach to this container:

```bash
docker exec -it YOUR_DOCKER_CONTAINER_HEX_ID bash
```

Since we started a new terminal, we started over with the terminal's environment variables.  We'll need to "activate the overlay again."  Note that `.` is a shortcut for the `source` command, and now we're running it from another directory (`/root` instead of `/root/workspace_folder`)

```bash
. ./workspace_folder/THAT_FILE_FROM_MILESTONE_2_THAT_CREATES_AN_OVERLAY
```

Now that we've told ROS2 about where to find the executable code in our package, we can run the keyboard listener node `key` from this package:

```bash
ros2 run rclpy_launch_demo key
```

You should see:

```
command:
```

Try typing some keys and hitting enter after each one.  What do you see?  Now stop this node with `ctrl-c`.

Using the `tree` command, locate the file `key.py`. Navigate to this directory and view the source of the `key.py` code and try reading it.  What key command will reveal `THAT'S RIGHT`?

Once you've figured out what this node is doing, re-run the `key` code:

```bash
ros2 run rclpy_launch_demo key
```

And you'll see:

```
command:
```

Try some of the allowed keys. Notice that the Waypoint in the other window is changing.  Using the `key` node `command:` interface, change the Waypoint until you see:

```
[sub-2] [INFO] [1598404938.208070200] [my_sub_name]: x:-4.00, y:2.00, z:0.00
```

Stop the nodes in both windows by using `ctrl-c` in each.

## Symlink-Install

The build tool `colcon` has the ability to configure the executables in Python packages as symbolic links instead of copying files. This means that if you update the Python source file, then the change will automatically be reflected when you use `ros2 run ...`. To compare this to default `colcon` behavior, do the following two exercises in each of the container windows from the end of the workshop.

First, use `nano` to change pub.py in the timer_callback() function block so that the x value of `self.my_goal_waypoint` is always 17.32. Save your changes

```python
self.my_goal_waypoint.x = 17.32
self.publisher_.publish(self.my_goal_waypoint)
```

Build and source the overlay.

```bash
cd ~/workspace_folder
colcon build
source ./install/local_setup.bash
```

Run the pub using `ros2 run` and view the output with the command that prints topic data. The x value should now always be 17.32.

Then using `nano`, change x value from 17.32 to 13.67. Exit `nano` and save changes.

Now, run the pub node again and view the output. Did it change?

We will repeat the same process but using the symlink-install option this time. Lets set x to another random value. This time, 22.45. Save your changes to the file and build the workspace.

```python
self.my_goal_waypoint.x = 22.45
self.publisher_.publish(self.my_goal_waypoint)
```

```bash
cd ~/workspace_folder
colcon build --symlink-install
source ./install/local_setup.bash
```

Run the pub and view the topic data. The x value should again be changed to reflect the file update made prior to this execution of `colcon build`.

Make one final change to the file, setting x to be 91.26, and save your changes.

Run the pub node and check the topic data stream. Does the value of x reflect your most recent change to the file?

`--symlink-install` gives the option to maintain a link to the file instead of copying the file to the relevant install location. This is a Python-specific option because C/C++ has to be compiled to generate an updated executable.

## Extras

In addition to the hex value given for a container, you can also assign names to containers in the `docker run` command. The syntax is as follows.

```bash
docker run --name container-name -it image-name
```

Then, it is easier to use `docker exec`, such as in the following line.

```bash
docker exec -it container-name bash
```
