
# Module 6

The goals of this module are to learn the basics of Behavior Trees using the [`py_trees`](https://github.com/splintered-reality/py_trees) library. This module is a bit of a detour from ROS and is focused on a tool in the field of Robotics Control Theory and the broader computer science concept of Behavior Trees.

NOTE:  The authors of the `py-trees` code use the British spelling "behaviour" instead of "behavior".   I've tried to use it correctly in context below, but watch out so this little difference does not trip you up.

## Milestone 0: Set up module.

Build the workshop, where `embedml/ros-humble-workshop:module-6` is the IMAGE_NAME:

```bash
HINT:  docker (DO YOU RECALL THIS COMMAND?) . (DO YOU RECALL THIS COMMAND?) IMAGE_NAME
```

(Note: this module should build properly on Powershell, but if you encounter issues, then run `docker rmi -f embedml/ros-humble-workshop:module-6` and run `docker pull embedml/ros-humble-workshop:module-6` before continuing.)

Run the docker image and link your local directory to a folder inside the docker container:

WINDOWS POWERSHELL:

```ps
HINT:  docker run -v ${PWD}/py_trees:/root/workspace_folder/src/py_trees -it IMAGE_NAME
```

UBUNTU or OSX

```bash
HINT:  docker run -v $(pwd)/py_trees:/root/workspace_folder/src/py_trees -it IMAGE_NAME
```

Verify that your container is linked to your local folder (`touch` is a unix command that creates an empty file.) :

```bash
touch /root/workspace_folder/src/py_trees/connected.txt
```

Notice that we have linked your host machine folder `py_trees` to a folder inside the `workspace_folder/src/py_trees` directory.   This means you can edit files inside `py_trees` on your host computer using any editor you like and changes will be reflected inside the docker container.  Also, modifications inside the docker container maked within the `workspace_folder/src/py_trees` directory will be reflected in the `py_trees` directory on your host machine.

Now in the Docker container, build the files in the workspace:

```bash
cd workspace_folder
colcon (DO YOU RECALL THIS COMMAND?)
```

Remember that `colcon` recursively traverses the contents of `workspace_folder/src` looking for folders with a `package.xml` file, which contains instructions for how to compile and link the the contents of that package.  This should only take a few seconds.

From within the `workspace_folder`, "activate the overlay," which means running a small script that tells `ros2` about the code contained within the `workspace_folder`:

```bash
source ./install/setup.bash
```

Or, do both of the previous commands in one line (for convenience).  Activating the overlay multiple times will not break anything.  Note that in the second command, `.` is a shortcut for the `source` command:

```bash
colcon build; . ./install/setup.bash
```

## Milestone 1: Learning about the `py_trees` library, the `ros2 run` process and "entry points".
 
From the `workspace_folder`, try running a simple behavior tree (this will only work if you have built with `colcon` and activated the overlay):

```bash
ros2 run py_trees_ros_tutorials simple-tree-example
```

You should see something like:

```
[-] Workshop Simple Sequence [x]
    --> Dummy Failure Node [x] -- failure
    --> Dummy 'Flipping' Node
```

This is an [ASCII representation](https://en.wikipedia.org/wiki/ASCII) of a `py-trees` behavior tree.
This tree very simple, with a "Sequence" node at the root and two children nodes.
The children nodes are also leaf nodes.  Recall that a "Sequence" is like an "AND" statement in logic, which is true only if all its component parts are true.

Let's find the corresponding code.  Since we just ran `ros2 run py_trees_ros_tutorials simple-tree-example`, we know we can find the corresponding code by looking in `py_trees/setup.py`.

Using an editor of your choice on your host machine, navigate to the `setup.py` file located in `workshop-5-behavior-trees/py_trees/setup.py`.

Within `setup.py`, search for `simple-tree-example`, which we used in the command `ros2 run py_trees_ros_tutorials simple-tree-example`.
On the line containting `simple-tree-example`, you'll see a long string on the right hand side of the `=` statment.  This is of the form `PACKAGE.FILENAME:PYTHON_ENTRY_POINT_METHOD`.   Take careful note of each of `PACKAGE`, `FILENAME`, and `PYTHON_ENTRY_POINT_METHOD`.  

The `PACKAGE` part corresponds to a python package *which is also a folder* within the `py_trees` folder.
Navigate to the `PACKAGE` folder and find the file identified by `FILENAME.py`  (Note that the `FILENAME` part from the `setup.py` will not have the suffix `.py`, but the file will have `.py`).  

Open this `FILENAME.py` (in the `PACKAGE` folder) using your favorite editor.  Now search for the `PYTHON_ENTRY_POINT_METHOD` you found in the `setup.py` command.  This is the "entry point" in the python file that starts when we run `ros2 run py_trees_ros_tutorials simple-tree-example`.

## Milestone 2: Modify the behavior tree

Using an editor, open the `FILENAME.py`.   Starting from the entrypoint, notice this code (lines 58-62):

```python
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
```

The code `root = tutorial_create_root()` creates the root `behavior` of our Behavior Tree.

Navigate to the `tutorial_create_root()` function in the code and change the `root` tree, currently a `Sequence` node to be of type `Selector` (also known as a "Fallback" or "OR" node).

Also, change the name of the root node on line 41:

```python
name="UNITY_ID's Selector Node"
```

(replace UNITY_ID with your ID, of course).

Save the file.  Since we're using `ros2 run`, we'll need to rebuild the package with `colcon build`.  Navigate in your docker container to `/root/workspace_folder` and run the command:

```bash
colcon build; . ./install/setup.bash
```

This rebuilds the package with your changes and applies the overal.

Now run the node again:

```bash
ros2 run py_trees_ros_tutorials simple-tree-example
```

You should see output of the state of the behavior tree as before, but slightly different:

```
[o] djyanke's Selector [*]
    --> Dummy Failure Node [x] -- failure
    --> Dummy 'Flipping' Node [*] -- constant
```

Now the `Flipping` node in the tree shows a changing value as it "flips" through the 3 possible Behavior Tree states.

What are the 3 possible return values in response to a "tick" of the behavior tree? Note: The `Flipping` node displayes "constant" to indicate if a node's state hasn't changed since the last tick. "constant" is not one of the 3 states.

Change directories to the folder linked to your host machine:

```bash
cd /root/workspace_folder/src/py_trees
```

Now generate figure of your behavior tree with the changes you made above (where `FILENAME` is the filename from above)

```bash
py-trees-render -b py_trees_ros_tutorials.FILENAME.tutorial_create_root
```

Related but not critical Note: FYI you can change the "tick" rate by changing the line:

```python
    tree.tick_tock(period_ms=2000.0) # ms = Miliseconds
```

## That's it for behavior trees
