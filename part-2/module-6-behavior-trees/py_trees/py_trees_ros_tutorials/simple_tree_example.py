#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################


##############################################################################
# Imports
##############################################################################

import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys

from . import mock




##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Here is where we layout the structure of our tree, creating the root and attaching children to it.
    Returns:
        the root of the tree
    """
    root = py_trees.composites.Sequence(
        name="Workshop Simple Sequence"
    )

    my_failure = py_trees.behaviours.Failure(name="Dummy Failure Node")
    flipper = py_trees.behaviours.Periodic(name="Dummy 'Flipping' Node", n=1)

    root.add_child(my_failure)
    root.add_child(flipper)

    return root


def tutorial_main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=2000.0) # ms = Miliseconds

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()
