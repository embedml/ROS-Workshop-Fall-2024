#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ADAPTED FROM: https://py-trees.readthedocs.io/en/release-2.1.x/behaviours.html 

import py_trees
import py_trees
import random
from sensor_msgs.msg import BatteryState
from . import battery_listener 



class Foo(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        # CREATE ROS NODE
        self.my_battery_listener  = battery_listener.BatteryListener()
        self.current_battery_state = BatteryState()
        self.current_battery_state.percentage = 0.42


    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [Foo::update()]" % self.name)
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])
        if not ready_to_make_a_decision:
            return py_trees.common.Status.RUNNING
        elif decision:
            battery_percentage = self.my_battery_listener.current_battery_state.percentage
            self.feedback_message = ('%0.2f' % battery_percentage)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Uh oh"
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
