#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Launch the mock robot.
"""
##############################################################################
# Imports
##############################################################################

import launch
import launch_ros.actions
import py_trees_ros_tutorials.utilities as utilities

##############################################################################
# Helpers
##############################################################################


def generate_launch_description():
    """Launch the mock robot."""

    launch_description = launch.LaunchDescription()

    ##########################################################################
    # Mock Robot Nodes
    ##########################################################################
    for node_name in ['battery', 'dashboard', 'docking_controller',
                      'led_strip', 'move_base', 'rotation_controller',
                      'safety_sensors']:
        node_executable = "mock-{}".format(node_name.replace('_', '-'))
        launch_description.add_action(
            launch_ros.actions.Node(
                package='py_trees_ros_tutorials',
                node_name=node_name,
                node_executable=node_executable,
                output='screen'
            )
        )
    return launch_description

##############################################################################
# Main
##############################################################################


def main():
    """A rosrunnable launch."""
    launch_descriptions = []
    launch_descriptions.append(generate_launch_description())
    launch_service = utilities.generate_ros_launch_service(
        launch_descriptions=launch_descriptions,
        debug=False
    )
    return launch_service.run()