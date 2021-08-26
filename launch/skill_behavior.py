#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_behaviors/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Dynamic Loader for BT
"""
##############################################################################
# Imports
##############################################################################

import py_trees_ros_behaviors.skill_engine as skill_engine

##############################################################################
# Launch Service
##############################################################################


def generate_launch_description():
    """
    Launch description for the tutorial.
    """
    return skill_engine.generate_launch_description()
