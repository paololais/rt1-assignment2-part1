.. assignment_2_2024 documentation master file, created by
   sphinx-quickstart on Fri Mar 14 17:36:15 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

assignment_2_2024 documentation
===============================
This package implements the functionality for controlling a robot in a simulation environment. It includes two main nodes: an action client (user_input.py) and a service node (get_last_target_service.py). 

The package allows the user to set a target position for the robot, cancel the target, and retrieve the last set target coordinates. It also publishes the robot's position and velocity as a custom message.


.. toctree::
   :maxdepth: 2
   :caption: Contents:

Indices
*******
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

ROS Package Documentation
**************************

This is the documentation for the `rt1-assignment2-part1` package.

User Input Service
==================
.. automodule:: user_input
   :members:
   :no-index:

Last Target Service
===================
.. automodule:: get_last_target_service
   :members:
   :no-index:
