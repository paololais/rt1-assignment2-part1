.. assignment_2_2024 documentation master file, created by
   sphinx-quickstart on Fri Mar 14 17:36:15 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

assignment_2_2024 Documentation
===============================
This is the documentation of the assignment_2_2024 package, developed for the 
Research Track course of Robotics Engineering Master's degree at University of Genoa.

This package implements the functionalities to control a robot in a simulation 
environment. It includes two main nodes: an action client (`user_input.py`) and 
a service node (`get_last_target_service.py`). 

**Features:**
   - Set a target position for the robot.
   - Cancel the target.
   - Retrieve the last set target coordinates.
   - Publish the robot's position and velocity as a custom message.

For installation and usage, check the README file:  
https://github.com/paololais/rt1-assignment2-part1

Indices
*******
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

User Input Service
==================
.. automodule:: user_input
   :members:

Last Target Service
===================
.. automodule:: get_last_target_service
   :members:
