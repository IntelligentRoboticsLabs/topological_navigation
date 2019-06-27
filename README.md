# Topological Navigation

[![pipeline status](https://gitlab.com/fmrico/topological_navigation/badges/develop/pipeline.svg)](https://gitlab.com/fmrico/topological_navigation/commits/develop)

## Packages

- topological_navigation
- topological_navigation_msgs

## Summary

- **Maintainer:** Francisco Martín <fmrico@gmail.com>
- **Author:** Francisco Martín <fmrico@gmail.com>
- **License:** BSD
- **ROS Distro:** Kinetic
- **Dependencies:**
  - **ROS Standard:** roscpp, rospy, tf, geometry_msgs, std_msgs, sensor_msgs, nav_msgs, visualization_msgs, move_base_msgs, actionlib, actionlib_msgs, cv_bridge, costmap_2d, image_transport, roslint
  - **Internals:**
     - bica: https://gitlab.com/fmrico/bica.git
     - bica_planning: https://gitlab.com/Intelligent-Robotics/bica_planning.git
     - ROSPLan: https://gitlab.com/Intelligent-Robotics/ROSPlan.git
     - pepper_navigation_bringup: https://gitlab.com/Intelligent-Robotics/pepper_navigation_bringup.git
     - pepper_basic_capabilities: https://gitlab.com/Intelligent-Robotics/pepper_basic_capabilities.git
     - rosweb_clientserver: https://gitlab.com/Intelligent-Robotics/rosweb_clientserver.git

## Description

This repository contains the topological navigation system. This system uses
ROS's standard navigation system ([move_base](http://wiki.ros.org/move_base?distro=kinetic)),
which it uses to move the robot, through actions that move_base contains.

The topological navigation system establishes waypoints in the environment, which
are relevant positions for the operation of the robot. To move between waypoints in the
same room, the robot can perform a simple navigation action (`move` action) or, if it is
in different rooms that are connected without doors, by `navigate` action. In addition,
it establishes doors (composed of waypoints) in which the robot performs the action `cross`.

The topological navigation system also performs a translation between waypoints and
their respective metric position, and vice versa, and the insertion in the knowledge base
of the predicates that compose the topological map.

This system is used by setting the predicate `(robot_at ?waypoint)` in the planning
as goal or requirement, calculating the planner the sequence of `move`,` navigate` or `cross`
actions necessary to satisfy it.

Video of the system working:

[![Topological Navigation](https://img.youtube.com/vi/zOPen1bFXk4/0.jpg)](https://www.youtube.com/watch?v=zOPen1bFXk4)

## PDDL Components

- **Types**: waypoint, door, room
- **Predicates:**
  - (door_connected ?d - door ?r1 ?r2 - room ?wp1 ?wp2 - waypoint)
  - (free_connected ?r1 ?r2 - room)
  - (robot_at ?wp - waypoint)
  - (robot_at_room ?room - room)
  - (waypoint_at ?wp - waypoint ?r - room)
  - (door_opened ?d - door)
- **Actions:** navigate, cross, move

## Nodes

### topological_navigation_node

The functions of the `topological_navigation_node` are:
- Perform the translation between waypoints and their respective metric position, and vice versa, through the services that it implements.
- Set the PDDL predicates that encode the topological map. When the node is launched, the topological map information is loaded using `rosparam`. This node reads them, storing the metric positions, and entering in the knowledge base the predicates that code the map.

#### Parameters

This node loads a yaml file using rosparam containing a topological map. An example of an small map is at `topological_navigation` package, in `src/test_topological_map.yaml`.

#### Suscribed Topics

None.

#### Published Topics

None.

#### Services

- /topological_navigation/get_location (topological_navigation_msgs/GetLocation)
- /topological_navigation/set_location (topological_navigation_msgs/SetLocation)

## Launch

This package contains 2 launch files:

- **topological_navigation_alone.launch:** This launcher is made for testing the package isolated. It launchs the tological_navigation_node and all the actions.
- **topological_navigation.launch:** This launcher is made for being integrated (via including) in a complete application. 2 args must be defined:
  - **pddl_domain_file**: The absolute path of the domain file (the output of pddl_builder in package bica_planning)
  - **topological_map**: The yaml file containing the definition of the topological map.
