(door_connected ?d - door ?r1 ?r2 - room ?wp1 ?wp2 - waypoint)
(free_connected ?r1 ?r2 - room)

(robot_at ?r - robot ?wp - waypoint)
(robot_at_room ?r - robot ?room - room)

(waypoint_at ?wp - waypoint ?r - room)
(door_opened ?d - door)
