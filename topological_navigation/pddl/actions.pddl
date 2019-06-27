(:durative-action cross
    :parameters (?r - robot ?from ?to - room ?d - door ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (over all(door_opened ?d))

        (at start(waypoint_at ?wp1 ?from))
        (at start(waypoint_at ?wp2 ?to))

        (at start(robot_at_room ?r ?from))

        (at start(door_connected ?d ?from ?to ?wp1 ?wp2))
        (at start(robot_at ?r ?wp1))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at start(not(robot_at_room ?r ?from)))
        (at end(robot_at_room ?r ?to))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action navigate
    :parameters (?r - robot ?from ?to - room ?d - door ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 10)
    :condition (and

        (at start(waypoint_at ?wp1 ?from))
        (at start(waypoint_at ?wp2 ?to))

        (at start(robot_at_room ?r ?from))

        (at start(free_connected ?from ?to))
        (at start(robot_at ?r ?wp1))
    )
    :effect (and
      (at start(not(robot_at ?r ?wp1)))
      (at start(not(robot_at_room ?r ?from)))
      (at end(robot_at_room ?r ?to))
      (at end(robot_at ?r ?wp2))
    )
)


(:durative-action move
    :parameters (?r - robot ?from ?to - waypoint ?room - room)
    :duration ( = ?duration 10)
    :condition (and
        (at start(waypoint_at ?from ?room))
        (at start(waypoint_at ?to ?room))
        (at start(robot_at_room ?r ?room))
        (at start(robot_at ?r ?from))
    )
    :effect (and
       (at start(not(robot_at ?r ?from)))
       (at end(robot_at ?r ?to))
   )
)
