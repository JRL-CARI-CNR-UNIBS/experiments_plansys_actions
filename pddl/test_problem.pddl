(define (problem robot_problem)
  (:domain robot_domain)
  (:objects
    robot1 robot2 - robot
    person1 - person
    room1 room2 room3 - room
    ;; Waypoints lists
    entrance waypoint1_room1 cleaning_wp_room1 patrol_wp1_room1 patrol_wp2_room1 
    waypoint1_room2 cleaning_wp_room2 patrol_wp1_room2 patrol_wp2_room2 
    waypoint1_room3 cleaning_wp_room3 patrol_wp1_room3 patrol_wp2_room3 
    wp_destination_person1 start_wp_robot1 start_wp_robot2 - waypoint
  )

  (:init
    ;; Robot initial positions
    (robot_at robot1 start_wp_robot1)
    (robot_at robot2 start_wp_robot2)

    ;; Person initial position
    (person_at person1 entrance)

    ;; Waypoint types
    (waypoint_of_room cleaning_wp_room1 room1)
    (waypoint_of_room cleaning_wp_room2 room2)
    (waypoint_of_room cleaning_wp_room3 room3)
    (entrance_waypoint entrance)
    (destination_waypoint wp_destination_person1 person1)
  )

  (:goal
    (and
      ;; All rooms are cleaned
      (cleaned room1)
      (cleaned room2)
      (cleaned room3)
      
      ;; All waypoints are patrolled
      (patrolled patrol_wp1_room1)
      (patrolled patrol_wp2_room1)
      (patrolled patrol_wp1_room2)
      (patrolled patrol_wp2_room2)
      (patrolled patrol_wp1_room3)
      (patrolled patrol_wp2_room3)

      ;; Person is at the destination
      (delivered person1 wp_destination_person1)
    )
  )
)
