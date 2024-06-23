(define (problem clean_problem)
  (:domain patrol)
  (:objects
    robot1 robot2 - robot
    room1 room2 room3 - room
    room1wp first_room door_wp entrance room2_middle first_room_middle table - waypoint
  )
  
  (:init
    ;; Posizioni iniziali dei robot
    (robot_at robot1 door_wp)
    (robot_at robot2 door_wp)

    ;; Waypoint associati alle stanze
    (waypoint_in room1wp room1)
    (waypoint_in first_room room1)
    (waypoint_in door_wp room1)
    (waypoint_in entrance room1)
    (waypoint_in room2_middle room2)
    (waypoint_in first_room_middle room2)
    (waypoint_in table room3)
  )

  (:goal
    (and
      ;; La stanza 1 è pulita
      (cleaned room1)
    )
  )
)
