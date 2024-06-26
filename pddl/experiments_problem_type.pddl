(define (problem cleaning_problem)
  (:domain robot_domain)
  
  (:objects
    robot1 robot2 - robot
    personA - person
    room1 room2 room3 - room
    hall entrance1 entrance2 entrance3 sofa corridor - waypoint
    start_wp_robot1 start_wp_robot2 start_wp_robot3 - waypoint  ; Robot starting waypoint
    clean1 clean2 clean3 - cleaning_waypoint
    entrance - welcome_waypoint
    table - destination_waypoint
  )
  
  (:init
    ; Initial state of the robot
    
    (robot_at robot1 start_wp_robot1)
    (robot_at robot2 start_wp_robot2)
    
    ; Initial state of the person

    (person_at personA entrance)
    
    ; Waypoints of the rooms
    
    (waypoint_of_room clean1 room1)
    (waypoint_of_room clean2 room2)
    (waypoint_of_room clean3 room3)
    
    ; Typing of the waypoints for cleaning
  )
  
  (:goal
    (and
      (delivered personA)
      (cleaned room1)
      (cleaned room2)
      (cleaned room3)
      (patrolled corridor)
      (patrolled hall)
    )

  )

)