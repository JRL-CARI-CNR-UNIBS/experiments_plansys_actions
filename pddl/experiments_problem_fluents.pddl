(define (problem cleaning_problem)
  (:domain robot_domain)
  
  (:objects
    robot1 robot2 - robot
    personA - person
    room1 room2 room3 - room
    hall entrance clean1 clean2 clean3 sofa - waypoint
    start_wp_robot1 start_wp_robot2 - waypoint  ; Robot starting waypoint
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

    (cleaning_waypoint clean1)
    (cleaning_waypoint clean2)
    (cleaning_waypoint clean3)

    ; Typing of the waypoints for patrolling
    (patrolling_waypoint hall)
    
    ; Welcom waypoint and destination waypoint
    
    (welcome_waypoint entrance)
    (destination_waypoint sofa)  

    ; cleaning time
    
    (= (duration_clean robot1 room1 clean1) 10.1)
    (= (duration_clean robot1 room2 clean2) 10.1)
    (= (duration_clean robot1 room3 clean3) 10.1)
    (= (duration_clean robot2 room1 clean1) 10.2)
    (= (duration_clean robot2 room2 clean2) 10.2)
    (= (duration_clean robot2 room3 clean3) 10.2)

    ; patrolling time

    (= (duration_patrol robot1 hall) 5.2)
    (= (duration_patrol robot2 hall) 5.4)

    (= (duration_move robot1 start_wp_robot1 start_wp_robot2) 11)
    (= (duration_move robot1 start_wp_robot1 hall) 10)
    (= (duration_move robot1 start_wp_robot1 entrance) 13)
    (= (duration_move robot1 start_wp_robot1 clean1) 8)
    (= (duration_move robot1 start_wp_robot1 clean2) 11)
    (= (duration_move robot1 start_wp_robot1 clean3) 11)
    (= (duration_move robot1 start_wp_robot1 sofa) 11)
    (= (duration_move robot1 start_wp_robot2 start_wp_robot1) 8)
    (= (duration_move robot1 start_wp_robot2 hall) 15)
    (= (duration_move robot1 start_wp_robot2 entrance) 14)
    (= (duration_move robot1 start_wp_robot2 clean1) 5)
    (= (duration_move robot1 start_wp_robot2 clean2) 9)
    (= (duration_move robot1 start_wp_robot2 clean3) 7)
    (= (duration_move robot1 start_wp_robot2 sofa) 8)
    (= (duration_move robot1 hall start_wp_robot1) 13)
    (= (duration_move robot1 hall start_wp_robot2) 7)
    (= (duration_move robot1 hall entrance) 3)
    (= (duration_move robot1 hall clean1) 11)
    (= (duration_move robot1 hall clean2) 6)
    (= (duration_move robot1 hall clean3) 3)
    (= (duration_move robot1 hall sofa) 12)
    (= (duration_move robot1 entrance start_wp_robot1) 11)
    (= (duration_move robot1 entrance start_wp_robot2) 4)
    (= (duration_move robot1 entrance hall) 13)
    (= (duration_move robot1 entrance clean1) 7)
    (= (duration_move robot1 entrance clean2) 2)
    (= (duration_move robot1 entrance clean3) 12)
    (= (duration_move robot1 entrance sofa) 7)
    (= (duration_move robot1 clean1 start_wp_robot1) 3)
    (= (duration_move robot1 clean1 start_wp_robot2) 9)
    (= (duration_move robot1 clean1 hall) 7)
    (= (duration_move robot1 clean1 entrance) 3)
    (= (duration_move robot1 clean1 clean2) 4)
    (= (duration_move robot1 clean1 clean3) 9)
    (= (duration_move robot1 clean1 sofa) 13)
    (= (duration_move robot1 clean2 start_wp_robot1) 10)
    (= (duration_move robot1 clean2 start_wp_robot2) 6)
    (= (duration_move robot1 clean2 hall) 15)
    (= (duration_move robot1 clean2 entrance) 11)
    (= (duration_move robot1 clean2 clean1) 5)
    (= (duration_move robot1 clean2 clean3) 5)
    (= (duration_move robot1 clean2 sofa) 15)
    (= (duration_move robot1 clean3 start_wp_robot1) 14)
    (= (duration_move robot1 clean3 start_wp_robot2) 11)
    (= (duration_move robot1 clean3 hall) 2)
    (= (duration_move robot1 clean3 entrance) 6)
    (= (duration_move robot1 clean3 clean1) 11)
    (= (duration_move robot1 clean3 clean2) 12)
    (= (duration_move robot1 clean3 sofa) 5)
    (= (duration_move robot1 sofa start_wp_robot1) 10)
    (= (duration_move robot1 sofa start_wp_robot2) 2)
    (= (duration_move robot1 sofa hall) 12)
    (= (duration_move robot1 sofa entrance) 7)
    (= (duration_move robot1 sofa clean1) 8)
    (= (duration_move robot1 sofa clean2) 14)
    (= (duration_move robot1 sofa clean3) 10)
    (= (duration_move robot2 start_wp_robot1 start_wp_robot2) 15)
    (= (duration_move robot2 start_wp_robot1 hall) 6)
    (= (duration_move robot2 start_wp_robot1 entrance) 8)
    (= (duration_move robot2 start_wp_robot1 clean1) 6)
    (= (duration_move robot2 start_wp_robot1 clean2) 2)
    (= (duration_move robot2 start_wp_robot1 clean3) 9)
    (= (duration_move robot2 start_wp_robot1 sofa) 13)
    (= (duration_move robot2 start_wp_robot2 start_wp_robot1) 7)
    (= (duration_move robot2 start_wp_robot2 hall) 9)
    (= (duration_move robot2 start_wp_robot2 entrance) 9)
    (= (duration_move robot2 start_wp_robot2 clean1) 3)
    (= (duration_move robot2 start_wp_robot2 clean2) 11)
    (= (duration_move robot2 start_wp_robot2 clean3) 14)
    (= (duration_move robot2 start_wp_robot2 sofa) 7)
    (= (duration_move robot2 hall start_wp_robot1) 13)
    (= (duration_move robot2 hall start_wp_robot2) 7)
    (= (duration_move robot2 hall entrance) 3)
    (= (duration_move robot2 hall clean1) 14)
    (= (duration_move robot2 hall clean2) 10)
    (= (duration_move robot2 hall clean3) 10)
    (= (duration_move robot2 hall sofa) 15)
    (= (duration_move robot2 entrance start_wp_robot1) 5)
    (= (duration_move robot2 entrance start_wp_robot2) 7)
    (= (duration_move robot2 entrance hall) 5)
    (= (duration_move robot2 entrance clean1) 9)
    (= (duration_move robot2 entrance clean2) 11)
    (= (duration_move robot2 entrance clean3) 14)
    (= (duration_move robot2 entrance sofa) 4)
    (= (duration_move robot2 clean1 start_wp_robot1) 14)
    (= (duration_move robot2 clean1 start_wp_robot2) 3)
    (= (duration_move robot2 clean1 hall) 3)
    (= (duration_move robot2 clean1 entrance) 4)
    (= (duration_move robot2 clean1 clean2) 10)
    (= (duration_move robot2 clean1 clean3) 14)
    (= (duration_move robot2 clean1 sofa) 7)
    (= (duration_move robot2 clean2 start_wp_robot1) 3)
    (= (duration_move robot2 clean2 start_wp_robot2) 13)
    (= (duration_move robot2 clean2 hall) 8)
    (= (duration_move robot2 clean2 entrance) 15)
    (= (duration_move robot2 clean2 clean1) 10)
    (= (duration_move robot2 clean2 clean3) 9)
    (= (duration_move robot2 clean2 sofa) 3)
    (= (duration_move robot2 clean3 start_wp_robot1) 2)
    (= (duration_move robot2 clean3 start_wp_robot2) 2)
    (= (duration_move robot2 clean3 hall) 4)
    (= (duration_move robot2 clean3 entrance) 6)
    (= (duration_move robot2 clean3 clean1) 12)
    (= (duration_move robot2 clean3 clean2) 13)
    (= (duration_move robot2 clean3 sofa) 10)
    (= (duration_move robot2 sofa start_wp_robot1) 2)
    (= (duration_move robot2 sofa start_wp_robot2) 15)
    (= (duration_move robot2 sofa hall) 14)
    (= (duration_move robot2 sofa entrance) 7)
    (= (duration_move robot2 sofa clean1) 11)
    (= (duration_move robot2 sofa clean2) 9)
    (= (duration_move robot2 sofa clean3) 12)

    (= (duration_attend_person robot1 entrance sofa) 5)
    (= (duration_attend_person robot2 entrance sofa) 5)
  )
  
  (:goal
    (and
      (delivered personA)
      (cleaned room1)
      (cleaned room2)
      (cleaned room3)
      (patrolled hall)
    )

  )

)