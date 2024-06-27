(define (domain robot_domain)
  (:requirements :typing :durative-actions :fluents)
  
  (:types
    robot
    room
    person
    cleaning_waypoint - waypoint
    patrolling_waypoint - waypoint
    welcome_waypoint - waypoint
    destination_waypoint - waypoint 
    general_waypoint - waypoint
  )
  
  (:predicates
    (robot_at ?r - robot ?w - waypoint)
    (person_at ?p - person ?w - waypoint)
    (cleaned ?rm - room)
    (patrolled ?w - waypoint)
    (delivered ?p - person)
    (waypoint_of_room ?w - waypoint ?rm - room)
  )
  
  (:functions
    (duration_move ?r - robot ?from - waypoint ?to - waypoint)
    (duration_clean ?r - robot ?rm - room ?w - cleaning_waypoint)
    (duration_patrol ?r - robot ?w - patrolling_waypoint) 
    (duration_attend_person ?r - robot ?w_welcome - welcome_waypoint ?w_dest - destination_waypoint)
  )

  (:durative-action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :duration (= ?duration (duration_move ?r ?from ?to)) 
    :condition (and
      (at start (robot_at ?r ?from))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )
  
  (:durative-action clean
    :parameters (?r - robot ?rm - room ?w - cleaning_waypoint)
    :duration (= ?duration (duration_clean ?r ?rm ?w))
    :condition (and
      (over all (robot_at ?r ?w))
      (over all (waypoint_of_room ?w ?rm))
    )
    :effect (at end (cleaned ?rm))
  )
  
  (:durative-action patrol
    :parameters (?r - robot ?w - patrolling_waypoint)
    :duration (= ?duration (duration_patrol ?r ?w))
    :condition (and
      (over all (robot_at ?r ?w))
    )
    :effect (at end (patrolled ?w))
  )
  
  (:durative-action attend_person
    :parameters (?r - robot ?p - person ?w_welcome - welcome_waypoint ?w_dest - destination_waypoint)
    :duration (= ?duration (duration_attend_person ?r ?w_welcome ?w_dest)) 
    :condition (and
      (at start (robot_at ?r ?w_welcome))
      (at start (person_at ?p ?w_welcome))
    )
    :effect (and
      (at end (delivered ?p))
      (at start (not (robot_at ?r ?w_welcome)))
      (at end (robot_at ?r ?w_dest))
      (at start (not (person_at ?p ?w_welcome)))
      (at end (person_at ?p ?w_dest))
    )
  )

)