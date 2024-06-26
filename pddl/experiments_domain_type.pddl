(define (domain robot_domain)
  (:requirements :typing :durative-actions)
  
  (:types
    robot
    room
    cleaning_waypoint - waypoint
    welcome_waypoint - waypoint
    destination_waypoint - waypoint
    person
  )
  
  (:predicates
    (robot_at ?r - robot ?w - waypoint)
    (person_at ?p - person ?w - waypoint)
    (cleaned ?rm - room)
    (patrolled ?w - waypoint)
    (delivered ?p - person)
    (waypoint_of_room ?w - waypoint ?rm - room)
  )
  
  (:durative-action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :duration (= ?duration 2) 
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
    :duration (= ?duration 5) 
    :condition (and
      (over all (robot_at ?r ?w))
      (over all (waypoint_of_room ?w ?rm))
    )
    :effect (at end (cleaned ?rm))
  )
  
  (:durative-action patrol
    :parameters (?r - robot ?w - waypoint)
    :duration (= ?duration 3) 
    :condition (and
      (over all (robot_at ?r ?w))
    )
    :effect (at end (patrolled ?w))
  )
  
  (:durative-action attend_person
    :parameters (?r - robot ?p - person ?w_welcome - welcome_waypoint ?w_dest - destination_waypoint)
    :duration (= ?duration 4)
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
