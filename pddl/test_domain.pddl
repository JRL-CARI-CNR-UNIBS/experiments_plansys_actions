(define (domain robot_domain)
  (:requirements :strips :typing :durative-actions)
  (:types robot person room waypoint)
  
  (:predicates
    (robot_at ?r - robot ?w - waypoint)
    (person_at ?p - person ?w - waypoint)
    (cleaned ?rm - room)
    (patrolled ?w - waypoint)
    (delivered ?p - person ?w - waypoint)
    (waypoint_of_room ?w - waypoint ?rm - room)
    (entrance_waypoint ?w - waypoint)
    (destination_waypoint ?w - waypoint ?p - person)
  )
  
  (:durative-action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :duration (= ?duration 1)
    :condition (and (at start (robot_at ?r ?from)))
    :effect (and (at start (not (robot_at ?r ?from)))
                 (at end (robot_at ?r ?to)))
  )

  (:durative-action clean
    :parameters (?r - robot ?rm - room ?w - waypoint)
    :duration (= ?duration 5)
    :condition (and (at start (robot_at ?r ?w)) (at start (waypoint_of_room ?w ?rm)))
    :effect (at end (cleaned ?rm))
  )

  (:durative-action patrol
    :parameters (?r - robot ?w - waypoint)
    :duration (= ?duration 2)
    :condition (at start (robot_at ?r ?w))
    :effect (at end (patrolled ?w))
  )

  (:durative-action attend_person
    :parameters (?r - robot ?p - person ?e - waypoint ?d - waypoint)
    :duration (= ?duration 3)
    :condition (and (at start (robot_at ?r ?e)) 
                    (at start (person_at ?p ?e)) 
                    (at start (entrance_waypoint ?e)) 
                    (at start (destination_waypoint ?d ?p)))
    :effect (and (at end (delivered ?p ?d)) 
                 (at end (person_at ?p ?d)))
  )
)
