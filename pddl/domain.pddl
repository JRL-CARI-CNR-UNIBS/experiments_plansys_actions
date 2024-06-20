(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
person
waypoint
room
clean_waypoint
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?wp - waypoint)
(waypoint_in ?wp - waypoint ?s - room)
(patrolled ?wp - waypoint)
(cleaned ?s - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
    (move_duration ?r - robot ?wp1 ?wp2 - waypoint)
    (clean_duration ?r - robot ?s - room)
    (patrol_duration ?r - robot ?wp - waypoint)
    (attend_person_duration ?r - robot)
);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration (move_duration ?r ?wp1 ?wp2))
    :condition (and
        (at start(robot_at ?r ?wp1))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action patrol
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp))
       )
    :effect (and
        (at end(patrolled ?wp))
    )
)

(:durative-action clean
:parameters (?r - robot ?s - room ?wp - clean_waypoint)
:duration ( = ?duration (clean_duration ?r robot ?s ?room))
:condition (and
        (at start(robot_at ?r ?wp))
        (at start(waypoint_in ?wp ?s))
)
:effect (at end (cleaned ?s)))

(:durative-action attend_person
:parameters (?r - robot ?p - person ?w - waypoint)
:duration (= ?duration 15)
:condition (and
    (at start (in_waypoint ?r ?w))
    (at start (at_waypoint ?p ?w)))
:effect (at start (attending ?r ?p)))


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
