(define (problem problem_name)
  (:domain patrol)

  (:objects
    r2d2 - robot
    door_wp entrance first_room - waypoint
  )

  (:init
    (robot_at r2d2 door_wp)
  )

  (:goal
    (patrolled wp1)
  )

)
