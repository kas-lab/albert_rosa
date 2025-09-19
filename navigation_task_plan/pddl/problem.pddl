(define (problem navigation)
  (:domain navigation)

  (:objects
    wp1 wp2 wp3 wp4 - waypoint
  )

  (:init
    (robot_at wp1)

    (connected wp1 wp2)
    (connected wp2 wp3)
    (connected wp3 wp4)
    (action_feasible move)

  )

  (:goal
    (robot_at wp4)
  )
)
