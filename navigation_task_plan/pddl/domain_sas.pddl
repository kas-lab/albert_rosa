(define (domain albert)
  (:requirements :typing :negative-preconditions :fluents)

  (:types
    waypoint configuration
  )

  (:constants
    config1 config2 config3 config4 - configuration
  )

  (:predicates
    (at ?w - waypoint)
    (is-corridor ?from ?to - waypoint)
    (is-dark ?from ?to - waypoint)
    (is-lit ?from ?to - waypoint)
    (can-use ?c - configuration)
    (config-valid ?c - configuration)
    (can-traverse ?from ?to - waypoint ?c - configuration)

    ;; Properties
    (uses-lidar ?c - configuration)
    (uses-high-speed ?c - configuration)
    (uses-low-speed ?c - configuration)
  )

  (:functions
    (battery-level)
    (distance ?from - waypoint ?to - waypoint)
    (energy-cost ?from - waypoint ?to - waypoint ?c - configuration)
  )
  
  (:action move_lit
    :parameters (?from ?to - waypoint ?c - configuration)
    :precondition (and
      (at ?from)
      (is-corridor ?from ?to)
      (is-lit ?from ?to)
      (can-use ?c)
      (config-valid ?c)
      (can-traverse ?from ?to ?c)
      (>= (battery-level) (energy-cost ?from ?to ?c))
    )
    :effect (and
      (not (at ?from))
      (at ?to)
      (decrease (battery-level) (energy-cost ?from ?to ?c))
    )
  )

  (:action move_dark
    :parameters (?from ?to - waypoint ?c - configuration)
    :precondition (and
      (at ?from)
      (is-corridor ?from ?to)
      (is-dark ?from ?to)
      (uses-lidar ?c)
      (can-use ?c)
      (config-valid ?c)
      (can-traverse ?from ?to ?c)
      (>= (battery-level) (energy-cost ?from ?to ?c))
    )
    :effect (and
      (not (at ?from))
      (at ?to)
      (decrease (battery-level) (energy-cost ?from ?to ?c))
    )
  )
)
