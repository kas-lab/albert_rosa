(define (domain albert)
  (:requirements :typing :negative-preconditions :equality :fluents)
  (:types
    waypoint
    configuration
    action
  )
  (:predicates
    (at ?w - waypoint)
    (is-corridor ?from ?to - waypoint)
    (is-dark ?from ?to - waypoint)
    (is-lit ?from ?to - waypoint)
    (can-use ?c - configuration)
    (config-valid ?c - configuration)
    (can-traverse ?from ?to - waypoint ?c - configuration)
    (has-enough-battery ?from ?to - waypoint ?c - configuration)
    (has-charging-station ?w - waypoint)
    
    ;; ROSA–PlanSys2 predicates
    (move_lit_action ?a - action)
    (move_dark_action ?a - action)
    (move_to_recharge_action ?a - action)  ; ✅ NEW
    (recharge_action ?a - action)
    (action_feasible ?a - action)
    (battery_recharged ?w - waypoint)
  )
  (:functions
    (battery-level)
    (energy-cost ?from - waypoint ?to - waypoint ?c - configuration)
  )

  ;; ═══════════════════════════════════════════════
  ;; EXISTING ACTIONS (unchanged)
  ;; ═══════════════════════════════════════════════
  (:durative-action move_lit
    :parameters (?a - action ?from ?to - waypoint ?c - configuration)
    :duration (= ?duration 1)
    :condition (and
      (at start (at ?from))
      (at start (is-corridor ?from ?to))
      (at start (is-lit ?from ?to))
      (at start (can-use ?c))
      (at start (config-valid ?c))
      (at start (can-traverse ?from ?to ?c))
      (at start (has-enough-battery ?from ?to ?c))
      (at start (move_lit_action ?a))
      (at start (action_feasible ?a))
    )
    :effect (and
      (at end (not (at ?from)))
      (at end (at ?to))
      (at end (decrease (battery-level) (energy-cost ?from ?to ?c)))
    )
  )

  (:durative-action move_dark
    :parameters (?a - action ?from ?to - waypoint ?c - configuration)
    :duration (= ?duration 1)
    :condition (and
      (at start (at ?from))
      (at start (is-corridor ?from ?to))
      (at start (is-dark ?from ?to))
      (at start (can-use ?c))
      (at start (config-valid ?c))
      (at start (can-traverse ?from ?to ?c))
      (at start (has-enough-battery ?from ?to ?c))
      (at start (move_dark_action ?a))
      (at start (action_feasible ?a))
    )
    :effect (and
      (at end (not (at ?from)))
      (at end (at ?to))
      (at end (decrease (battery-level) (energy-cost ?from ?to ?c)))
    )
  )

  ;; ═══════════════════════════════════════════════
  ;; NEW: Emergency move to charging station
  ;; Uses degraded_speed_config (slowest, minimal power)
  ;; ═══════════════════════════════════════════════
  (:durative-action move_to_recharge
    :parameters (?a - action ?from ?to - waypoint ?c - configuration)
    :duration (= ?duration 1)
    :condition (and
      (at start (at ?from))
      (at start (is-corridor ?from ?to))
      (at start (can-use ?c))
      (at start (config-valid ?c))
      (at start (can-traverse ?from ?to ?c))
      (at start (move_to_recharge_action ?a))
      (at start (action_feasible ?a))
    )
    :effect (and
      (at end (not (at ?from)))
      (at end (at ?to))
      (at end (decrease (battery-level) (energy-cost ?from ?to ?c)))
    )
  )

  ;; ═══════════════════════════════════════════════
  ;; RECHARGE (unchanged)
  ;; ═══════════════════════════════════════════════
  (:durative-action recharge
    :parameters (?a - action ?charging - waypoint)
    :duration (= ?duration 1)
    :condition (and
      (at start (at ?charging))
      (at start (has-charging-station ?charging))
      (at start (recharge_action ?a))
      (at start (action_feasible ?a))
    )
    :effect (and
      (at end (battery_recharged ?charging)) 
      (at end (increase (battery-level) 40))
    )
  )
)