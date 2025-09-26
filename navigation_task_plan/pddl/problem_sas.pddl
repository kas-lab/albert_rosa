(define (problem albert)
  (:domain albert)

  (:objects
    wp_0 wp_1 wp_2 wp_3 wp_4 - waypoint
  )

  (:init
    (at wp_0)

    ;; Corridor map
    (is-corridor wp_0 wp_1)
    (is-corridor wp_0 wp_2)
    (is-corridor wp_1 wp_3)
    (is-corridor wp_2 wp_3)
    (is-corridor wp_3 wp_4)

    ;; Lighting conditions (replacing NOT is-dark with explicit is-lit)
    (is-lit wp_0 wp_1)
    (is-dark wp_0 wp_2)
    (is-lit wp_1 wp_3)
    (is-lit wp_2 wp_3)
    (is-dark wp_3 wp_4)

    ;; Configuration definitions
    (can-use config1)
    (can-use config2)
    (can-use config3)
    (can-use config4)

    (config-valid config1)
    (config-valid config2)
    (config-valid config3)
    (config-valid config4)

    ;; Property mappings
    (uses-lidar config1)
    (uses-high-speed config1)

    (uses-low-speed config2)

    (uses-high-speed config3)

    (uses-lidar config4)
    (uses-low-speed config4)

    ;; Traversal permissions
    ;; wp_0 → wp_1: not dark → all configs allowed
    (can-traverse wp_0 wp_1 config1)
    (can-traverse wp_0 wp_1 config2)
    (can-traverse wp_0 wp_1 config3)
    (can-traverse wp_0 wp_1 config4)

    ;; wp_0 → wp_2: dark → only configs with lidar
    (can-traverse wp_0 wp_2 config1)
    (can-traverse wp_0 wp_2 config4)

    ;; wp_1 → wp_3: not dark → all configs allowed
    (can-traverse wp_1 wp_3 config1)
    (can-traverse wp_1 wp_3 config2)
    (can-traverse wp_1 wp_3 config3)
    (can-traverse wp_1 wp_3 config4)

    ;; wp_2 → wp_3: not dark → all configs allowed
    (can-traverse wp_2 wp_3 config1)
    (can-traverse wp_2 wp_3 config2)
    (can-traverse wp_2 wp_3 config3)
    (can-traverse wp_2 wp_3 config4)

    ;; wp_3 → wp_4: dark → only configs with lidar
    (can-traverse wp_3 wp_4 config1)
    (can-traverse wp_3 wp_4 config4)

    ;; Distances
    (= (distance wp_0 wp_1) 20)
    (= (distance wp_0 wp_2) 10)
    (= (distance wp_1 wp_3) 20)
    (= (distance wp_2 wp_3) 20)
    (= (distance wp_3 wp_4) 10)

    ;; Battery
    (= (battery-level) 70)

    ;; Energy costs based on speed and distance
    ;; High-speed: 1.5x distance
    ;; Low-speed: 1.0x distance

    ;; wp_0 → wp_1 (distance 20)
    (= (energy-cost wp_0 wp_1 config1) 30) ; 1.5×20
    (= (energy-cost wp_0 wp_1 config2) 20) ; 1.0×20
    (= (energy-cost wp_0 wp_1 config3) 24) ; 1.2×20
    (= (energy-cost wp_0 wp_1 config4) 28) ; 1.4×20

    ;; wp_0 → wp_2 (distance 10)
    (= (energy-cost wp_0 wp_2 config1) 15) ; 1.5×10
    (= (energy-cost wp_0 wp_2 config2) 10) ; 1.0×10
    (= (energy-cost wp_0 wp_2 config3) 12) ; 1.2×10
    (= (energy-cost wp_0 wp_2 config4) 14) ; 1.4×10

    ;; wp_1 → wp_3 (distance 20)
    (= (energy-cost wp_1 wp_3 config1) 30)
    (= (energy-cost wp_1 wp_3 config2) 20)
    (= (energy-cost wp_1 wp_3 config3) 24)
    (= (energy-cost wp_1 wp_3 config4) 28)

    ;; wp_2 → wp_3 (distance 20)
    (= (energy-cost wp_2 wp_3 config1) 30)
    (= (energy-cost wp_2 wp_3 config2) 20)
    (= (energy-cost wp_2 wp_3 config3) 24)
    (= (energy-cost wp_2 wp_3 config4) 28)

    ;; wp_3 → wp_4 (distance 10)
    (= (energy-cost wp_3 wp_4 config1) 15)
    (= (energy-cost wp_3 wp_4 config2) 10)
    (= (energy-cost wp_3 wp_4 config3) 12)
    (= (energy-cost wp_3 wp_4 config4) 14)
  )

  (:goal (at wp_4))
)
