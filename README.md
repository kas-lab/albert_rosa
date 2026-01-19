# Proactive Self-Adaptation in Autonomous Mobile Robots via Predictive Feasibility Reasoning

**MSc Thesis - Mohamed Msallak - TU Delft - January 2025**

## Overview

Implementation of a proactive self-adaptation framework for energy-constrained autonomous mobile robots. The system extends ROSA with rolling-horizon battery prediction and integrates PlanSys2 for symbolic task planning, enabling anticipatory task-and-architecture co-adaptation.

**Key features:**
- Rolling-horizon energy prediction over upcoming navigation actions
- Proactive configuration switching and recharge planning
- Benchmark evaluation across procedurally generated environments
- Comparison with reactive threshold-based baseline

⚠️ **Compatibility Notice:** This system has been developed and tested exclusively on **ROS 2 Humble**. Compatibility with other ROS 2 distributions is not guaranteed.

## Installation

### Prerequisites

This project builds on the ROSA framework. Follow ROSA installation instructions:
- **ROSA repository:** https://github.com/kas-lab/rosa

Additional dependencies:
- ROS 2 Humble
- PlanSys2
- TypeDB 2.x
- Nav2 navigation stack

### Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [this-repo]
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running the System

### Prerequisites for All Runs

**1. Start TypeDB Server** (Terminal 1)
```bash
typedb server &
```

**2. Start Battery Monitor** (Terminal 2)

For benchmark mode:
```bash
ros2 run navigation_battery battery_monitor --ros-args -p benchmark_mode:=true
```

For single navigation runs:
```bash
ros2 run navigation_battery battery_monitor --ros-args -p benchmark_mode:=false
```

⚠️ **Important:** The battery monitor must be running before launching the navigation system.

---

## Running the Benchmark

### Proactive Adaptation (Terminal 3)
```bash
ros2 run navigation_planner benchmark_tracker
```

### Reactive Adaptation Baseline (Terminal 3)
```bash
ros2 launch navigation_task_plan navigate_task_plan_reactive.launch.py \
  scenario_difficulty:=easy \
  benchmark_mode:=true \
  max_benchmark_runs:=100
```

**Benchmark Parameters:**
- `scenario_difficulty:=` - Map difficulty: `easy`, `medium`, or `hard`
- `benchmark_mode:=` - Set to `true` for automated benchmark runs
- `max_benchmark_runs:=` - Number of episodes to run
- `enable_debug_in_benchmark:=` - Set to `true` for debug output

### Results

Benchmark results are saved to:
- `benchmark_results/` - Metrics, success rates, configuration usage
- `real_sim_results/` - Simulation validation data

---

## Running Single Navigation Episodes

Useful for testing and debugging individual scenarios.

### Single Run - Proactive Adaptation

**Terminal 1:** TypeDB
```bash
typedb server &
```

**Terminal 2:** Battery Monitor
```bash
ros2 run navigation_battery battery_monitor --ros-args -p benchmark_mode:=false
```

**Terminal 3:** Navigation (Proactive)
```bash
ros2 launch navigation_task_plan navigate_task_plan_horizon.launch.py
```

### Single Run - Reactive Adaptation

**Terminal 1:** TypeDB
```bash
typedb server &
```

**Terminal 2:** Battery Monitor
```bash
ros2 run navigation_battery battery_monitor --ros-args -p benchmark_mode:=false
```

**Terminal 3:** Navigation (Reactive)
```bash
ros2 launch navigation_task_plan navigate_task_plan_reactive.launch.py \
  scenario_difficulty:=easy \
  benchmark_mode:=false \
  max_benchmark_runs:=1 \
  enable_debug_in_benchmark:=false
```

---

## Repository Structure
```
├── navigation_kb/              # TypeDB knowledge base
│   ├── config/
│   │   ├── schema.tql         # KB schema definition
│   │   └── data.tql           # Static domain knowledge
│   └── launch/                # KB launch files
│
├── navigation_rosa/            # ROSA integration
│   ├── config/
│   │   ├── navigation_rosa.tql          # Proactive constraints
│   │   └── navigation_rosa_reactive.tql # Reactive constraints
│   └── launch/                # ROSA launch files
│
├── navigation_task_plan/      # PlanSys2 task planning
│   ├── pddl/
│   │   ├── domain_sas.pddl            # PDDL domain (proactive)
│   │   └── domain_sas_reactive.pddl   # PDDL domain (reactive)
│   ├── src/
│   │   ├── navigate_horizon.cpp       # Proactive controller
│   │   └── navigate_reactive.cpp      # Reactive controller
│   └── launch/
│       └── benchmark_with_map_regen.launch.py  # Main benchmark
│
├── navigation_planner/         # Benchmark & map generation
│   └── navigation_planner/
│       ├── benchmark_tracker.py       # Metrics collection
│       └── map_generator.py           # Procedural map generation
│
├── navigation_battery/         # Battery monitoring
│   └── navigation_battery/
│       └── battery_monitor.py         # Battery discharge model
│
├── benchmark_results/          # Benchmark output data
└── real_sim_results/           # Simulation validation data
```

## Key Configuration Files

### Proactive Thresholds
Located in `navigation_rosa/config/navigation_rosa.tql`

### Reactive Thresholds
Located in `navigation_rosa/config/navigation_rosa_reactive.tql`

### PDDL Domain
- Proactive: `navigation_task_plan/pddl/domain_sas.pddl`
- Reactive: `navigation_task_plan/pddl/domain_sas_reactive.pddl`

### Map Difficulty Parameters
Configured in benchmark launch file parameters:
- `easy`: Dense connectivity, multiple charging stations
- `medium`: Moderate connectivity, limited chargers
- `hard`: Sparse connectivity, clustered chargers

---

## Starting ROSA Independently

If you need to launch ROSA separately (not automatically started by navigation launch):

### ROSA (Proactive)
```bash
typedb server &
ros2 launch navigation_rosa navigation_rosa.launch.py
```

### ROSA (Reactive Baseline)
```bash
typedb server &
ros2 launch navigation_rosa navigation_rosa_reactive.launch.py
```

---

## Troubleshooting

### Battery Monitor Not Publishing
Check that battery monitor is running:
```bash
ros2 node list | grep battery
```

Check battery topic:
```bash
ros2 topic echo /battery_level
```

### TypeDB Connection Issues
Verify TypeDB server is running:
```bash
ps aux | grep typedb
```

Restart if needed:
```bash
pkill typedb
typedb server &
```

### Benchmark Not Starting
Ensure all prerequisites are running:
1. TypeDB server
2. Battery monitor in correct mode
3. Check launch file parameters

---

## Repository Status

**Note:** This repository contains research code developed during an MSc thesis project. The implementation prioritizes experimental validation and rapid iteration over production-level software engineering. The `defense-submission` branch contains the working system used for thesis evaluation.

## Thesis

**Defense:** January 27, 2025  
**Supervisors:** Gustavo Rezende Silva, Carlos Hernández Corbato  
**Document:** [Available after defense]

## Citation
```bibtex
@mastersthesis{msallak2025proactive,
  title={Proactive Self-Adaptation in Autonomous Mobile Robots via Predictive Feasibility Reasoning},
  author={Msallak, Mohamed},
  year={2025},
  school={Delft University of Technology}
}
```

## Contact

Mohamed Msallak - M.Msallak@student.tudelft.nl

## License

See [LICENSE](LICENSE) file for details.