#!/usr/bin/env python3
"""
Proactive Adaptation Loop Benchmark
====================================
Simulates the FULL navigate.cpp proactive adaptation behavior:
- Generates initial plan
- Simulates execution step-by-step
- Updates predicted battery to ROSA
- ROSA re-evaluates constraints
- Configs become unfeasible dynamically
- System replans with new config
- Tracks adaptation behavior over 100 runs

This mimics the enable_proactive loop in navigate.cpp!
"""

import subprocess
import time
import json
import random
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, asdict
import numpy as np
from typedb.driver import TypeDB, SessionType, TransactionType


@dataclass
class AdaptationEvent:
    """Records when adaptation happened"""
    action_index: int
    trigger: str  # "battery_low", "corridor_narrow", etc.
    old_config: str
    new_config: str
    predicted_battery: float
    actual_battery: float


@dataclass
class ProactiveRunResult:
    """Results from one proactive run"""
    run_id: int
    init_wp: str
    goal_wp: str
    initial_battery: float
    success: bool
    
    # Planning stats
    initial_planning_time: float
    num_replans: int
    total_planning_time: float
    
    # Adaptation stats
    adaptations: List[AdaptationEvent]
    configs_used: List[str]
    
    # Execution simulation
    actions_completed: int
    final_battery: float
    
    error: str = ""


class ProactiveAdaptationBenchmark:
    """
    Simulates navigate.cpp proactive adaptation loop
    """
    
    def __init__(self,
                 typedb_address: str = "localhost:1729",
                 database: str = "navigation",
                 domain_file: str = "pddl/domain.pddl",
                 n_runs: int = 100,
                 seed: Optional[int] = None):
        """
        Initialize benchmark
        """
        self.typedb_address = typedb_address
        self.database = database
        self.domain_file = Path(domain_file)
        self.n_runs = n_runs
        self.seed = seed
        
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        print(f"🔌 Connecting to TypeDB...")
        self.driver = TypeDB.core_driver(typedb_address)
        
        # Load map data
        print("📍 Loading map data...")
        self.waypoints = self._fetch_waypoints()
        self.corridors = self._fetch_corridors()
        self.corridor_distances = self._fetch_corridor_distances()
        self.corridor_widths = self._fetch_corridor_widths()
        self.chargers = self._fetch_charging_stations()
        self.lighting = self._fetch_lighting()
        
        print(f"   ✓ {len(self.waypoints)} waypoints")
        print(f"   ✓ {len(self.corridors)} corridors")
        print(f"   ✓ {len(self.corridor_distances)} distances")
        print(f"   ✓ {len(self.corridor_widths)} widths")
        
        self.results: List[ProactiveRunResult] = []
    
    # ═══════════════════════════════════════════════════════════
    # TypeDB Queries (same as before)
    # ═══════════════════════════════════════════════════════════
    
    def _fetch_waypoints(self) -> List[str]:
        """Fetch waypoints"""
        waypoints = []
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = "match $wp isa waypoint, has waypoint-name $name; get $name;"
                for answer in tx.query().get(query):
                    waypoints.append(answer.get("name").get_value())
        return sorted(waypoints)
    
    def _fetch_corridors(self) -> List[Tuple[str, str]]:
        """Fetch corridors"""
        corridors = []
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """
                    match $c (from: $w1, to: $w2) isa corridor;
                    $w1 has waypoint-name $a; $w2 has waypoint-name $b;
                    get $a, $b;
                """
                for answer in tx.query().get(query):
                    a = answer.get("a").get_value()
                    b = answer.get("b").get_value()
                    corridors.append((a, b))
                    corridors.append((b, a))
        return corridors
    
    def _fetch_corridor_distances(self) -> Dict[Tuple[str, str], float]:
        """Fetch distances"""
        distances = {}
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """
                    match $c (from: $w1, to: $w2) isa corridor, has distance $d;
                    $w1 has waypoint-name $a; $w2 has waypoint-name $b;
                    get $a, $b, $d;
                """
                for answer in tx.query().get(query):
                    a = answer.get("a").get_value()
                    b = answer.get("b").get_value()
                    dist = answer.get("d").get_value()
                    distances[(a, b)] = dist
                    distances[(b, a)] = dist
        return distances
    
    def _fetch_corridor_widths(self) -> Dict[Tuple[str, str], float]:
        """Fetch corridor widths"""
        widths = {}
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """
                    match $c (from: $w1, to: $w2) isa corridor, has corridor-width $w;
                    $w1 has waypoint-name $a; $w2 has waypoint-name $b;
                    get $a, $b, $w;
                """
                for answer in tx.query().get(query):
                    a = answer.get("a").get_value()
                    b = answer.get("b").get_value()
                    width = answer.get("w").get_value()
                    widths[(a, b)] = width
                    widths[(b, a)] = width
        return widths
    
    def _fetch_charging_stations(self) -> List[str]:
        """Fetch chargers"""
        chargers = []
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """
                    match $wp isa waypoint, has waypoint-name $name,
                        has has-charging-station true;
                    get $name;
                """
                for answer in tx.query().get(query):
                    chargers.append(answer.get("name").get_value())
        return chargers
    
    def _fetch_lighting(self) -> Dict[Tuple[str, str], Dict[str, bool]]:
        """Fetch lighting"""
        lighting = {}
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """
                    match $l (from: $w1, to: $w2) isa lighting-condition,
                        has is-lit $lit, has is-dark $dark;
                    $w1 has waypoint-name $a; $w2 has waypoint-name $b;
                    get $a, $b, $lit, $dark;
                """
                for answer in tx.query().get(query):
                    a = answer.get("a").get_value()
                    b = answer.get("b").get_value()
                    is_lit = answer.get("lit").get_value()
                    is_dark = answer.get("dark").get_value()
                    lighting[(a, b)] = {"is_lit": is_lit, "is_dark": is_dark}
        return lighting
    
    # ═══════════════════════════════════════════════════════════
    # ROSA Simulation (mimics navigate.cpp behavior)
    # ═══════════════════════════════════════════════════════════
    
    def update_predicted_battery_in_kb(self, predicted_level: float):
        """
        Simulates updatePredictedBatteryInKB() from navigate.cpp
        Updates TypeDB measurement, triggers ROSA reasoning
        """
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as tx:
                # Update predicted battery measurement
                query = """
                    match
                        $m (measured-attribute: $qa) isa measurement,
                            has latest true;
                        $qa has measure-name 'predicted-battery-level';
                    delete $m has latest true;
                """
                tx.query().delete(query)
                
                insert_query = f"""
                    match $qa isa QualityAttribute,
                        has measure-name 'predicted-battery-level';
                    insert
                        (measured-attribute: $qa) isa measurement,
                            has measurement-value {predicted_level},
                            has latest true;
                """
                tx.query().insert(insert_query)
                tx.commit()
        
        # Small delay to let ROSA process
        time.sleep(0.1)
    
    def query_feasible_configs(self, battery_level: float) -> List[str]:
        """
        Simulates getFeasibleConfigsFromKB() from navigate.cpp
        Queries ROSA for configs that satisfy constraints
        """
        # Update predicted battery first (triggers ROSA)
        self.update_predicted_battery_in_kb(battery_level)
        
        configs = []
        with self.driver.session(self.database, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """
                    match
                        $cfg isa component-configuration,
                            has component-configuration-name $name,
                            has priority $pri;
                        not {
                            $cfg has component-configuration-status 'unfeasible';
                        };
                    get $name, $pri;
                """
                
                config_priorities = {}
                for answer in tx.query().get(query):
                    name = answer.get("name").get_value()
                    pri = answer.get("pri").get_value()
                    config_priorities[name] = pri
        
        if not config_priorities:
            return []
        
        # Sort by priority
        sorted_configs = sorted(config_priorities.items(), key=lambda x: x[1])
        return [sorted_configs[0][0]]  # Return best config
    
    # ═══════════════════════════════════════════════════════════
    # Simulation Loop (mimics step() from navigate.cpp)
    # ═══════════════════════════════════════════════════════════
    
    def simulate_proactive_execution(self,
                                    init: str,
                                    goal: str,
                                    initial_battery: float,
                                    run_folder: Path) -> ProactiveRunResult:
        """
        Simulates FULL proactive adaptation loop:
        1. Generate initial plan
        2. Simulate execution action-by-action
        3. Update predicted battery after each action
        4. ROSA re-evaluates constraints
        5. Detect config changes
        6. Replan if needed
        7. Continue until goal or failure
        """
        print(f"\n{'='*60}")
        print(f"SIMULATING PROACTIVE RUN: {init} → {goal}")
        print(f"Initial battery: {initial_battery:.1f}%")
        print(f"{'='*60}\n")
        
        result = ProactiveRunResult(
            run_id=0,
            init_wp=init,
            goal_wp=goal,
            initial_battery=initial_battery,
            success=False,
            initial_planning_time=0.0,
            num_replans=0,
            total_planning_time=0.0,
            adaptations=[],
            configs_used=[],
            actions_completed=0,
            final_battery=initial_battery
        )
        
        current_battery = initial_battery
        current_wp = init
        action_index = 0
        
        while current_wp != goal and current_battery > 0:
            print(f"\n--- Action {action_index} ---")
            print(f"Position: {current_wp}, Battery: {current_battery:.1f}%")
            
            # 1. Query feasible configs (simulates getFeasibleConfigsFromKB)
            feasible_configs = self.query_feasible_configs(current_battery)
            
            if not feasible_configs:
                print("❌ No feasible configs!")
                result.error = "No feasible configuration"
                break
            
            current_config = feasible_configs[0]
            print(f"Config: {current_config}")
            
            # Track config usage
            if not result.configs_used or result.configs_used[-1] != current_config:
                if result.configs_used:
                    # Config changed - adaptation!
                    adaptation = AdaptationEvent(
                        action_index=action_index,
                        trigger="battery_threshold",
                        old_config=result.configs_used[-1],
                        new_config=current_config,
                        predicted_battery=current_battery,
                        actual_battery=current_battery
                    )
                    result.adaptations.append(adaptation)
                    result.num_replans += 1
                    print(f"🔄 ADAPTATION: {adaptation.old_config} → {current_config}")
                
                result.configs_used.append(current_config)
            
            # 2. Plan next action (simplified - just move to next waypoint)
            next_wp = self._get_next_waypoint(current_wp, goal)
            if not next_wp:
                result.error = "No path to goal"
                break
            
            # 3. Simulate action execution
            action_cost = self._calculate_action_cost(
                current_wp, next_wp, current_config
            )
            
            print(f"Action: {current_wp} → {next_wp}, Cost: {action_cost:.2f}%")
            
            # 4. Update battery (simulate action completion)
            current_battery -= action_cost
            current_wp = next_wp
            action_index += 1
            result.actions_completed += 1
            
            # 5. Calculate predicted remaining battery
            remaining_distance = self._estimate_remaining_cost(current_wp, goal, current_config)
            predicted_battery = current_battery - remaining_distance
            
            print(f"After action: Battery={current_battery:.1f}%, Predicted={predicted_battery:.1f}%")
            
            # 6. Update ROSA with prediction (triggers re-evaluation)
            self.update_predicted_battery_in_kb(predicted_battery)
            
            # Small delay to simulate time passing
            time.sleep(0.05)
        
        # Check success
        if current_wp == goal:
            result.success = True
            print(f"\n✅ SUCCESS! Reached {goal}")
        else:
            print(f"\n❌ FAILED at {current_wp} (battery={current_battery:.1f}%)")
        
        result.final_battery = current_battery
        
        print(f"\nAdaptations: {len(result.adaptations)}")
        print(f"Configs used: {result.configs_used}")
        print(f"Actions completed: {result.actions_completed}")
        
        return result
    
    def _get_next_waypoint(self, current: str, goal: str) -> Optional[str]:
        """Get next waypoint towards goal (simplified pathfinding)"""
        # Simple: just move sequentially wp_X → wp_(X+1)
        current_num = int(current.split('_')[1])
        goal_num = int(goal.split('_')[1])
        
        if current_num < goal_num:
            next_num = current_num + 1
            next_wp = f"wp_{next_num}"
            if (current, next_wp) in self.corridors:
                return next_wp
        
        return None
    
    def _calculate_action_cost(self, from_wp: str, to_wp: str, config: str) -> float:
        """Calculate battery cost for one action"""
        distance = self.corridor_distances.get((from_wp, to_wp), 2.0)
        
        # Config speed factors (from your TypeDB data)
        speed_factors = {
            'high_speed_config': 2.0,
            'low_speed_config': 0.6,
            'degraded_speed_config': 0.3
        }
        
        speed = speed_factors.get(config, 1.0)
        cost = distance * speed  # Simplified cost model
        
        return cost
    
    def _estimate_remaining_cost(self, from_wp: str, goal: str, config: str) -> float:
        """Estimate cost to reach goal from current position"""
        total_cost = 0.0
        current = from_wp
        
        while current != goal:
            next_wp = self._get_next_waypoint(current, goal)
            if not next_wp:
                return 999.0  # Unreachable
            
            cost = self._calculate_action_cost(current, next_wp, config)
            total_cost += cost
            current = next_wp
        
        return total_cost
    
    # ═══════════════════════════════════════════════════════════
    # Benchmark Runner
    # ═══════════════════════════════════════════════════════════
    
    def run_benchmark(self, output_folder: Optional[Path] = None):
        """Run proactive adaptation benchmark"""
        if output_folder is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_folder = Path("proactive_results") / f"benchmark_{timestamp}"
        
        output_folder.mkdir(parents=True, exist_ok=True)
        print(f"\n📁 Output: {output_folder}\n")
        
        # Generate random test cases
        print(f"🎲 Generating {self.n_runs} random scenarios...\n")
        
        test_cases = []
        for _ in range(self.n_runs):
            init, goal = random.sample(self.waypoints, 2)
            battery = random.uniform(30.0, 100.0)  # Vary battery
            test_cases.append((init, goal, battery))
        
        # Run simulations
        print("🚀 Starting proactive simulations...\n")
        start_time = time.time()
        
        for run_id, (init, goal, battery) in enumerate(test_cases, 1):
            print(f"\n[{run_id}/{self.n_runs}] {init}→{goal} (battery={battery:.1f}%)")
            
            run_folder = output_folder / f"run_{run_id:03d}"
            run_folder.mkdir(exist_ok=True)
            
            result = self.simulate_proactive_execution(
                init, goal, battery, run_folder
            )
            
            result.run_id = run_id
            self.results.append(result)
            
            # Save intermediate
            if run_id % 10 == 0:
                self._save_results(output_folder)
        
        total_time = time.time() - start_time
        print(f"\n✅ Benchmark completed in {total_time:.2f}s")
        
        self._save_results(output_folder)
        self._print_statistics()
        
        return output_folder
    
    def _save_results(self, folder: Path):
        """Save results"""
        with open(folder / 'proactive_results.json', 'w') as f:
            json.dump([asdict(r) for r in self.results], f, indent=2)
    
    def _print_statistics(self):
        """Print statistics"""
        successful = [r for r in self.results if r.success]
        
        print("\n" + "="*60)
        print("📊 PROACTIVE ADAPTATION STATISTICS")
        print("="*60)
        print(f"Total runs:    {len(self.results)}")
        print(f"Successful:    {len(successful)} ({len(successful)/len(self.results)*100:.1f}%)")
        
        if successful:
            replans = [r.num_replans for r in successful]
            adaptations = [len(r.adaptations) for r in successful]
            
            print(f"\n🔄 Adaptation Behavior:")
            print(f"   Replans:     {np.mean(replans):.1f} ± {np.std(replans):.1f}")
            print(f"   Adaptations: {np.mean(adaptations):.1f} ± {np.std(adaptations):.1f}")
            
            # Config usage
            all_configs = []
            for r in successful:
                all_configs.extend(r.configs_used)
            
            from collections import Counter
            config_counts = Counter(all_configs)
            
            print(f"\n⚙️  Configuration Usage:")
            for cfg, count in config_counts.most_common():
                print(f"   {cfg}: {count} times")
        
        print("="*60 + "\n")
    
    def __del__(self):
        if hasattr(self, 'driver'):
            self.driver.close()


def main():
    """Entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Proactive Adaptation Benchmark')
    parser.add_argument('--typedb', default='localhost:1729')
    parser.add_argument('--database', default='navigation')
    parser.add_argument('--domain', default='pddl/domain.pddl')
    parser.add_argument('--runs', type=int, default=100)
    parser.add_argument('--seed', type=int, default=None)
    
    args = parser.parse_args()
    
    benchmark = ProactiveAdaptationBenchmark(
        typedb_address=args.typedb,
        database=args.database,
        domain_file=args.domain,
        n_runs=args.runs,
        seed=args.seed
    )
    
    benchmark.run_benchmark()


if __name__ == '__main__':
    main()