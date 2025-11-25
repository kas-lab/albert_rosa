#!/usr/bin/env python3
"""
REALISTIC Map Generator with Spatial Coordinates
================================================
Generates physically realistic maps for warehouse/factory navigation:
- Waypoints have X,Y coordinates
- Distances computed from Euclidean geometry
- Triangle inequality always satisfied
- Realistic layout (not random nonsense!)

Usage:
    python3 map_generator_realistic.py \
        --n_waypoints 11 \
        --n_chargers 2 \
        --seed 42 \
        --area_width 50 \
        --area_length 50
"""

import argparse
import random
import json
import math
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Tuple


@dataclass
class Waypoint:
    """Waypoint with spatial coordinates"""
    name: str
    x: float
    y: float
    has_charger: bool = False
    risk_level: str = "safe"


@dataclass
class Corridor:
    """Corridor between two waypoints"""
    wp1: str
    wp2: str
    distance: float  # Euclidean distance
    width: float
    obstacle_density: str
    max_safe_speed: float
    is_lit: bool


@dataclass
class MapConfig:
    """Map configuration"""
    n_waypoints: int
    n_chargers: int
    corridor_density: float
    lighting_ratio: float
    seed: int
    area_width: float
    area_length: float
    
    # Generated
    waypoints: List[Waypoint] = None
    corridors: List[Corridor] = None


class RealisticMapGenerator:
    """
    Generates REALISTIC maps with spatial coordinates
    """
    
    def __init__(self, config: MapConfig):
        self.config = config
        random.seed(config.seed)
        
        print(f"🏭 Generating REALISTIC warehouse layout")
        print(f"   Area: {config.area_width}m × {config.area_length}m")
        print(f"   Waypoints: {config.n_waypoints}")
        print(f"   Chargers: {config.n_chargers}")
        
        # Generate waypoints with coordinates
        self._generate_waypoints()
        
        # Generate corridor network
        self._generate_corridors()
        
        print(f"\n✅ Generated map:")
        print(f"   Waypoints: {len(self.config.waypoints)}")
        print(f"   Chargers: {sum(1 for wp in self.config.waypoints if wp.has_charger)}")
        print(f"   Corridors: {len(self.config.corridors)}")
        print(f"   Mean distance: {sum(c.distance for c in self.config.corridors) / len(self.config.corridors):.1f}m")
    
    def _generate_waypoints(self):
        """Generate waypoints with random spatial coordinates"""
        waypoints = []
        
        # Generate random positions
        # Use grid with jitter to avoid clustering
        n = self.config.n_waypoints
        grid_size = math.ceil(math.sqrt(n))
        
        cell_width = self.config.area_width / grid_size
        cell_length = self.config.area_length / grid_size
        
        for i in range(n):
            # Grid position
            grid_x = i % grid_size
            grid_y = i // grid_size
            
            # Add jitter (random offset within cell)
            jitter_x = random.uniform(-0.3, 0.3) * cell_width
            jitter_y = random.uniform(-0.3, 0.3) * cell_length
            
            x = (grid_x + 0.5) * cell_width + jitter_x
            y = (grid_y + 0.5) * cell_length + jitter_y
            
            # Clamp to area
            x = max(1.0, min(self.config.area_width - 1.0, x))
            y = max(1.0, min(self.config.area_length - 1.0, y))
            
            # Random risk level
            risk_choice = random.random()
            if risk_choice < 0.7:
                risk = "safe"
            elif risk_choice < 0.9:
                risk = "moderate"
            else:
                risk = "high"
            
            wp = Waypoint(
                name=f"wp_{i}",
                x=round(x, 1),
                y=round(y, 1),
                has_charger=False,
                risk_level=risk
            )
            waypoints.append(wp)
        
        # Select chargers
        # Always include wp_0
        waypoints[0].has_charger = True
        
        # Select others randomly
        if self.config.n_chargers > 1:
            charger_candidates = waypoints[1:]
            n_extra = min(self.config.n_chargers - 1, len(charger_candidates))
            extra_chargers = random.sample(charger_candidates, n_extra)
            for wp in extra_chargers:
                wp.has_charger = True
        
        self.config.waypoints = waypoints
        
        print(f"\n📍 Waypoint positions:")
        for wp in waypoints[:5]:
            charger_mark = "🔌" if wp.has_charger else "  "
            print(f"   {charger_mark} {wp.name}: ({wp.x:.1f}, {wp.y:.1f})")
        if len(waypoints) > 5:
            print(f"   ... and {len(waypoints)-5} more")
    
    def _euclidean_distance(self, wp1: Waypoint, wp2: Waypoint) -> float:
        """Compute Euclidean distance between waypoints"""
        dx = wp2.x - wp1.x
        dy = wp2.y - wp1.y
        return math.sqrt(dx*dx + dy*dy)
    
    def _generate_corridors(self):
        """Generate corridor network with REAL distances"""
        corridors = []
        
        # Step 1: Spanning tree (ensure connectivity)
        available_indices = list(range(len(self.config.waypoints)))
        connected_indices = set()
        connected_indices.add(available_indices.pop(0))
        
        while available_indices:
            # Pick from connected
            idx1 = random.choice(list(connected_indices))
            wp1 = self.config.waypoints[idx1]
            
            # Pick from available and remove
            idx_pos = random.randrange(len(available_indices))
            idx2 = available_indices.pop(idx_pos)
            wp2 = self.config.waypoints[idx2]
            
            # Create corridor with REAL distance
            corridor = self._create_corridor(wp1, wp2)
            corridors.append(corridor)
            
            connected_indices.add(idx2)
        
        print(f"✅ Spanning tree: All {len(self.config.waypoints)} waypoints connected")
        
        # Step 2: Add extra edges
        n = self.config.n_waypoints
        max_edges = n * (n - 1) // 2
        target_edges = int(max_edges * self.config.corridor_density)
        
        # Find all possible edges not yet added
        all_possible = []
        for i, wp1 in enumerate(self.config.waypoints):
            for wp2 in self.config.waypoints[i+1:]:
                if not self._corridor_exists(corridors, wp1.name, wp2.name):
                    all_possible.append((wp1, wp2))
        
        n_to_add = min(target_edges - len(corridors), len(all_possible))
        
        if n_to_add > 0:
            extra_pairs = random.sample(all_possible, n_to_add)
            for wp1, wp2 in extra_pairs:
                corridor = self._create_corridor(wp1, wp2)
                corridors.append(corridor)
        
        self.config.corridors = corridors
        
        print(f"✅ Total corridors: {len(corridors)} ({len(connected_indices)-1} spanning + {len(corridors)-(len(connected_indices)-1)} extra)")
    
    def _create_corridor(self, wp1: Waypoint, wp2: Waypoint) -> Corridor:
        """Create corridor with REAL Euclidean distance"""
        # Compute REAL distance
        distance = round(self._euclidean_distance(wp1, wp2), 1)
        
        # Corridor width (meters)
        width_choice = random.random()
        if width_choice < 0.1:
            width = round(random.uniform(0.8, 1.0), 1)  # Very narrow (10%)
        elif width_choice < 0.3:
            width = round(random.uniform(1.0, 1.4), 1)  # Narrow (20%)
        else:
            width = round(random.uniform(1.4, 2.0), 1)  # Normal/wide (70%)
        
        # Obstacle density
        density_choice = random.random()
        if density_choice < 0.6:
            obstacle_density = "low"
        elif density_choice < 0.85:
            obstacle_density = "medium"
        else:
            obstacle_density = "high"
        
        # Max safe speed (based on width and obstacles)
        if width < 1.0 or obstacle_density == "high":
            max_safe_speed = round(random.uniform(0.6, 1.0), 1)
        elif width < 1.4 or obstacle_density == "medium":
            max_safe_speed = round(random.uniform(1.0, 1.5), 1)
        else:
            max_safe_speed = round(random.uniform(1.5, 2.0), 1)
        
        # Lighting
        is_lit = random.random() < self.config.lighting_ratio
        
        return Corridor(
            wp1=wp1.name,
            wp2=wp2.name,
            distance=distance,
            width=width,
            obstacle_density=obstacle_density,
            max_safe_speed=max_safe_speed,
            is_lit=is_lit
        )
    
    def _corridor_exists(self, corridors: List[Corridor], wp1: str, wp2: str) -> bool:
        """Check if corridor exists"""
        for c in corridors:
            if (c.wp1 == wp1 and c.wp2 == wp2) or (c.wp1 == wp2 and c.wp2 == wp1):
                return True
        return False
    
    def generate_typedb_file(self, output_path: Path):
        """Generate TypeDB file"""
        lines = []
        
        lines.append("# Auto-generated REALISTIC map with spatial coordinates")
        lines.append(f"# Area: {self.config.area_width}m × {self.config.area_length}m")
        lines.append(f"# Seed: {self.config.seed}\n")
        lines.append("insert\n")
        
        # Waypoints
        lines.append("# === WAYPOINTS ===")
        for wp in self.config.waypoints:
            idx = int(wp.name.split('_')[1])
            
            line = f'$wp_{idx} isa waypoint, has waypoint-name "{wp.name}", has risk-level "{wp.risk_level}"'
            
            if wp.has_charger:
                line += ', has has-charging-station true'
            
            line += ';'
            lines.append(line)
            lines.append(f'# Position: ({wp.x:.1f}, {wp.y:.1f})')
        
        lines.append("")
        
        # Corridors
        lines.append("# === CORRIDORS ===")
        for corridor in self.config.corridors:
            idx1 = int(corridor.wp1.split('_')[1])
            idx2 = int(corridor.wp2.split('_')[1])
            var_name = f'c{idx1}{idx2}'
            
            lines.append(f'${var_name} (from: $wp_{idx1}, to: $wp_{idx2}) isa corridor,')
            lines.append(f'  has distance {corridor.distance},')
            lines.append(f'  has corridor-width {corridor.width},')
            lines.append(f'  has obstacle-density "{corridor.obstacle_density}",')
            lines.append(f'  has max-safe-speed {corridor.max_safe_speed},')
            lines.append(f'  has is-lit {str(corridor.is_lit).lower()};')
            lines.append("")
        
        # Lighting conditions
        lines.append("# === LIGHTING CONDITIONS ===")
        for corridor in self.config.corridors:
            idx1 = int(corridor.wp1.split('_')[1])
            idx2 = int(corridor.wp2.split('_')[1])
            light_var = f'light{idx1}{idx2}'
            
            lines.append(f'${light_var} (from: $wp_{idx1}, to: $wp_{idx2}) isa lighting-condition,')
            lines.append(f'  has is-lit {str(corridor.is_lit).lower()},')
            lines.append(f'  has is-dark {str(not corridor.is_lit).lower()};')
            lines.append("")
        
        # Write to file
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            f.write('\n'.join(lines))
        
        print(f"\n💾 Saved to {output_path}")
        
        # Save config with coordinates
        config_file = output_path.parent / f"{output_path.stem}_config.json"
        config_dict = {
            'seed': self.config.seed,
            'area_width': self.config.area_width,
            'area_length': self.config.area_length,
            'n_waypoints': self.config.n_waypoints,
            'n_chargers': self.config.n_chargers,
            'waypoints': [
                {
                    'name': wp.name,
                    'x': wp.x,
                    'y': wp.y,
                    'has_charger': wp.has_charger,
                    'risk_level': wp.risk_level
                }
                for wp in self.config.waypoints
            ],
            'corridors': [
                {
                    'wp1': c.wp1,
                    'wp2': c.wp2,
                    'distance': c.distance,
                    'width': c.width,
                    'obstacle_density': c.obstacle_density,
                    'max_safe_speed': c.max_safe_speed,
                    'is_lit': c.is_lit
                }
                for c in self.config.corridors
            ]
        }
        
        with open(config_file, 'w') as f:
            json.dump(config_dict, f, indent=2)
        
        print(f"💾 Config saved to {config_file}")


def main():
    parser = argparse.ArgumentParser(description='Generate REALISTIC map with spatial coordinates')
    parser.add_argument('--n_waypoints', type=int, default=11)
    parser.add_argument('--n_chargers', type=int, default=2)
    parser.add_argument('--corridor_density', type=float, default=0.6)
    parser.add_argument('--lighting_ratio', type=float, default=0.6)
    parser.add_argument('--seed', type=int, default=None, help='Random seed (None = random)')
    parser.add_argument('--area_width', type=float, default=50.0, help='Width of area (meters)')
    parser.add_argument('--area_length', type=float, default=50.0, help='Length of area (meters)')
    parser.add_argument('--output', type=str, default='realistic_map.tql')
    
    args = parser.parse_args()
    
    # If no seed provided, use random
    if args.seed is None:
        import time
        args.seed = int(time.time() * 1000) % 100000
        print(f"🎲 Using random seed: {args.seed}")
    
    # Validate
    assert 2 <= args.n_waypoints <= 100
    assert 0 <= args.n_chargers <= args.n_waypoints
    assert 0.0 <= args.corridor_density <= 1.0
    assert 0.0 <= args.lighting_ratio <= 1.0
    assert args.area_width > 0 and args.area_length > 0
    #!/usr/bin/env python3
"""
Correct Map Generator - Matches YOUR Schema!
=============================================
Generates maps in the format YOUR TypeDB schema expects:
- Waypoints with risk-level, has-charging-station
- Corridors with (from: to:) format
- Attributes: corridor-width, obstacle-density, max-safe-speed, is-lit
- Lighting conditions as separate relations

NO traversal-energy-cost! navigate.cpp computes those dynamically!
"""

import argparse
import random
import json
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Tuple


@dataclass
class MapConfig:
    """Configuration for generated map"""
    n_waypoints: int
    n_chargers: int
    corridor_density: float
    lighting_ratio: float
    seed: int
    
    # Derived
    waypoint_names: List[str] = None
    charger_waypoints: List[str] = None
    corridors: List[Tuple[str, str, float, float, str, float, bool]] = None
    # Tuple: (wp1, wp2, distance, width, obstacle_density, max_safe_speed, is_lit)


class CorrectMapGenerator:
    """Generates maps matching YOUR actual schema"""
    
    def __init__(self, config: MapConfig):
        self.config = config
        random.seed(config.seed)
        
        # Generate waypoint names
        self.config.waypoint_names = [f'wp_{i}' for i in range(config.n_waypoints)]
        
        # Select charger locations
        self._select_chargers()
        
        # Generate corridor network
        self._generate_corridors()
        
        print(f"✅ Generated map:")
        print(f"   Waypoints: {config.n_waypoints}")
        print(f"   Chargers: {config.n_chargers} at {self.config.charger_waypoints}")
        print(f"   Corridors: {len(self.config.corridors)}")
        print(f"   Lit: {sum(1 for c in self.config.corridors if c[6])}")
    
    def _select_chargers(self):
        """Select which waypoints have charging stations"""
        # Always include wp_0 as a charger (home base)
        chargers = ['wp_0']
        
        # Randomly select remaining chargers
        remaining = [wp for wp in self.config.waypoint_names if wp != 'wp_0']
        
        if self.config.n_chargers > 1:
            additional = random.sample(
                remaining,
                min(self.config.n_chargers - 1, len(remaining))
            )
            chargers.extend(additional)
        
        self.config.charger_waypoints = sorted(chargers, key=lambda x: int(x.split('_')[1]))
    
    def _generate_corridors(self):
        """Generate corridor network"""
        corridors = []
        
        # Step 1: Spanning tree (ensure ALL waypoints connected)
        available = list(self.config.waypoint_names)
        connected = set()
        connected.add(available.pop(0))  # Start with wp_0
        
        # Connect ALL remaining waypoints
        while available:
            # Pick random waypoint from connected set
            wp1 = random.choice(list(connected))
            # Pick random waypoint from available set
            wp2 = available.pop(random.randrange(len(available)))
            
            # Create corridor
            corridor = self._create_corridor(wp1, wp2)
            corridors.append(corridor)
            
            # Add to connected set
            connected.add(wp2)
        
        # Verify all waypoints are connected
        assert len(connected) == len(self.config.waypoint_names), \
            f"Spanning tree failed! Connected: {len(connected)}, Total: {len(self.config.waypoint_names)}"
        
        print(f"✅ Spanning tree: All {len(connected)} waypoints connected")
        
        # Step 2: Add extra edges based on density
        n = self.config.n_waypoints
        max_edges = n * (n - 1) // 2
        target_edges = int(max_edges * self.config.corridor_density)
        
        all_possible = []
        for i, wp1 in enumerate(self.config.waypoint_names):
            for wp2 in self.config.waypoint_names[i+1:]:
                if not self._edge_exists(corridors, wp1, wp2):
                    all_possible.append((wp1, wp2))
        
        n_to_add = min(target_edges - len(corridors), len(all_possible))
        
        if n_to_add > 0:
            extra = random.sample(all_possible, n_to_add)
            
            for wp1, wp2 in extra:
                corridor = self._create_corridor(wp1, wp2)
                corridors.append(corridor)
        
        self.config.corridors = corridors
        
        print(f"✅ Total corridors: {len(corridors)} ({len(connected)-1} spanning + {len(corridors)-(len(connected)-1)} extra)")
    
    def _create_corridor(self, wp1: str, wp2: str) -> Tuple:
        """Create corridor tuple with all attributes"""
        distance = round(random.uniform(2.0, 5.0), 1)
        
        # Corridor width (meters) - some narrow, some wide
        width_choice = random.random()
        if width_choice < 0.1:
            width = round(random.uniform(0.8, 1.0), 1)  # Very narrow (10%)
        elif width_choice < 0.3:
            width = round(random.uniform(1.0, 1.4), 1)  # Narrow (20%)
        else:
            width = round(random.uniform(1.4, 2.0), 1)  # Normal/wide (70%)
        
        # Obstacle density
        density_choice = random.random()
        if density_choice < 0.6:
            obstacle_density = "low"
        elif density_choice < 0.85:
            obstacle_density = "medium"
        else:
            obstacle_density = "high"
        
        # Max safe speed (based on width and obstacles)
        if width < 1.0 or obstacle_density == "high":
            max_safe_speed = round(random.uniform(0.6, 1.0), 1)
        elif width < 1.4 or obstacle_density == "medium":
            max_safe_speed = round(random.uniform(1.0, 1.5), 1)
        else:
            max_safe_speed = round(random.uniform(1.5, 2.0), 1)
        
        # Lighting
        is_lit = random.random() < self.config.lighting_ratio
        
        return (wp1, wp2, distance, width, obstacle_density, max_safe_speed, is_lit)
    
    def _edge_exists(self, corridors: List[Tuple], wp1: str, wp2: str) -> bool:
        """Check if edge exists"""
        for c in corridors:
            if (c[0] == wp1 and c[1] == wp2) or (c[0] == wp2 and c[1] == wp1):
                return True
        return False
    
    def generate_typedb_file(self, output_path: Path):
        """Generate TypeDB file in YOUR format"""
        lines = []
        
        lines.append("# Auto-generated map matching YOUR schema")
        lines.append(f"# Config: {asdict(self.config)}\n")
        lines.append("insert\n")
        
        # Waypoints with risk-level and charging stations
        lines.append("# === WAYPOINTS ===")
        for wp in self.config.waypoint_names:
            wp_idx = int(wp.split('_')[1])
            
            # Random risk level
            risk_choice = random.random()
            if risk_choice < 0.7:
                risk = "safe"
            elif risk_choice < 0.9:
                risk = "moderate"
            else:
                risk = "high"
            
            line = f'$wp_{wp_idx} isa waypoint, has waypoint-name "{wp}", has risk-level "{risk}"'
            
            # Add charging station if applicable
            if wp in self.config.charger_waypoints:
                line += ', has has-charging-station true'
            
            line += ';'
            lines.append(line)
        
        lines.append("")
        
        # Corridors with ALL attributes
        lines.append("# === CORRIDORS ===")
        for i, (wp1, wp2, distance, width, obstacle, max_speed, is_lit) in enumerate(self.config.corridors):
            idx1 = int(wp1.split('_')[1])
            idx2 = int(wp2.split('_')[1])
            var_name = f'c{idx1}{idx2}'
            
            lines.append(f'${var_name} (from: $wp_{idx1}, to: $wp_{idx2}) isa corridor,')
            lines.append(f'  has distance {distance},')
            lines.append(f'  has corridor-width {width},')
            lines.append(f'  has obstacle-density "{obstacle}",')
            lines.append(f'  has max-safe-speed {max_speed};')
            lines.append("")
        
        # Lighting conditions (separate relations)
        lines.append("# === LIGHTING CONDITIONS ===")
        for i, (wp1, wp2, distance, width, obstacle, max_speed, is_lit) in enumerate(self.config.corridors):
            idx1 = int(wp1.split('_')[1])
            idx2 = int(wp2.split('_')[1])
            light_var = f'light{idx1}{idx2}'
            
            lines.append(f'${light_var} (from: $wp_{idx1}, to: $wp_{idx2}) isa lighting-condition,')
            lines.append(f'  has is-lit {str(is_lit).lower()},')
            lines.append(f'  has is-dark {str(not is_lit).lower()};')
            lines.append("")
        
        # Write to file
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            f.write('\n'.join(lines))
        
        print(f"💾 Saved to {output_path}")
        
        # Save config
        config_file = output_path.parent / f"{output_path.stem}_config.json"
        with open(config_file, 'w') as f:
            json.dump(asdict(self.config), f, indent=2, default=str)
        
        print(f"💾 Config saved to {config_file}")


def main():
    parser = argparse.ArgumentParser(description='Generate map matching YOUR schema')
    parser.add_argument('--n_waypoints', type=int, default=10)
    parser.add_argument('--n_chargers', type=int, default=2)
    parser.add_argument('--corridor_density', type=float, default=0.6)
    parser.add_argument('--lighting_ratio', type=float, default=0.6)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', type=str, default='generated_data.tql')
    
    args = parser.parse_args()
    
    # Validate
    assert 2 <= args.n_waypoints <= 100
    assert 0 <= args.n_chargers <= args.n_waypoints
    assert 0.0 <= args.corridor_density <= 1.0
    assert 0.0 <= args.lighting_ratio <= 1.0
    
    # Create config
    config = MapConfig(
        n_waypoints=args.n_waypoints,
        n_chargers=args.n_chargers,
        corridor_density=args.corridor_density,
        lighting_ratio=args.lighting_ratio,
        seed=args.seed
    )
    
    # Generate
    generator = CorrectMapGenerator(config)
    
    # Save
    output_path = Path(args.output)
    generator.generate_typedb_file(output_path)
    
    print(f"\n✅ Done! Load with:")
    print(f"   typedb console")
    print(f"   > transaction navigation data write")
    print(f"   > source {output_path}")
    print(f"   > commit")


if __name__ == '__main__':
    main()
    # Create config
    config = MapConfig(
        n_waypoints=args.n_waypoints,
        n_chargers=args.n_chargers,
        corridor_density=args.corridor_density,
        lighting_ratio=args.lighting_ratio,
        seed=args.seed,
        area_width=args.area_width,
        area_length=args.area_length
    )
    
    # Generate
    generator = RealisticMapGenerator(config)
    
    # Save
    output_path = Path(args.output)
    generator.generate_typedb_file(output_path)
    
    print(f"\n✅ Done!")
    print(f"\n💡 For a different layout, use a different seed:")
    print(f"   python3 {Path(__file__).name} --seed {args.seed + 1}")


if __name__ == '__main__':
    main()