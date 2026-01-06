#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ros_typedb_msgs.srv import Query
import random
import math
import time

# Optional plotting
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class MapRegenerator(Node):
    def __init__(self):
        super().__init__('map_regenerator')

        # TypeDB client
        self.typedb_client = self.create_client(Query, '/rosa_kb/query')

        # Subscribe to completion signal
        self.run_complete_sub = self.create_subscription(
            Int32,
            '/benchmark/run_complete',
            self.run_complete_callback,
            10
        )

        # Parameters
        self.declare_parameter('n_waypoints', 60)
        self.declare_parameter('n_chargers', 2)
        self.declare_parameter('n_narrow_corridors', 6)
        self.declare_parameter('corridor_length', 5.0)
        self.declare_parameter('start_seed', 103)

        self.n_waypoints = self.get_parameter('n_waypoints').value
        self.n_chargers = self.get_parameter('n_chargers').value
        self.n_narrow_corridors = self.get_parameter('n_narrow_corridors').value
        self.corridor_length = self.get_parameter('corridor_length').value
        self.current_seed = self.get_parameter('start_seed').value

        self.get_logger().info('🗺️  Map Regenerator ready!')
        self.get_logger().info(f'   Waypoints: {self.n_waypoints}')
        self.get_logger().info(f'   Chargers: {self.n_chargers}')
        self.get_logger().info(f'   Narrow corridors: {self.n_narrow_corridors}')

        while not self.typedb_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for TypeDB service...')

        self.get_logger().info('🔨 Generating initial map...')
        self.generate_and_inject_map()

    # ------------------------------------------------------
    # CALLBACK
    # ------------------------------------------------------
    def run_complete_callback(self, msg):
        self.current_seed += 1
        self.get_logger().info(f"\n🔄 Run {msg.data} complete — regenerating map (seed={self.current_seed})")
        
        # Wait for TypeDB
        self.get_logger().info("⏳ Waiting 3 seconds for TypeDB to settle...")
        time.sleep(3.0)
        
        self.generate_and_inject_map()

    # ------------------------------------------------------
    # MAIN PIPELINE
    # ------------------------------------------------------
    def generate_and_inject_map(self):
        self.get_logger().info('🗑️ Deleting old map...')
        success = self.delete_map_sequential()  # ← NEW: Sequential deletion
        if not success:
            self.get_logger().error('❌ CRITICAL: Failed to delete old map! Aborting regeneration.')
            return
        
        time.sleep(2.0)  # Longer delay after delete

        self.get_logger().info('🏗️ Generating new map...')
        map_data = self.generate_map()

        self.plot_map(map_data)

        self.get_logger().info('💉 Injecting new map...')
        success = self.insert_map(map_data)
        if not success:
            self.get_logger().error('❌ CRITICAL: Failed to insert new map!')
            return

        self.get_logger().info('✅ Map regeneration complete!\n')

    # ------------------------------------------------------
    # DELETE MAP SEQUENTIALLY (one entity type at a time)
    # ------------------------------------------------------
    def delete_map_sequential(self):
        """Delete entities one type at a time to avoid lock contention"""
        
        # Step 1: Delete lighting conditions
        self.get_logger().info('  → Deleting lighting conditions...')
        if not self._delete_entity_type('lighting-condition'):
            return False
        time.sleep(0.5)
        
        # Step 2: Delete corridors
        self.get_logger().info('  → Deleting corridors...')
        if not self._delete_entity_type('corridor'):
            return False
        time.sleep(0.5)
        
        # Step 3: Delete waypoints
        self.get_logger().info('  → Deleting waypoints...')
        if not self._delete_entity_type('waypoint'):
            return False
        
        self.get_logger().info('✓ All map entities deleted')
        return True
    
    def _delete_entity_type(self, entity_type):
        """Delete a single entity type"""
        delete_query = f"""
        match
          $x isa {entity_type};
        delete
          $x isa {entity_type};
        """
        
        for attempt in range(3):
            if attempt > 0:
                self.get_logger().warn(f"    🔄 Retry {attempt}/3 for {entity_type}...")
                time.sleep(2.0)
            
            req = Query.Request()
            req.query_type = "delete"
            req.query = delete_query

            future = self.typedb_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)  # Shorter timeout per entity

            if not future.done():
                self.get_logger().error(f"    ⏱️ TIMEOUT deleting {entity_type} (attempt {attempt + 1}/3)")
                continue

            res = future.result()

            if res is None:
                self.get_logger().error(f"    ❌ Delete {entity_type} returned None (attempt {attempt + 1}/3)")
                continue

            if res.success:
                self.get_logger().info(f"    ✓ Deleted {entity_type}")
                return True
            else:
                self.get_logger().error(f"    ❌ Delete {entity_type} failed (attempt {attempt + 1}/3)")
                continue
        
        self.get_logger().error(f"❌ Failed to delete {entity_type} after 3 attempts!")
        return False

    # ------------------------------------------------------
    # MAP GENERATION
    # ------------------------------------------------------
    def generate_map(self):
        random.seed(self.current_seed)

        # Generate waypoints on grid
        rows = 8
        cols = 8
        self.n_waypoints = rows * cols  # 64 waypoints

        waypoints = []
        for i in range(self.n_waypoints):
            row = i // cols
            col = i % cols
            risk = random.choice(["safe", "safe", "safe", "moderate", "high"])
            waypoints.append({
                "id": i,
                "row": row,
                "col": col,
                "risk_level": risk
            })

        # Chargers: always include wp_0
        chargers = [0]
        if self.n_chargers > 1:
            extra = random.sample(range(1, self.n_waypoints), self.n_chargers - 1)
            chargers.extend(extra)

        # Spanning tree + extra edges
        corridors = []
        connected = {0}
        available = list(range(1, self.n_waypoints))

        while available:
            a = random.choice(list(connected))
            b = available.pop(0)
            corridors.append((a, b))
            connected.add(b)

        extra_count = int(len(corridors) * 0.3)
        possible = [
            (i, j) for i in range(self.n_waypoints)
            for j in range(i + 1, self.n_waypoints)
            if (i, j) not in corridors and (j, i) not in corridors
        ]
        if extra_count > 0:
            corridors.extend(random.sample(possible, min(extra_count, len(possible))))

        narrow_ids = random.sample(range(len(corridors)),
                                   min(self.n_narrow_corridors, len(corridors)))

        corridor_data = []
        for idx, (a, b) in enumerate(corridors):
            if idx in narrow_ids:
                width = round(random.uniform(0.8, 1.0), 1)
            else:
                width = round(random.uniform(1.4, 2.0), 1)

            density = random.choice(["low", "low", "medium", "high"])

            if width < 1.0 or density == "high":
                max_speed = round(random.uniform(0.6, 1.0), 1)
            elif width < 1.4 or density == "medium":
                max_speed = round(random.uniform(1.0, 1.5), 1)
            else:
                max_speed = round(random.uniform(1.5, 2.0), 1)

            corridor_data.append({
                "wp1": a,
                "wp2": b,
                "distance": self.corridor_length,
                "width": width,
                "obstacle_density": density,
                "max_safe_speed": max_speed
            })

        return {
            "waypoints": waypoints,
            "chargers": chargers,
            "corridors": corridor_data
        }

    # ------------------------------------------------------
    # INSERT MAP
    # ------------------------------------------------------
    def insert_map(self, map_data):
        lines = ["insert", ""]

        # WAYPOINTS
        for wp in map_data["waypoints"]:
            charger = "true" if wp["id"] in map_data["chargers"] else "false"
            lines.append(
                f'$wp{wp["id"]} isa waypoint, '
                f'has waypoint-name "wp_{wp["id"]}", '
                f'has has-charging-station {charger}, '
                f'has risk-level "{wp["risk_level"]}";'
            )

        # CORRIDORS
        lines.append("")
        for c in map_data["corridors"]:
            wp1 = c["wp1"]
            wp2 = c["wp2"]
            lines.append(
                f'$c{wp1}{wp2} (from: $wp{wp1}, to: $wp{wp2}) isa corridor, '
                f'has distance {c["distance"]}, '
                f'has corridor-width {c["width"]}, '
                f'has obstacle-density "{c["obstacle_density"]}", '
                f'has max-safe-speed {c["max_safe_speed"]};'
            )

        # LIGHTING (randomly dark or lit)
        lines.append("")

        # ✅ Decide which corridors are dark (30% of them)
        num_dark = int(len(map_data["corridors"]) * 0.3)
        dark_corridor_indices = random.sample(range(len(map_data["corridors"])), num_dark)

        for idx, c in enumerate(map_data["corridors"]):
            wp1 = c["wp1"]
            wp2 = c["wp2"]
            
            # ✅ Check if this corridor is dark
            if idx in dark_corridor_indices:
                # DARK corridor
                lines.append(
                    f'$light{wp1}{wp2} (from: $wp{wp1}, to: $wp{wp2}) '
                    f'isa lighting-condition, has is-lit false, has is-dark true;'
                )
            else:
                # LIT corridor
                lines.append(
                    f'$light{wp1}{wp2} (from: $wp{wp1}, to: $wp{wp2}) '
                    f'isa lighting-condition, has is-lit true, has is-dark false;'
        )

        for attempt in range(3):
            if attempt > 0:
                self.get_logger().warn(f"🔄 Retry attempt {attempt}/3...")
                time.sleep(3.0)
            
            req = Query.Request()
            req.query_type = "insert"
            req.query = "\n".join(lines)

            future = self.typedb_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

            if not future.done():
                self.get_logger().error(f"⏱️ INSERT TIMEOUT after 30 seconds (attempt {attempt + 1}/3)")
                continue

            res = future.result()

            if res is None:
                self.get_logger().error(f"❌ Insert returned None (attempt {attempt + 1}/3)")
                continue

            if res.success:
                self.get_logger().info(
                    f'✓ Inserted {len(map_data["waypoints"])} waypoints, '
                    f'{len(map_data["corridors"])} corridors'
                )
                return True
            else:
                self.get_logger().error(f"❌ Insert failed (attempt {attempt + 1}/3)")
                continue
        
        self.get_logger().error("❌ INSERT FAILED after 3 attempts!")
        return False

    # ------------------------------------------------------
    # PLOT MAP
    # ------------------------------------------------------
    def plot_map(self, map_data):
        if not HAS_MATPLOTLIB:
            return

        try:
            fig, ax = plt.subplots()

            # Plot corridors
            for c in map_data["corridors"]:
                a = map_data["waypoints"][c["wp1"]]
                b = map_data["waypoints"][c["wp2"]]
                x1, y1 = a["col"], -a["row"]
                x2, y2 = b["col"], -b["row"]
                ax.plot([x1, x2], [y1, y2],
                        linewidth=2 if c["width"] < 1.4 else 1,
                        alpha=0.8)

            # Plot waypoints
            xs = [wp["col"] for wp in map_data["waypoints"]]
            ys = [-wp["row"] for wp in map_data["waypoints"]]
            ax.scatter(xs, ys, c="blue", s=80)

            # Chargers in red
            for cid in map_data["chargers"]:
                wp = map_data["waypoints"][cid]
                ax.scatter([wp["col"]], [-wp["row"]], c="red", s=120)

            # Labels
            for wp in map_data["waypoints"]:
                ax.text(wp["col"] + 0.05, -wp["row"] + 0.05,
                        f'wp_{wp["id"]}', fontsize=8)

            ax.set_xlabel("Grid col")
            ax.set_ylabel("Grid row (inverted)")
            ax.set_aspect("equal")
            ax.grid(True, linestyle="--", alpha=0.3)

            fname = f"/home/mohamed/albert_humble_rosa/map_layout_seed_{self.current_seed}.png"
            plt.tight_layout()
            plt.savefig(fname, dpi=140)
            plt.close(fig)

            self.get_logger().info(f"🖼️ Saved map layout to: {fname}")

        except Exception as e:
            self.get_logger().error(f"Plot error: {e}")


def main():
    rclpy.init()
    node = MapRegenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()