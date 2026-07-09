import time
import sys
import os

# --------- PATH SETUP ------------
_here         = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_here, '..'))
for _p in [_project_root, _here]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from lib.data.dataplot import DataPlotter, plot_multiple
from lib.system.polar import *
from lib.system.trajectory import *

from robot       import RobotModel
from controllers import SpeedController, PositionController
from path_planner import DijkstraPlanner, World
from dds_comm    import GodotCommunicator


def navigate(robot, planner, current_node, dest, speed_ctrl, pos_ctrl, comm, delta_t):
    """Plan path and how to go"""

    # 1. Calculate waypoints
    waypoints, dist = planner.plan(current_node, dest)
    
    if not waypoints or dist == float('inf'):
        print(f"  ERROR: No path found to {dest}")
        return False

    # 2. Start path with waypoints (virtual robot)
    pos_ctrl.start_path((robot.x, robot.y), waypoints)
    
    # 3. Send where to go to Godot
    while pos_ctrl.active:
        (v_ref, w_ref) = pos_ctrl.evaluate(delta_t)
        (f_left, f_right) = speed_ctrl.evaluate(delta_t, v_ref, w_ref)
        robot.evaluate(delta_t, f_left, f_right)
        
        # Send updated pose to Godot
        pose = robot.get_pose()
        comm.send_pose(pose[0], pose[1], pose[2])
        
        # Synchronize with Godot
        comm.dds.wait('tick')

    # 4. Send final pose to Godot
    final_pose = robot.get_pose()
    comm.send_pose(final_pose[0], final_pose[1], final_pose[2])

    return True

def main():

    # --------- PROBLEM PARAMETERS ------------
    M = 1.5;    r = 0.14;   b = 0.8;    beta = 0.7;     WB = 0.25

    F_max = 20.0;   T_max = 10.0
    acc, dec = 3.0, 3.0;     v_max = 1.0;       w_max=3.14

    # --------- TUNNING CONSTANTS ------------
    S_kp_l = 9.0;   S_ki_l = 4.0
    S_kp_a = 3.0;   S_ki_a = 15.0

    P_kp_l = 2.0;   P_kp_a = 5.0
    
    # --------- INITIALISE ------------
    world = World(5, 5)
    planner = DijkstraPlanner(world)
    robot = RobotModel(M, r, b, beta, WB)

    # Controllers:
    speed_ctrl = SpeedController(robot, S_kp_l, S_ki_l, F_max, S_kp_a, S_ki_a, T_max)
    pos_ctrl   = PositionController(robot, acc, dec, 0.05, P_kp_l, P_kp_a, v_max, w_max)

    # DDS:
    comm = GodotCommunicator()
    comm.start()

    # TIME:
    delta_t = 0.01  # 10ms per control step (must match Godot tick rate)

    '''
    plot_v = DataPlotter()
    plot_v.set_x("Time (s)")
    plot_v.add_y("v_ref",  "Reference Linear Speed")
    plot_v.add_y("v_real", "Real Linear Speed")

    plot_w = DataPlotter()
    plot_w.set_x("Time (s)")
    plot_w.add_y("w_ref",  "Reference Angular Speed")
    plot_w.add_y("w_real", "Real Angular Speed")
    '''

    # --------- SET STRATEGY ------------
    current_node = world.START_NODE
    start_coords = world.get_node_coords(current_node)
    robot.set_pose(start_coords[0], start_coords[1], 0.0)
    comm.send_pose(start_coords[0], start_coords[1], 0.0)
    print(f"Robot at START: ({start_coords[0]:.2f}, {start_coords[1]:.2f})")

    objects_collected  = 0
    unvisited_targets  = world.TARGET_NODES.copy()
    print(f"Targets: {unvisited_targets}")

    try:
        while objects_collected < 4:

            # Find nearest unvisited target (by Dijkstra cost)
            nearest_target = None
            min_cost       = float('inf')

            for target in unvisited_targets:
                _, cost = planner.plan(current_node, target)
                if cost < min_cost:
                    min_cost       = cost
                    nearest_target = target

            if not nearest_target:
                print("No reachable targets left.")
                break

            print(f"Next target: {nearest_target} (cost: {min_cost:.2f} m)")

            # Navigate to target
            success = navigate(robot, planner, current_node, nearest_target,
                               speed_ctrl, pos_ctrl, comm, delta_t)

            if not success:
                print(f"  Failed to reach {nearest_target}, skipping.")
                unvisited_targets.remove(nearest_target)
                continue

            # Update
            current_node = nearest_target
            unvisited_targets.remove(nearest_target)

            pose = robot.get_pose()
            print(f"  Arrived at {nearest_target} "
                  f"(pose: {pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})")

            # Check for object detection:
            print("  Checking for object...")
    
            object_detected = False
            for _ in range(20):
                comm.send_pose(pose[0], pose[1], pose[2])
                comm.dds.wait('tick')
                if comm.check_object():
                    object_detected = True
                    break

            if object_detected:
                print("  OBJECT DETECTED! Picking...")
                comm.send_pick_command()
                objects_collected += 1
                print(f"  Objects collected: {objects_collected}/4")

                # Wait for ObjectDetected to reset
                for _ in range(50):
                    time.sleep(0.01)
                    if not comm.check_object():
                        break

                # Navigate to basket to drop
                print("  Navigating to basket...")
                success = navigate(robot, planner, current_node, world.BASKET_NODE,
                                   speed_ctrl, pos_ctrl, comm, delta_t)

                if success:
                    print("  Dropping object in basket...")
                    comm.send_drop_command()
                    time.sleep(0.3)
                    current_node = world.BASKET_NODE
                else:
                    print("  ERROR: Could not reach basket!")
            else:
                print("  No object detected at this node.")

        # ══════════════════════════════════════════════════════════════════════
        print(f"\nStrategy complete! Objects collected: {objects_collected}/4")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        comm.stop()
        print("DDS connection closed.")

if __name__ == "__main__":
    main()