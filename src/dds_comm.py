import sys
import os
import time

# --------- PATH SETUP ------------
_here         = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_here, '..'))
for _p in [_project_root, _here]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from lib.dds.dds import *


PHYSICS_HZ = 60
DT         = 1.0 / PHYSICS_HZ   # fixed time step used by ALL Python controllers


class GodotCommunicator:
    def __init__(self, local_port=4445, remote_host='127.0.0.1', remote_port=4444):
        self.dds         = DDS(uPort=local_port)
        self.remote_host = remote_host
        self.remote_port = remote_port

        # Variables we expect to receive from Godot
        self.subscribed_vars = [
            'ObjectDetected',  # 1.0 if object within range, 0.0 otherwise
            'tick',
        ]

    def start(self):
        self.dds.start(self.remote_host, self.remote_port)
        self.dds.subscribe(self.subscribed_vars)

    def stop(self):
        self.dds.stop()

    # --------- SYNCHRONIZATION ------------

    def wait_tick(self, timeout=1.0):
        """
        Block until Godot publishes tick=1.0 (one physics step has elapsed).
        :param timeout: Maximum seconds to wait before giving up (returns False).
        :return: True if tick received, False on timeout.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            val = self.dds.read('tick')
            if val is not None and float(val) >= 1.0:
                return True
            time.sleep(0.0005)   # 0.5 ms polling — avoids busy-waiting
        return False

    # --------- OUT: Python → Godot ------------

    def send_pose(self, x, y, theta):
        """Send robot pose to Godot to update visual representation."""
        self.dds.publish('X',     float(x),     DDS.DDS_TYPE_FLOAT)
        self.dds.publish('Y',     float(y),     DDS.DDS_TYPE_FLOAT)
        self.dds.publish('Theta', float(theta), DDS.DDS_TYPE_FLOAT)

    def send_pick_command(self):
        """Tell Godot to remove the nearest object from the scene."""
        self.dds.publish('Pick', 1.0, DDS.DDS_TYPE_FLOAT)

    def send_drop_command(self):
        """Tell Godot to place the collected object in the basket."""
        self.dds.publish('Drop', 1.0, DDS.DDS_TYPE_FLOAT)

    # --------- IN: Godot → Python ------------
    def check_object(self):
        """Return True if Godot reports an object within pick range."""
        val = self.dds.read('ObjectDetected')
        return val is not None and float(val) > 0.5