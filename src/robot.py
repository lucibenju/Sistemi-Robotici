#
# robot.py
#
# Defines the mobile robot model used in this project.
# The robot is based on TwoWheelsCart2D, which models a cylinder-shaped
# differential-drive robot with two INDEPENDENT traction sides (left and right).
# Each side can receive a different force, producing both linear and angular motion.
# The number of physical wheels per side is not specified by the model — the class
# name refers to the two independent drive channels (left / right), not a fixed
# wheel count.
#

import sys
import os

# ── path setup ────────────────────────────────────────────────────────────────
_here         = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_here, '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)
# ──────────────────────────────────────────────────────────────────────────────
from lib.system.cart import TwoWheelsCart2D
from lib.utils.geometry import normalize_angle


class RobotModel(TwoWheelsCart2D):
    """
    Mobile robot with independent left and right traction, based on TwoWheelsCart2D.

    Same dynamic model: receives one force per independent drive side:
        evaluate(delta_t, f_left, f_right)

    Internally the class computes:
        F_total = f_left + f_right                   (net linear force)
        T_total = traction_wheelbase * (f_right - f_left)  (net torque)
    """

    def __init__(self,
                 mass        : float = None,
                 radius      : float = None,
                 lin_friction: float = None,
                 ang_friction: float = None,
                 wheelbase   : float = None):
        """
        Initialize the robot model with optional parameters.
        cart = RobotModel(mass=2.0)                  # override mass only
        cart = RobotModel(1.5, 0.14, 0.8, 0.7, 0.25)  # positional args
        """
        super().__init__(
            _mass               = mass         if mass         is not None else  1.5,
            _radius             = radius       if radius       is not None else  0.14,
            _lin_friction       = lin_friction if lin_friction is not None else  0.8,
            _ang_friction       = ang_friction if ang_friction is not None else  0.7,
            _traction_wheelbase = wheelbase    if wheelbase    is not None else  0.25
        )