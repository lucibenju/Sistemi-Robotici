import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from lib.system.controllers import PI_Controller
from lib.system.polar import Polar2DController
from lib.system.trajectory import Path2D

class SpeedController:
    """
    Two controllers: one for linear speed (v) and one for angular speed (w).
    Each controller is a PI with saturation:
        - Linear PI:  controls v -> force F
        - Angular PI: controls w -> torque T

    Then computes the wheel forces:
      F = f_left + f_right
      T = wheelbase * (f_right - f_left)
      => f_right = (F + T/wheelbase) / 2
      => f_left  = (F - T/wheelbase) / 2
    """
    def __init__(self, robot, kp_linear, ki_linear, sat_linear, kp_angular, ki_angular, sat_angular):
        
        # S1
        # Output: Force F (N) for linear speed control, with saturation
        self.linear_speed_controller = PI_Controller(_kp=kp_linear, _ki=ki_linear, _sat = sat_linear) 
        # Output: Torque T (Nm) for angular speed control, with saturation
        self.angular_speed_controller = PI_Controller(_kp=kp_angular, _ki=ki_angular, _sat = sat_angular)

        # S2
        self.robot = robot
        self.wheelbase = robot.traction_wheelbase

        self._w_filt = 0.0 # For the feedback (avoid angular oscillation)

    def evaluate(self, delta_t, v_ref, w_ref):
        """"
        :return: (f_left, f_right)
        """
        # Feedback. 
        # v_ref y w_ref comes from virtualRobot and position Control
        v_current = self.robot.v 

        # Filter the angular speed Feedback
        self._w_filt = 0.15 * self.robot.w + 0.85 * self._w_filt
        w_current = self._w_filt
       
        # Calculate errors
        error_v = v_ref - v_current
        error_w = w_ref - w_current
        
        # PI + Saturation:
        # Compute needed Force and Torque using the PI controllers + sat
        F = self.linear_speed_controller.evaluate(delta_t, error_v)
        T = self.angular_speed_controller.evaluate(delta_t, error_w)

        # T_total = wheelbase * (f_right - f_left)
        # F_total = f_right + f_left
        # (using in the evaluate of TwoWheelsCart2D)
        f_right = (F + T / self.wheelbase) / 2.0
        f_left  = (F - T / self.wheelbase) / 2.0
        
        return (f_left, f_right)

class PositionController:
    """
    - Virtual Robot (to simulate next position with acc, dec, threshold)
    - Polar2DController (Takes (x_T, y_T) from virtual robot and the feedback pose 
      to control using PID+sat with param: kp_lin, kp_ang, v_max, w_max)
    """

    def __init__(self, robot, acc, dec, threshold, kp_lin, kp_ang, v_max, w_max):
        """
        :param robot:     model

        VIRTUAL ROBOT:
        :param acc:       acceleration VR
        :param dec:       deceleration VR
        :param threshold: arrival distance to switch waypoint

        POS CONTROL:
        :param kp_lin:    proportional gain for distance error
        :param kp_ang:    proportional gain for heading error
        :param v_max:     max linear speed (saturation)
        :param w_max:     max angular speed (saturation)
        """
        
        # Virtual Robot:
        self.robot = robot
        self.path = Path2D(v_max, acc, dec, threshold)
        self.polar = Polar2DController(kp_lin, v_max, kp_ang, w_max)
        
        # State
        self.active = False

    def start_path(self, start_pose, waypoints):
        self.path.set_path(waypoints)
        self.path.start((start_pose[0], start_pose[1]))
        self.active = True

    def evaluate(self, delta_t):
        if not self.active:
            return (0.0, 0.0)

        current_pose = (self.robot.x, self.robot.y, self.robot.theta)

        target = self.path.evaluate(delta_t, current_pose)

        if target is None:
            self.active = False
            return (0.0, 0.0)
        
        (x_T, y_T) = target
        (v_ref, w_ref) = self.polar.evaluate(delta_t, x_T, y_T, current_pose)

        return (v_ref, w_ref)