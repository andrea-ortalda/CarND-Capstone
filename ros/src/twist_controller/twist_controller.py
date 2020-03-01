from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

#GAS_DENSITY = 2.858
#MPH_TO_MPS = 0.44704
MIN_SPEED = 0.1


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, accel_limit,
        wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # Yaw controller for steering angle
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel,
                                max_steer_angle)

        # PID controller for throttle
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0 # minimum throttle value
        mx = 0.2 # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # cut-off frequency
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_vel = None

        self.last_time = rospy.get_time()

    '''
    This method calculates the target commands for the ego vehicle
    '''
    def control(self, dbw_enabled, curr_vel, target_lin_vel, target_ang_vel):

        if not dbw_enabled:
            # Resetting the controller when dbw is not enabled to avoid random behavior
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # Applying low-pass filter to current ego vehicle speed signal
        curr_vel_filtered = self.vel_lpf.filt(curr_vel)

        # Getting steering command from yaw controller
        steering_cmd = self.yaw_controller.get_steering(target_lin_vel, target_ang_vel, curr_vel_filtered)

        # Getting throttle/brake commands from PID controller
        vel_error = target_lin_vel - curr_vel_filtered
        self.last_vel = curr_vel_filtered

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle_cmd = self.throttle_controller.step(vel_error, sample_time)
        brake_cmd = 0.0

        if target_lin_vel == 0.0 and curr_vel_filtered < MIN_SPEED:
            throttle_cmd = 0.0
            brake_cmd = 700 # N*m standstill torque

        elif throttle_cmd < 0.1 and vel_error < 0:
            throttle_cmd = 0.0
            target_decel = max(vel_error, self.decel_limit)
            brake_cmd = abs(target_decel)*self.vehicle_mass*self.wheel_radius # Torque in N*m

        return throttle_cmd, brake_cmd, steering_cmd
