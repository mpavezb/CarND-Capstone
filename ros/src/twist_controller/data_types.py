class ControlParameters(object):
    def __init__(self):
        self.vehicle_mass = None
        self.fuel_capacity = None
        self.brake_deadband = None
        self.decel_limit = None
        self.accel_limit = None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None


class ControlCommand(object):
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0


class Velocity(object):
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular
