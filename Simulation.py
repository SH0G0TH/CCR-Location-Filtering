import math


class Sim:
    def __init__(self, tran_len=500.0, tran_rad=0.0, rob_vel=0.5, time_grain=.1, process_noise=0, measurement_noise=0,starting_position=(0,0)):
        self.tran_len = tran_len  # The length of the transect line the robot follow in m
        self.tran_rad = tran_rad  # The curve of the line the robot follows, measured in radian.
        self.rob_vel = rob_vel  # Velocity of the robot in m/s
        self.time_grain = time_grain  # How often the simulation will report the position of the robot.
        self.process_noise = process_noise  # How noisy the movement of the ROV is
        self.measurement_noise = measurement_noise  # How noisy the measurement signals of the robot are
        self.position = starting_position
        self.yaw = math.pi
        self.timestamp = 0.0

    def simulate(self):
        idealPath = []
        truePath = []
        measuredPath = []
        distanceTraveled = 0





