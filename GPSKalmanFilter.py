"""
Estimate Position using a 4 dim Kalman filter,
X, Y, Xvel, Yvel

Based on Clyde McQueen's terrain_kf class
"""

import math
import filterpy
import filterpy.common
import filterpy.kalman
import numpy as np


class GpsKF:
    startTime = 0
    yawOffset = 0
    def __init__(self, startLat:float, startLong:float, startYaw:float, dt:float):
        # State is Latitude (observed), Longitude (observed), yaw (hidden), vx (hidden), vy(hidden), vYaw(hidden), ax(hidden), ay(hidden), aYaw(hidden)
        self.x = np.array([startLat, startLong, startYaw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Initial covariance
        self.P = np.diag([10000, 10000, .5, 5, 5, .5, 2, 2, .01])


        # Process noise
        self.Q = filterpy.common.Q_continuous_white_noise(dim=9, dt=dt)
        # print('Possible process noise:')
        # print(Q)

        # State transition function
        '''
        xPos = xPos + xVel * dt
        yPos = yPos + yVel * dt
        
        xVel = cos(yaw)*locXVel + sin(yaw)*locYVel
        yVel = cos(yaw)*locYVel - sin(yaw)*loc
        
        Lat, Long = metersToGPS(xpos, ypos)
        
        SEATTLE AQUARIUM CCR OFFICE is 47.607477320727824, -122.34279024772025
        
        1 degree of long = 75.0522385372km
        1 degree of lat = 111.32km
        
        Compared GPS against this(Mostly for fun and for ease of comparison)
        '''

        self.F = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],])

        # Measurement function
        self.H = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])

        # Measurement covariance
        self.R = np.array([[22500000, 0],
                           [0, 22500000]])

    def predict(self, dt = None):
        if dt != None:
            self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.x, self.P = filterpy.kalman.predict(self.x, self.P, F=self.F, Q=self.Q)

    def update(self, z:np.array(float)):
        self.x, self.P = filterpy.kalman.update(self.x, self.P, z, self.R, H=self.H)

    def project(self, steps: int):
        """
        The filter runs at t = the time of the last measurement.
        To simulate a sensor delay we project forward using a multiple of dt.
        The results of the projection do not affect the state.

        E.g., to simulate a sensor with a 0.8s delay, try:
            kf.predict()
            kf.update(z)
            curr_x, curr_P = kf.project(8)

        This projection can get wild... probably need some filters on the result.
        """
        x = self.x
        P = self.P
        for _ in range(steps):
            x, P = filterpy.kalman.predict(x, P, self.F)
        return x, P


    def metersToLat(self, m:float):
        return (m/6371000.0) * 180/math.pi

    def metersToLong(self, m: float):
        return (m / 6378137) * 180 / math.pi / math.cos(self.x[0]*math.pi/180)