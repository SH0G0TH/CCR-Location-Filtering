import math
import filterpy
import filterpy.common
import filterpy.kalman
import numpy as np
import pandas


def readGPSLog(filename: str):
    return pandas.read_csv(filename, usecols=["timestamp", "GPS_INPUT.lat_deg", "GPS_INPUT.long_deg",
                                              "GPS_INPUT.yaw"])


def readDVLLog(filename: str):
    return pandas.read_csv(filename, usecols=["timestamp", "VISION_POSITION_DELTA.x_delta",
                                              "VISION_POSITION_DELTA.x_delta", "VISION_POSITION_DELTA.yaw_delta"])


def readAttitudeFile(filename: str):
    return pandas.read_csv(filename, usecols=["timestamp", "ATTITUDE.yaw", "ATTITUDE.yawspeed"])


# 0,0 = seattle aquarium
# posy = (latm - lataquarium)*pi/180
# posx = (longm - longaquarium) * pi/180 * cos(lataquarium)
def latLongToPos(lat: float, long: float):
    aquariumPos = (47.607477320727824, -122.34279024772025)
    xPos = (long - aquariumPos[1]) * math.pi / 180 * math.cos(aquariumPos[0] * math.pi / 180)
    yPos = (lat - aquariumPos[0]) * math.pi / 180

    return xPos, yPos


def PosToLatLong(x: float, y: float):
    aquariumPos = (47.607477320727824, -122.34279024772025)
    long = ((x * 180 / math.pi) / math.cos(aquariumPos[0] * math.pi / 180)) + aquariumPos[1]
    lat = (y * 180 / math.pi) + aquariumPos[0]

    return lat, long


def mov(x: np.array, dt):
    return x + np.array(dt * x[1], 0, dt * x[3], 0, dt * (x[5] % (math.pi * 2)), 0)


def mov(x: np.array, dt):
    return x + np.array([dt * x[1], 0,
                         dt * x[3], 0,
                         (dt * x[5]) % (math.pi * 2), 0])

def normalize_angle(x):  # Taken from Kalman Filter textbook
    x = x % (2 * np.pi)  # force in range [0, 2 pi)
    if x > np.pi:  # move to [-pi, pi)
        x -= 2 * np.pi
    return x


class ROVUKF:
    def __init__(self, startX: float, startY: float, startYaw: float, dt: float):
        self.x = np.array([startX, 0.0, startY, 0.0, startYaw, 0.0])
        self.yawOffset = 0

    def hxGPS(self, x):
        return np.array(PosToLatLong(x[0], x[2]))

    def hxDVL(self, x):
        xVel = x[1]
        yVel = x[3]
        yawVel = x[5]
        yaw = x[4] - self.yawOffset

        xVM = xVel*math.cos(-yaw) + yVel*math.sin(-yaw)
        yVM = xVel*-math.sin(-yaw) + yVel*math.cos(-yaw)

        return np.array([xVM, yVM, yawVel])


    #    xVel = DVL.xvel cox(yaw) + DVL.yvel*sin(yaw)
    #    yVel = DVL.xVEL * -sin(yaw) + DVL.yvel*cos(yaw)


if __name__ == '__main__':
    print("Still under development! Check back later!")
