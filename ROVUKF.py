import math
import filterpy.common
import filterpy.kalman
from filterpy.kalman import UnscentedKalmanFilter as UKF
import numpy as np
import pandas
import matplotlib.pyplot as plt
from scipy.linalg import block_diag


def readGPSLog(filename: str):
    return pandas.read_csv(filename, usecols=["timestamp", "GPS_INPUT.lat_deg", "GPS_INPUT.lon_deg",
                                              "GPS_INPUT.yaw"])


def readDVLLog(filename: str):
    return pandas.read_csv(filename, usecols=["timestamp", "VISION_POSITION_DELTA.x_delta",
                                              "VISION_POSITION_DELTA.y_delta", "VISION_POSITION_DELTA.yaw_delta"])


def readAttitudeFile(filename: str):
    return pandas.read_csv(filename, usecols=["timestamp", "ATTITUDE.yaw", "ATTITUDE.yawspeed"])


# 0,0 = seattle aquarium
# posy = (latm - lataquarium)*pi/180
# posx = (longm - longaquarium) * pi/180 * cos(lataquarium)
def latLongToPos(lat: float, long: float):
    aquariumPos = (47.607477320727824, -122.34279024772025)
    xPos = (long - aquariumPos[1]) * math.pi / 180 * math.cos(aquariumPos[0] * math.pi / 180) * 6371.000 * 1000
    yPos = (lat - aquariumPos[0]) * math.pi / 180 * 6371.000 * 1000

    return xPos, yPos


def PosToLatLong(x: float, y: float):
    aquariumPos = (47.607477320727824, -122.34279024772025)
    long = ((x/6371000 * 180 / math.pi) / math.cos(aquariumPos[0] * math.pi / 180)) + aquariumPos[1]
    lat = (y/6371000 * 180 / math.pi) + aquariumPos[0]

    return lat, long


def mov(x: np.array, dt):
    return x + np.array([dt * x[1], 0,
                         dt * x[3], 0,
                         (dt * x[5]) % (math.pi * 2), 0])


def normalize_angle(x):# Taken from Kalman Filter textbook
    x = x % (2 * np.pi)  # force in range [0, 2 pi)
    # if x > np.pi:  # move to [-pi, pi)
    #     x -= 2 * np.pi
    return x


def hxGPS(x):
    return np.array(PosToLatLong(x[0], x[2]))

def hxDVL(x, dt):
    xVel = x[1]
    yVel = x[3]
    yawVel = x[5]
    yaw = x[4]

    xVM = xVel*math.cos(yaw) + yVel*math.sin(yaw)
    yVM = xVel*-math.sin(yaw) + yVel*math.cos(yaw)

    return np.array([xVM*dt, yVM*dt])

def hxYaw(x, dt):
    yawVel = x[5]
    yaw = x[4]

    return np.array(normalize_angle(yaw - yawVel*dt))

def state_mean(sigmas, Wm):
    x = np.zeros(6)
    sum_sin, sum_cos = 0., 0.
    for i in range(len(sigmas)):
        s = sigmas[i]
        x[0] += s[0] * Wm[i]
        x[1] += s[1] * Wm[i]
        x[2] += s[2] * Wm[i]
        x[3] += s[3] * Wm[i]
        x[5] += s[5] * Wm[i]
        sum_sin += np.sin(s[4]) * Wm[i]
        sum_cos += np.cos(s[4]) * Wm[i]
    x[4] = np.atan2(sum_sin, sum_cos)
    return x

def z_yaw_mean(sigmas, Wm):
    sum_sin = 0.0
    sum_cos = 0.0
    for i in range(len(sigmas)):
        s = sigmas[i]
        w = Wm[i]
        sum_sin+=np.sin(s)*w
        sum_cos+=np.cos(s)*w
    print(sigmas)
    return [np.arctan2(sum_sin, sum_cos)]


def residual_x(a, b):
    y = a - b
    y[4] = normalize_angle(y[4])
    return y

def residual_z(a, b):
    y = a - b
    y = [normalize_angle(y)]
    return y

def findStartYaw(GPSLog, index):
    line = np.poly1d(np.polyfit(x=GPSLog['xPos'][index:index+50], y=GPSLog['yPos'][index:index+50], deg=1))
    yaw = np.arctan2(line(1)-line(0), 1)
    if GPSLog['yPos'][index] < GPSLog['yPos'][index+50]:
        yaw += np.pi
    return normalize_angle(yaw)


if __name__ == '__main__':
    StateLog = []
    GPSlog = readGPSLog("2024-06-12 10-40-01_GPS_INPUT.csv")
    DVLlog = readDVLLog("2024-06-12 10-40-01_VISION_POSITION_DELTA.csv")
    GPSx, GPSy = latLongToPos(GPSlog['GPS_INPUT.lat_deg'],GPSlog['GPS_INPUT.lon_deg'])
    gpsR = np.diag([16, 16])
    dvlR = np.diag([.0003**2, .0003**2])
    yawR = .002**2

    GPSlog.insert(len(GPSlog.columns), 'xPos', GPSx)
    GPSlog.insert(len(GPSlog.columns), 'yPos', GPSy)

    sigma = filterpy.kalman.MerweScaledSigmaPoints(n=6, alpha=.1, beta=2, kappa=-4)

    ROVUKF = filterpy.kalman.UnscentedKalmanFilter(dim_x=6, dim_z=2, dt=.1, hx=hxGPS, fx=mov, points=sigma,
                                                   x_mean_fn=state_mean, residual_x=residual_x)
    GPSlog2 = GPSlog[GPSlog['timestamp']>DVLlog['timestamp'][0]]
    startX = GPSlog2.iloc[0]['xPos']
    startY = GPSlog2.iloc[0]['yPos']
    startYaw = findStartYaw(GPSlog, GPSlog2.index[0])

    ROVUKF.x = [GPSlog2['xPos'].iloc[0], 0, GPSlog2['yPos'].iloc[0], 0, startYaw, 0]
    # plt.plot(GPSlog['xPos'][770:1000], GPSlog['yPos'][770:1000])
    # plt.show()

    ROVUKF.P = np.diag([16, .25, 4, .25, np.pi/4, np.pi*np.pi])

    q = filterpy.common.Q_discrete_white_noise(dim=2, dt=.3, var=.04**2)
    ROVUKF.Q = block_diag(q, q, q)

    log = GPSlog.merge(DVLlog, how='outer',  on='timestamp')

    lastGPS = log['timestamp'][0]
    lastDVL = log['timestamp'][0]
    lastYaw = log['timestamp'][0]
    last = log['timestamp'][0]
    estPos = []
    for index, row in log.iterrows():
        dt=row['timestamp'] - last
        ROVUKF.predict(dt=dt)
        estPos.append([ROVUKF.x[0], ROVUKF.x[2]])
        last = row['timestamp']

        if np.isnan(row['xPos']):
            if row['VISION_POSITION_DELTA.yaw_delta']!=0:
                print('YAW')
                ROVUKF.residual_z = residual_z
                ROVUKF.z_mean = z_yaw_mean
                ROVUKF.update(z=row['VISION_POSITION_DELTA.yaw_delta'], hx=hxYaw, R=.002**2, dt=dt)
                ROVUKF.residual_z=None
                ROVUKF.z_mean=None
                lastYaw = row['timestamp']
            print('DVL')
            ROVUKF.update(z=[row['VISION_POSITION_DELTA.x_delta'],
                             row['VISION_POSITION_DELTA.y_delta']], hx=hxDVL, R=dvlR, dt=dt)
            lastDVL = row['timestamp']
        else:
            print('GPS')
            ROVUKF.update(z=[row['xPos'], row['yPos']], hx=hxGPS, R=gpsR)
            lastGPS = row['timestamp']

    estPos = np.array(estPos)
    plt.scatter(x=estPos[:, 0], y=estPos[:, 1])
    plt.show()





    print("Still under development! Check back later!")

