import math
import filterpy.common
import filterpy.kalman
import filterpy.stats
from filterpy.kalman import UnscentedKalmanFilter as UKF
import numpy as np
import pandas
import matplotlib.pyplot as plt
from scipy.linalg import block_diag


def readGPSLog(filename: str):
    GPSlog = pandas.read_csv(filename, usecols=["timestamp", "GPS_INPUT.lat_deg", "GPS_INPUT.lon_deg", "GPS_INPUT.yaw"])
    GPSx, GPSy = latLongToPos(GPSlog['GPS_INPUT.lat_deg'],GPSlog['GPS_INPUT.lon_deg'])
    GPSlog.insert(len(GPSlog.columns), 'xPos', GPSx)
    GPSlog.insert(len(GPSlog.columns), 'yPos', GPSy)
    return GPSlog

def readDVLLog(filename: str):
    DVLlog = pandas.read_csv(filename, usecols=["timestamp", "VISION_POSITION_DELTA.x", "VISION_POSITION_DELTA.y",
                                                "VISION_POSITION_DELTA.z", "VISION_POSITION_DELTA.yaw",
                                                "VISION_POSITION_DELTA.yaw_delta", "VISION_POSITION_DELTA.x_delta",
                                                "VISION_POSITION_DELTA.y_delta", "VISION_POSITION_DELTA.z_delta"])
    return DVLlog

def combineLog(DVL, GPS):
    return pandas.merge(DVL, GPS, how="outer", on="timestamp")

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
                         dt * x[3], 0])


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
    x = np.zeros(4)
    sum_sin, sum_cos = 0., 0.
    for i in range(len(sigmas)):
        s = sigmas[i]
        x[0] += s[0] * Wm[i]
        x[1] += s[1] * Wm[i]
        x[2] += s[2] * Wm[i]
        x[3] += s[3] * Wm[i]
    return x

def z_yaw_mean(sigmas, Wm):
    sum_sin = 0.0
    sum_cos = 0.0
    for i in range(len(sigmas)):
        s = sigmas[i]
        w = Wm[i]
        sum_sin+=np.sin(s)*w
        sum_cos+=np.cos(s)*w
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

def dataGate(UKF:filterpy.kalman.UnscentedKalmanFilter, position):
    # print("Sensor = ", [position[0].item(), UKF.x[1].item(), position[1].item(), UKF.x[3].item()])
    # print("Estimate = ", UKF.x)
    # print(UKF.P)
    x=np.array([position[0].item(), UKF.x[1].item(), position[1].item(), UKF.x[3].item()])
    mean = UKF.x
    output = filterpy.stats.mahalanobis(x=x, mean=mean, cov=UKF.P)
    return(output)

def createUKF(GPSlog):
    sigma = filterpy.kalman.MerweScaledSigmaPoints(n=4, alpha=.1, beta=2, kappa=-1)
    ROVUKF = filterpy.kalman.UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=.1, hx=hxGPS, fx=mov, points=sigma)
    q = filterpy.common.Q_discrete_white_noise(dim=2, dt=.3, var=3 ** 2)
    ROVUKF.Q = block_diag(q, q)
    ROVUKF.P = np.diag([25, .5, 25, .5])
    ROVUKF.R = np.diag([1, 1])
    startX = np.average(GPSlog['xPos'].iloc[0:5])
    startY = np.average(GPSlog['yPos'].iloc[0:5])
    ROVUKF.x = [startX, 0, startY, 0]
    return ROVUKF

def runUKF(GPSlog, UKF):
    last = GPSlog['timestamp'].iloc[0]
    estPos = []
    for index, row in GPSlog.iterrows():
        dt = row['timestamp'] - last
        UKF.predict(dt=dt)
        estPos.append([UKF.x[0], UKF.x[2]])
        last = row['timestamp']
        UKF.update(z=[row['GPS_INPUT.lat_deg'], row['GPS_INPUT.lon_deg']])
    return estPos

if __name__ == '__main__':

    deeplog = readGPSLog("2024-10-08_Deep1_GPS_INPUT.csv")
    deepDVL = readDVLLog("2024-10-08_Deep1_VISION_POSITION_DELTA.csv")
    comboLog = combineLog(deepDVL, deeplog)
    pandas.set_option('display.max_columns', 500)
    print(comboLog["VISION_POSITION_DELTA.yaw_delta"][0:10])

   # ROVUKF = createUKF(deeplog)


    # estPos = np.array(runUKF(Deeplog,ROVUKF))
    # print(estPos)
    #
    # plt.plot(Deeplog['xPos'],Deeplog['yPos'])
    # plt.plot(estPos[:,0],estPos[:,1])
    # plt.show()




    print("Still under development! Check back later!")

