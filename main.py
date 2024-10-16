import numpy as np
import csv
import matplotlib.pyplot as plt
import GPSKalmanFilter
import argparse

#Latitude is column 23, Long is

def readGPSLog(filename:str):
    time, latitude, longitude = [], [], []
    with open(filename, newline='') as GPSLog:
        GPSLogreader = csv.reader(GPSLog)
        row = next(GPSLogreader)
        for row in GPSLogreader:
            time.append(float(row[1]))
            latitude.append(float(row[10])) #23 is true lat, 10 is it represented as an integer
            longitude.append(float(row[11])) #24 is true long, 11 is it represented as an integer
    return np.array(time),np.array(latitude),np.array(longitude)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--GPS', default="", type=str, help='Name of GPS Log file')
    parser.add_argument('--log', type=str, default="", help='Name of File to Write to')
    args = parser.parse_args()
    if len(args.GPS)>0:
        t, zlat, zlong = readGPSLog(args.GPS)
    else:
        t, zlat, zlong = readGPSLog("2024-06-12 10-40-01_GPS_INPUT.csv")
    lats = []
    longs = []
    kf = GPSKalmanFilter.GpsKF(zlat[0], zlong[0],.22)
    prevTime = t[0]
    for ts, lat, long in zip(t, zlat, zlong):
        kf.predict(ts-prevTime)
        z=np.array([[lat,long]])
        kf.update(z)
        lats.append(kf.x[0])
        longs.append(kf.x[1])

    if len(args.log)==0:
        plt.plot(zlong,zlat)
        plt.plot(longs, lats)
        plt.show()
    else:
        with open(f'{args.log}.csv', mode='w', newline='') as outfile:
            datawriter = csv.writer(outfile)
            datawriter.writerow(['Timestamp, Latitude, Longitude'])
            for t,lat, long in zip(t, lats, longs):
                datawriter.writerow([t, lat, long])


if __name__ == '__main__':
    main()




