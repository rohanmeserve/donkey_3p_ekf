'''
Classes that allow more frequent position updates than the GPS alone
If using the EKF, needs the PositionEstimator to run before so it can use absolute accels

PositionEstimator calculates velocity and heading based on GPS readings, and if it has an IMU, then alters them with raw IMU readings (accel and gyro)

EKF is an Extended Kalman Filter that requires an IMU and some additional sensor measurements, but should produce more accurate pos predictions (accel only)

Written for
UCSD DSC190 WI/SP 2024
'''

import numpy as np
import math
import time

class PositionEstimator:
    '''
    A PositionEstimator that uses accelerometer and gyroscope readings of an imu to 
    predict position in-between readings from a gps, due to their difference in update
    frequencies. 

    Should be used unthreaded if there is not a significant difference
    between the gps update frequency and donkey loop frequency.

    Should be used threaded only when testing estimator effectiveness;
    the estimated position is only useful when the autopilot (an unthreaded part)
    needs to calculate error (CTE, also unthreaded) for steering values.

    In order to allow the autopilot to use the estimations at a higher frequency,
    the donkey loop frequency needs to be increased.
    '''

    def __init__(self):
        # position, velocity, and orientation values
        self.x = 0.
        self.y = 0.
        self.vx = 0.
        self.vy = 0.
        self.yaw = 0.

        # last gps values received
        self.last_pos_x = 0.
        self.last_pos_y = 0.

        # timestamps for calculating time difference
        self.last_time = time.time()
        self.last_gps_time = time.time()

        self.start_time = time.time()

        # create file for analysis purposes
        self.file = open('estimations.csv', 'w')
        # write column headers
        self.file.write('(est_x, est_y), (act_x, act_y), timestamp, yaw\n')


    def run(self, acl_x: float, acl_y: float, gyr_x: float, pos_x: float, pos_y: float):
        # updates stored velocity, orientation, and position using accelerometer and gyroscope readings

        curr_time = time.time()
        dt = curr_time - self.last_time

        # if IMU inputs are None (no IMU), then set to 0
        # this will essentially just project the position
        # based on last known positions and time elapsed
        if gyr_x is None:
            gyr_x = 0
        if acl_x is None:
            acl_x = 0
        if acly_y is None:
            acl_y = 0

        # update orientation
        self.yaw += float(gyr_x) * dt

        # lock yaw to 0->360 frame
        if self.yaw < 0:
            # handle negative case; -1deg = 359deg
            self.yaw += 2*math.pi
        self.yaw %= 2*math.pi


        # rotate accerlation vectors; apply trig then adjust signage according to heading
        ax = float(acl_x) * math.cos(self.yaw) - float(acl_y) * math.sin(self.yaw)
        ay = float(acl_x) * math.sin(self.yaw) - float(acl_y) * math.cos(self.yaw)

        # update velocity
        self.vx += ax * dt
        self.vy += ay * dt

        # update stored position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # calculate total velocity
        velocity_total = math.sqrt(self.vx**2 + self.vy**2)

        # update time for next run
        self.last_time = curr_time

        # checks if the pos/x and pos/y from gps have recently been updated
        # if true, reset stored position and velocity to match actual values
        if pos_x != self.last_pos_x or pos_y != self.last_pos_y:
            # calculate time difference since last gps update
            gps_dt = curr_time - self.last_gps_time
            # reset position based on gps values
            pos_x = float(pos_x)
            pos_y = float(pos_y)
            self.x = pos_x
            self.y = pos_y
            # reset velocity based on gps delta
            self.vx = (pos_x - self.last_pos_x) / gps_dt
            self.vy = (pos_y - self.last_pos_y) / gps_dt

            # reset yaw
            if (pos_x - self.last_pos_x) > 0 and (pos_y - self.last_pos_y) > 0:
                # NE, 0 -> 90
                self.yaw = abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x)))
            elif (pos_x - self.last_pos_x) < 0  and (pos_y - self.last_pos_y) > 0:
                # NW, 90 - 180
                self.yaw = math.pi - abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x)))
            elif (pos_x - self.last_pos_x) < 0 and (pos_y - self.last_pos_y) < 0:
                # SW, 180 -> 270
                self.yaw = abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x))) + math.pi
            else:
                # SE, 270 -> 360
                self.yaw = (2*math.pi) - abs(math.atan((pos_y - self.last_pos_y) / (pos_x - self.last_pos_x)))
            # lock to 0 -> 360 frame
            self.yaw %= 2*math.pi
            self.last_gps_time = curr_time

        # updates last known gps position to current gps position
        self.last_pos_x = pos_x
        self.last_pos_y = pos_y

        # (imu-adjusted x,y), (gps-raw x,y), timestamp, yaw
        self.file.write(f'({self.x},{self.y}), ({pos_x}, {pos_y}), {curr_time - self.start_time}, {self.yaw}\n')

        # returns estimated x, estimated y, estimated orientation, absolute x accerlation, absolute y acceration, and total velocity
        return self.x, self.y, self.yaw, ax, ay, velocity_total


class kalmanFilter():
    '''
    Kalman Filter class fuses the data from GPS and IMU.
    The predict and update functions play the most vital role.
    '''
    def __init__(self, initPos, initVel, posStdDev, accStdDev, currTime):
        # set these values from the arguments received
        # current state ; assumes car is at origin (0,0) and at rest
        self.X = np.array([[np.float64(initPos)], [np.float64(initVel)]])
        # Identity matrix
        self.I = np.identity(2)
        # Initial guess for covariance
        self.P = np.identity(2)
        # transformation matrix for input data
        self.H = np.identity(2)
        # process (accelerometer) error variance
        self.Q = np.array([[accStdDev * accStdDev, 0], [0, accStdDev * accStdDev]])
        # measurement (GPS) error variance
        self.R = np.array([[posStdDev * posStdDev, 0], [0, posStdDev * posStdDev]])
        # current time
        self.currStateTime = currTime
        # self.A = defined in predict
        # self.B = defined in predict
        # self.u = defined in predict
        # self.z = defined in update

        # for heading calculation
        self.last_pred_x = 0
        self.last_pred_y = 0

    # main functions
    def predict(self, accThisAxis, timeNow):
        '''
        Predict function perform the initial matrix multiplications.
        Objective is to predict current state and compute P matrix.
        '''
        deltaT = timeNow - self.currStateTime
        self.B = np.array([[0.5 * deltaT * deltaT], [deltaT]])
        self.A = np.array([[1.0, deltaT], [0.0, 1.0]])
        self.u = np.array([[accThisAxis]])

        self.X = np.add(np.matmul(self.A, self.X), np.matmul(self.B, self.u))
        self.P = np.add(np.matmul(np.matmul(self.A, self.P), np.transpose(self.A)), self.Q)
        self.currStateTime = timeNow

    def update(self, pos, velThisAxis, posError, velError):
        '''
        Update function performs the update when the GPS data has been
        received. 
        '''
        self.z = np.array([[pos], [velThisAxis]])
        if(not posError):
            self.R[0, 0] = posError * posError
        else:
            self.R[1, 1] = velError * velError
        y = np.subtract(self.z, self.X)
        s = np.add(self.P, self.R)
        try:
            sInverse = np.linalg.inv(s)
        except np.linalg.LinAlgError:
            print("Matrix is not invertible")
            pass
        else:
            K = np.matmul(self.P, sInverse)
            self.X = np.add(self.X, np.matmul(K, y))
            self.P = np.matmul(np.subtract(self.I, K), self.P)

    def getPredictedPos(self):
        '''
        Returns predicted position in that axis.
        '''

        return self.X[0, 0]

    def getPredictedVel(self):
        '''
        Returns predicted velocity in that axis.
        '''

        return self.X[1, 0]

### SENSOR FUSION PART 
ACTUAL_GRAVITY = 9.80665

class GPS_IMU_EKF:
    '''
    An Extended Kalman Filter which takes in positions and accelerometer/gyroscope data and outputs a more accurate position estimate.

    NOTE: This code is based on Sarthak Mahajan's implementation of an IMU/GPS EKF, found at https://github.com/smahajan07/sensor-fusion/tree/master
    this file was created for the purpose of attempting to integrate their EKF implementation into the DonkeyCar GPS path-follow system.
    main changes are adapting the structure to fit Donkey's part objects, 
    and modifying GPS inputs from latitude/longitude to x/y in meters (NMEA parsing and conversion is handled within Donkey).
    '''

    def __init__(self, gpsStdDev, acceleroStdDev):
        # # setting some constants
        self.start_time = time.time()
        self.last_time = time.time()

        # longitude = x axis, latitude = y axis

        # set standard deviations
        gpsStdDev = gpsStdDev
        accEastStdDev = acceleroStdDev
        accNorthStdDev = acceleroStdDev

        # create objects of kalman filter; ASSUMING THAT START IS AT 0,0 AND STATIONARY
        self.objEast = kalmanFilter(0, 0, gpsStdDev, accEastStdDev, self.start_time)

        self.objNorth = kalmanFilter(0, 0, gpsStdDev, accNorthStdDev, self.start_time)
        
        self.last_pos_x = 0
        self.last_pos_y = 0

        # create file for analysis purposes
        self.file = open('ekf.csv', 'w')
        # write column headers
        self.file.write('(filtered_x, filtered_y), (gps_x, gps_y), timestamp\n')


    def run(self, pos_x, pos_y, acl_x, acl_y, hdop):
        curr_time = time.time()
        timestamp = curr_time - self.start_time
        dt = curr_time - self.last_time
        # {timestamp, lat, lon, alt, pitch, yaw, roll, north_accel, east_accel, up_accel, vel_north, vel_east, vel_down, vel_error, alt_error}
        # call the predict function for all objects
        self.objEast.predict(acl_x, dt)
        self.objNorth.predict(acl_y, dt)

        # if GPS data has changed since last, new data has been received; run update functions
        if(pos_x != self.last_pos_x or pos_y != self.last_pos_y):

            defPosErr = 0.0

            # call the update function for all objects
            vEast = (pos_x - self.last_pos_x) / dt
            self.objEast.update(pos_x, vEast, defPosErr, hdop)

            vNorth = (pos_y - self.last_pos_y) / dt
            self.objNorth.update(pos_y, vNorth, defPosErr, hdop)

        # get predicted values
        pred_x = self.objEast.getPredictedPos()
        pred_y = self.objNorth.getPredictedPos()

        pred_vx = self.objEast.getPredictedVel()
        pred_vy = self.objNorth.getPredictedVel()

        resultantV = np.sqrt(np.power(pred_vx, 2) + np.power(pred_vy, 2))

        # print("{} seconds in, Lat: {}, Lon: {}, Alt: {}, Vel(mph): {}".format(
        #         deltaT, predictedLat, predictedLon, predictedAlt, resultantV))
        # (predictions), (gps-actual), timestamp
        self.file.write(f'({pred_x}, {pred_y}), ({pos_x}, {pos_y}), {timestamp}\n')
        self.last_time = curr_time

        # recalculate heading given last outputted position and new
        if (pred_x - self.last_pred_x) > 0 and (pred_y - self.last_pred_y) > 0:
            # NE, 0 -> 90
            heading = abs(math.atan((pred_y - self.last_pred_y) / (pred_x - self.last_pred_x)))
        elif (pred_x - self.last_pred_x) < 0  and (pred_y - self.last_pred_y) > 0:
            # NW, 90 - 180
            heading = math.pi - abs(math.atan((pred_y - self.last_pred_y) / (pred_x - self.last_pred_x)))
        elif (pred_x - self.last_pred_x) < 0 and (pred_y - self.last_pred_y) < 0:
            # SW, 180 -> 270
            heading = abs(math.atan((pred_y - self.last_pred_y) / (pred_x - self.last_pred_x))) + math.pi
        else:
            # SE, 270 -> 360
            heading = (2*math.pi) - abs(math.atan((pred_y - self.last_pred_y) / (pred_x - self.last_pred_x)))
        # lock to 0 -> 360 frame
        heading %= 2*math.pi
        self.last_pred_x = pred_x
        self.last_pred_y = pred_y

        return pred_x, pred_y, heading