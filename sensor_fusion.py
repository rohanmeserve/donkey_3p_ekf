# IMPORTANT NOTE: This code is based on Sarthak Mahajan's implementation of an IMU/GPS EKF, found at https://github.com/smahajan07/sensor-fusion/tree/master
# this file was created for the purpose of attempting to integrate their EKF implementation into the DonkeyCar GPS path-follow system.
# main changes are adapting the structure to fit Donkey's part objects, 
# and modifying GPS inputs from latitude/longitude to x/y in meters (NMEA parsing and conversion is handled within Donkey).
# altogether, this file is largerly a copy/paste from the github.

### IMPORTS $$$

import numpy as np
import time

class kalmanFilter():
    '''
    Kalman Filter class fuses the data from GPS and IMU.
    The predict and update functions play the most vital role.
    '''
    def __init__(self, initPos, initVel, posStdDev, accStdDev, currTime):
        # set these values from the arguments received
        # current state
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

    IMPORTANT NOTE: This code is based on Sarthak Mahajan's implementation of an IMU/GPS EKF, found at https://github.com/smahajan07/sensor-fusion/tree/master
    this file was created for the purpose of attempting to integrate their EKF implementation into the DonkeyCar GPS path-follow system.
    main changes are adapting the structure to fit Donkey's part objects, 
    and modifying GPS inputs from latitude/longitude to x/y in meters (NMEA parsing and conversion is handled within Donkey).
    '''

    def __init__(self):
        # # setting some constants
        self.start_time = time.time()
        self.last_time = time.time()

        # longitude = x axis, latitude = y axis

        # set standard deviations
        gpsStdDev = 2.0
        accEastStdDev = ACTUAL_GRAVITY * 0.033436506994600976
        accNorthStdDev = ACTUAL_GRAVITY * 0.05355371135598354

        # create objects of kalman filter; ASSUMING THAT START IS AT 0,0 AND STATIONARY
        self.objEast = kalmanFilter(0, 0, gpsStdDev, accEastStdDev, self.start_time)

        self.objNorth = kalmanFilter(0, 0, gpsStdDev, accNorthStdDev, self.start_time)
        
        self.last_pos_x = 0
        self.last_pos_y = 0

        # create file for analysis purposes
        self.file = open('ekf.csv', 'w')
        # write column headers
        self.file.write('(filtered_x, filtered_y), (gps_x, gps_y), timestamp\n')


    # TODO: determine which inputs can be received from other Donkey parts, and which ones need to be calculated here
    # TODO: need to figure out where how the error (v_error) is calculated, and what purpose it serves
    def run(self, pos_x, pos_y, acl_x, acl_y, v_error):
        curr_time = time.time()
        timestamp = curr_time - self.start_time
        dt = curr_time - self.last_time
        # {timestamp, lat, lon, alt, pitch, yaw, roll, north_accel, east_accel, up_accel, vel_north, vel_east, vel_down, vel_error, alt_error}
        # call the predict function for all objects 
        # (since we already have the first reading, we call call predict)
        # TODO: figure out why ACTUAL_GRAVITY is used; might not be necessary in our implementation
        self.objEast.predict(acl_x * ACTUAL_GRAVITY, dt)
        self.objNorth.predict(acl_y * ACTUAL_GRAVITY, dt)

        # if GPS data has changed since last, new data has been received; run update functions
        if(pos_x != self.last_pos_x or pos_y != self.last_pos_y):

            defPosErr = 0.0

            # call the update function for all objects
            vEast = (pos_x - self.last_pos_x) / dt
            self.objEast.update(pos_x, vEast, defPosErr, v_error)

            vNorth = (pos_y - self.last_pos_y) / dt
            self.objNorth.update(pos_y, vNorth, defPosErr, v_error)

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

        return pred_x, pred_y
      

if __name__ == "__main__":
    # write stuff to test here
    print('running main; not doing anything')