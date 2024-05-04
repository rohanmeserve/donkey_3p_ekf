#!/usr/bin/env python3
import time
SENSOR_MPU6050 = 'mpu6050'
SENSOR_MPU9250 = 'mpu9250'
SENSOR_OAKDPRO = 'oakd_BNO086'

DLP_SETTING_DISABLED = 0
CONFIG_REGISTER = 0x1A

class IMU:
    '''
    Installation:
    
    - MPU6050
    sudo apt install python3-smbus
    or
    sudo apt-get install i2c-tools libi2c-dev python-dev python3-dev
    git clone https://github.com/pimoroni/py-smbus.git
    cd py-smbus/library
    python setup.py build
    sudo python setup.py install

    pip install mpu6050-raspberrypi
    
    - MPU9250
    pip install mpu9250-jmdev
    
    '''

    def __init__(self, addr=0x68, poll_delay=0.0166, sensor=SENSOR_MPU6050, dlp_setting=DLP_SETTING_DISABLED):
        self.sensortype = sensor
        if self.sensortype == SENSOR_OAKDPRO:
            import depthai as dai
            import cv2
            import math

            # Create pipeline
            self.pipeline = dai.Pipeline()

            # Define sources and outputs
            self.imu = self.pipeline.create(dai.node.IMU)
            self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)

            self.xlinkOut.setStreamName("imu")

            # enable ACCELEROMETER_RAW at 500 hz rate
            self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
            # enable GYROSCOPE_RAW at 400 hz rate
            self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
            # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
            # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
            self.imu.setBatchReportThreshold(1)
            # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
            # if lower or equal to batchReportThreshold then the sending is always blocking on device
            # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
            self.imu.setMaxBatchReports(10)

            # Link plugins IMU -> XLINK
            self.imu.out.link(self.xlinkOut.input)

            self.device = dai.Device(self.pipeline)
            self.imuQueue = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            self.baseTs = None

            #####
            #####



        elif self.sensortype == SENSOR_MPU6050:
            from mpu6050 import mpu6050 as MPU6050
            self.sensor = MPU6050(addr)
        
            if(dlp_setting > 0):
                self.sensor.bus.write_byte_data(self.sensor.address, CONFIG_REGISTER, dlp_setting)
        
        else:
            from mpu9250_jmdev.registers import AK8963_ADDRESS, GFS_1000, AFS_4G, AK8963_BIT_16, AK8963_MODE_C100HZ
            from mpu9250_jmdev.mpu_9250 import MPU9250

            self.sensor = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=addr,  # In 0x68 Address
                address_mpu_slave=None,
                bus=1,
                gfs=GFS_1000,
                afs=AFS_4G,
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C100HZ)
            
            if(dlp_setting > 0):
                self.sensor.writeSlave(CONFIG_REGISTER, dlp_setting)
            self.sensor.calibrateMPU6500()
            self.sensor.configure()

        
        self.accel = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.gyro = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.mag = {'x': 0., 'y': 0., 'z': 0.}
        self.temp = 0.
        self.poll_delay = poll_delay
        self.on = True

    def update(self):
        while self.on:
            self.poll()
            time.sleep(self.poll_delay)
                
    def poll(self):
        try:
            if self.sensortype == SENSOR_OAKDPRO:
                imuData = self.imuQueue.get()  # blocking call, will wait until a new data has arrived

                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    acceleroValues = imuPacket.acceleroMeter
                    gyroValues = imuPacket.gyroscope

                    acceleroTs = acceleroValues.getTimestampDevice()
                    gyroTs = gyroValues.getTimestampDevice()
                    if self.baseTs is None:
                        self.baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
                    acceleroTs = (acceleroTs - self.baseTs).total_seconds()*1000
                    gyroTs = (gyroTs - self.baseTs).total_seconds()*1000

                    imuF = "{:.06f}"
                    tsF  = "{:.03f}"

                    # note that the OAK-D Pro's axes need to be rotated
                    self.accel = { 'x' : imuF.format(acceleroValues.z), 'y' : imuF.format(acceleroValues.x + 9.81), 'z' : imuF.format(acceleroValues.y)}
                    self.gyro = { 'x' : imuF.format(gyroValues.z), 'y' : imuF.format(gyroValues.y), 'z' : imuF.format(gyroValues.x * -1)}
            elif self.sensortype == SENSOR_MPU6050:
                self.accel, self.gyro, self.temp = self.sensor.get_all_data()
            else:
                from mpu9250_jmdev.registers import GRAVITY
                ret = self.sensor.getAllData()
                self.accel = { 'x' : ret[1] * GRAVITY, 'y' : ret[2] * GRAVITY, 'z' : ret[3] * GRAVITY }
                self.gyro = { 'x' : ret[4], 'y' : ret[5], 'z' : ret[6] }
                self.mag = { 'x' : ret[13], 'y' : ret[14], 'z' : ret[15] }
                self.temp = ret[16]
        except:
            print('failed to read imu!!')
            
    def run_threaded(self):
        return self.accel['x'], self.accel['y'], self.accel['z'], self.gyro['x'], self.gyro['y'], self.gyro['z'], self.temp

    def run(self):
        self.poll()
        return self.accel['x'], self.accel['y'], self.accel['z'], self.gyro['x'], self.gyro['y'], self.gyro['z'], self.temp

    def shutdown(self):
        self.on = False

    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000


if __name__ == "__main__":
    iter = 0
    import sys
    sensor_type = SENSOR_OAKDPRO
    dlp_setting = DLP_SETTING_DISABLED
    if len(sys.argv) > 1:
        sensor_type = sys.argv[1]
    if len(sys.argv) > 2:
        dlp_setting = int(sys.argv[2])

    p   = IMU(sensor=sensor_type)
    x_tot = 0
    y_tot = 0
    z_tot = 0
    while True:
        data = p.run()
        x_tot += float(data[0])
        y_tot += float(data[1])
        z_tot += float(data[2])
        print(data)
        time.sleep(0.01)
        #iter += 1
    # print bias per axis
    print(f'x_bias = {x_tot/10000}')
    print(f'y_bias = {y_tot/10000}')
    print(f'z_bias = {z_tot/10000}')
    