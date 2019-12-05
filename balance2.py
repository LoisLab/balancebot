#Connections
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)


#from Kalman import KalmanAngle
import RPi.GPIO as GPIO
import smbus			#import SMBus module of I2C
import time
import math
import numpy as np
import matplotlib.pyplot as plt

GPIO.setmode(GPIO.BOARD)

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


class Motor:
    
    FORWARD=(GPIO.HIGH,GPIO.LOW)
    REVERSE=(GPIO.LOW,GPIO.HIGH)
    
    def __init__(self, pins):
        self.pins=pins
        GPIO.setup(self.pins[0], GPIO.OUT)
        GPIO.setup(self.pins[1], GPIO.OUT)
        GPIO.setup(self.pins[2], GPIO.OUT)
        GPIO.output(self.pins[2], GPIO.LOW)
        self.pwm=GPIO.PWM(self.pins[2], 100)
        self.pwm.start(0)

    def set_direction(self, direction):
        GPIO.output(self.pins[0], direction[0])
        GPIO.output(self.pins[1], direction[1])
        
    def set_speed(self, speed):
        self.pwm.ChangeDutyCycle(speed)
        
    def go(self):
        GPIO.output(self.pins[2], GPIO.HIGH)
        
    def stop(self):
        GPIO.output(self.pins[2], GPIO.LOW)

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

	#Write to power management register
	bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

	#Write to Configuration register
	#Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
	bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))

	#Write to Gyro configuration register
	bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

	#Write to interrupt enable register
	bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(DeviceAddress, addr)
        low = bus.read_byte_data(DeviceAddress, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value



bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68   # MPU6050 device address

MPU_Init()

time.sleep(1)
#Read Accelerometer raw value
#gyroX = read_raw_data(GYRO_XOUT_H)
gyroY = read_raw_data(GYRO_YOUT_H)
#gyroZ = read_raw_data(GYRO_ZOUT_H)

'''P = 0.28
I = 0.05
D = 0.015'''

P = 0.35
I = 0.6
D = 0.015

'''P = 0.5
I = 0.000
D = 0.03
'''

prior_error = 0
cumulative_error = 0
t0 = time.time()

left_wheel = Motor((38,36,7))
right_wheel = Motor((11,13,15))
left_wheel.set_direction(left_wheel.FORWARD)
right_wheel.set_direction(right_wheel.REVERSE)
left_wheel.set_speed(100)
right_wheel.set_speed(100)
left_wheel.stop()
right_wheel.stop()

throttles = []
gyros = []

print("Start!")
theta = 0   # should use accel to get initial theta
while True:
	v_angular = read_raw_data(GYRO_YOUT_H) + 13
	t1 = time.time()
	dt = t1-t0
	theta += v_angular*dt
	error = theta
	
	gyros.append(error)
	#error2 = read_raw_data(ACCEL_XOUT_H)
	
	de_dt = (error - prior_error) / dt
	cumulative_error += error*dt
	throttle = P*error + I*cumulative_error + D*v_angular
	t0 = t1
	prior_error = error
	if throttle > 0:
		left_wheel.set_direction(left_wheel.REVERSE)
		right_wheel.set_direction(right_wheel.FORWARD)
	else:
		left_wheel.set_direction(left_wheel.FORWARD)
		right_wheel.set_direction(right_wheel.REVERSE)
	throttle = min(abs(throttle), 99)
	throttles.append(throttle)
	left_wheel.set_speed(abs(throttle))
	right_wheel.set_speed(abs(throttle))
	if abs(v_angular) > 10000:
		break
	
GPIO.cleanup()

print('gyro',len(gyros),np.max(gyros),np.min(gyros),np.mean(gyros),np.std(gyros))
print('throttles',len(throttles),np.max(throttles),np.min(throttles),np.mean(throttles),np.std(throttles))
plt.plot(throttles)
plt.savefig('throttles')
plt.plot(gyros)
plt.savefig('gyros')
