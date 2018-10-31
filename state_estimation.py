import spidev
import time
import argparse 
import sys
import navio.mpu9250
import navio.util
from  EKF import *

navio.util.check_apm()

imu = navio.mpu9250.MPU9250()
imu_count=0

if imu.testConnection():
    print "Connection established: True"
else:
	print "Connection established: False"
	
imu.initialize()
ekf = EKF()

while True:
	t=time.time()
	m9a, m9g, m9m = imu.getMotion9()
	acc, gyro = m9a, m9g
	print "acc raw data: ", acc, "|| gyro raw data:", gyro
	ekf.predict(gyro, acc, t)
	imu_count+=1
	#if imu_count%10==0:
	ekf.update(acc,t)
	#print("position: ", ekf.x[4:7], "velocity", ekf.x[7:10])
	print "---------------------------------------------------------------------------------"
	time.sleep(1)
