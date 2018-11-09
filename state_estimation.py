import spidev
import time
import argparse 
import sys
import navio.mpu9250
import navio.util
from  EKF2 import *

navio.util.check_apm()

imu = navio.mpu9250.MPU9250()
imu2 = navio.lsm9ds1.LSM9DS1()
imu_count=0

if imu2.testConnection():
    print ("Connection established: True")
else:
	print ("Connection established: False")
	
imu2.initialize()
ekf = EKF()
i = 1
A = np.array([0,0,0])
B = np.array([0,0,0])
bA = np.array([0.0,0.0,0.0])
bb = np.array([0.0,0.0,0.0])
while True:
	
	m9a, m9g, m9m = imu2.getMotion9()
	#m9a2,m9g2,m9m2 = imu2.getMotion9()
	acc, gyro = m9a, m9g
	#acc2, gyro2 = m9a2, m9g2
	#print "acc raw data: ", acc, "|| gyro raw data:", gyro
	#if i <= 10:
		#A = A + acc
		#B = B + gyro

	#else : 
		#if i ==11:
			#bA = A/10.0
			#bb = B/10.0
			#bA = bA -[0,0,9.8]
		#print bA,bb

	acc_=np.array([acc[1],acc[0],-acc[2]])
	gyro_=np.array([gyro[1],gyro[0],gyro[2]])
	#print "acc_: ", acc_
	t=time.time()
	ekf.predict(gyro_, acc_, t)
	imu_count+=1
	if imu_count%10==0:
		print("UPDATE_:")
		ekf.update(acc_, gyro_,t)
	#print("position: ", ekf.x[4:7], "velocity", ekf.x[7:10])
	print ("---------------------------------------------------------------------------------")
	#i = i + 1
	#time.sleep(0.5)
