import spidev
import time
import argparse 
import sys
import navio.mpu9250
import navio.util
import EKF

navio.util.check_apm()

imu = navio.mpu9250.MPU9250()
imu_count=0

if imu.testConnection():
    print "Connection established: True"
else:
	print "Connection established: False"
	
imu.initialize()

while True:
	t=time.time()
	m9a, m9g, m9m = imu.getMotion9()
	acc, gyro = m9a, m9g

	EKF.predict(gyro, acc, t)
	imu_count+=1
	if imu_count%10==0:
		EKF.update(acc,t)
	print("position: ", x[4:7], "velocity", x[7:10], "bias_acc", x[10:13], "bias_gyro", x[13:16])
