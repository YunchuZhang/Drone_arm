import spidev
import time
import argparse 
import sys
import navio.mpu9250
import navio.util
from  EKF2 import *
from collections import deque
import zmq
import json
import numpy as np
from manipulation2 import *
savet = (0.0,0.0,0.0)
savet2 = (0.0,0.0,0.0)
savepose = deque(maxlen=2)
saveangu = deque(maxlen=2)
sub_port = 5556
context = zmq.Context()
#connect to socket we subscrib
socket_sub = context.socket(zmq.SUB)
#socket_sub.connect("tcp://localhost:5556")
socket_sub.connect("tcp://192.168.1.9:%d" %sub_port)
#socket_sub.setsockopt(zmq.SUBSCRIBE, b"")
socket_sub.setsockopt(zmq.SUBSCRIBE,b'')
def recv_array(socket, flags=0, copy=True, track=False):
	"""recv a numpy array"""
	md = socket.recv_json(flags=flags)
	msg = socket.recv(flags=flags, copy=copy, track=track)
	buf = buffer(msg)
	A = np.frombuffer(buf, dtype=md['dtype'])
	return A.reshape(md['shape'])



navio.util.check_apm()

imu = navio.mpu9250.MPU9250()
imu2 = navio.lsm9ds1.LSM9DS1()
imu_count=0

if imu.testConnection():
	print ("Connection established: True")
else:
	print ("Connection established: False")
	
imu.initialize()
ekf = EKF()
mpl=mpl()
i = 1
A = np.array([0,0,0])
B = np.array([0,0,0])
bA = np.array([0.0,0.0,0.0])
bb = np.array([0.0,0.0,0.0])

savepose.appendleft(savet)
savepose.appendleft(savet)
saveangu.appendleft(savet2)
saveangu.appendleft(savet2)

elapsed = 1.0
while True:
	start = time.clock()

	print('asds')
	contents  =  recv_array(socket_sub,copy=False)
	position = contents[0:3]
	a = contents[3]
	b = contents[4]
	c = contents[5]
	d = contents[6]
	orientation = np.array([a,b,c,d])
	orientation[0] *= -1.0
	print("orientation: ", orientation)

	Euler = mpl.quaternion2euler(orientation)
	print("Enler angle: ", Euler)
	savet2 = (Euler[0],Euler[1],Euler[2])
	saveangu.appendleft(savet2)

	Rotation_mat=np.dot(np.array([[0,0,1],[0,1,0],[-1,0,0]]),np.array([[0,1,0],[-1,0,0],[0,0,1.0]]))
	position = np.dot(Rotation_mat, position)
	position[2] += 0.22

	savet = (contents[0],contents[1],contents[2])
	savepose.appendleft(savet)
	print(contents)
		
	m9a, m9g, m9m = imu.getMotion9()
	#m9a2,m9g2,m9m2 = imu2.getMotion9()
	acc, gyro, mag = m9a, m9g, m9m

	acc_=np.array([-acc[0],-acc[1],-acc[2]])
	gyro_=np.array([gyro[0],gyro[1],gyro[2]])
	#print "acc_: ", acc_
	t=time.time()
	ekf.predict(gyro_, acc_, t)
	imu_count += 1

	
	if imu_count%10==0:
		#ekf.predict(gyro_, acc_, t)
		print("UPDATE_:")
		ekf.update(acc_, gyro_, position, vel, t)
	
	#print("position: ", ekf.x[4:7], "velocity", ekf.x[7:10])
	#time.sleep(0.2)
	elapsed = (time.clock() - start)
	print("time",elapsed)
	vx = (savepose[0][0] - savepose[-1][0])/elapsed*1.0
	vy = (savepose[0][1] - savepose[-1][1])/elapsed*1.0
	vz = (savepose[0][2] - savepose[-1][2])/elapsed*1.0
	vel = (vx,vy,vz)
	ax = (saveangu[0][0] - saveangu[-1][0])/elapsed*1.0
	ay = (saveangu[0][1] - saveangu[-1][1])/elapsed*1.0
	az = (saveangu[0][2] - saveangu[-1][2])/elapsed*1.0
	Angu = (ax,ay,az)
	print("Angu speed",Angu)# currently in optitrack coordinates
	print("speed",vel)# currently in optitrack coordinates
	print ("---------------------------------------------------------------------------------")


	
	 