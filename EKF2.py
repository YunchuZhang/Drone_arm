import numpy as np
import math
import os
import random as random
#from numpy import quaternion
import matplotlib as mplt
from manipulation2 import *

GRAVITY=np.array([0,0,-9.8])
geo_mag_field=np.array([24.1774,0,39.9646])#[Mx,0,Mz] the coordinate is x pointing forward and y is pointing right with z pointing down.
#the unit of geo magnetic field is mT. 
Rotation_mat=np.dot(np.array([[0,-1.0,0],[1.0,0,0],[0,0,1.0]]),np.array([[1.0,0,0],[0,-1.0,0],[0,0,-1.0]]))

mpl = mpl()

class EKF:
	x=np.zeros(19)#16 states q(4) omegas(3) p(3) v(3) bw(3) ba(3)
	xdot=np.zeros(19)#10 states derivaties
	z=np.zeros(12)#real raw data from sensor 9dof acc gyro mag
	zhat=np.zeros(12)#H*x_bar
	P=np.eye(19)#covariance matrix
	Q=np.zeros([12,12])#process noise covariance gyro_cov bw_cov ba_cov bm_cov
	F=np.zeros((19,19))#state transition
	G=np.zeros((19,12))
	H=np.zeros((12,19))#observation Matrix
	R=np.eye(6)#observation noise Matrix gravity_cov gyro_cov mag_cov
	gyro_cov = 0.0025
	acc_cov = 0.5
	mag_cov = 0.5

	lamda=0.0
	bw_cov=0.0000001
	ba_cov=0.0000001
	bm_cov=0.0000001

	gravity_cov=0.0025
	current_t=0

	initialized = False
	imu_initialized = False
	magnetic_initialized = False
	acc=np.zeros(3)
	gyro=np.zeros(3)
	mag=np.zeros(3)
	#******************#

	def __init__(self):
		initialized = False
		self.x[0]=1
		self.Q[0:3,0:3] = np.eye(3)*self.gyro_cov
		self.Q[3:6,3:6] = np.eye(3)*self.bw_cov
		self.Q[6:9,6:9] = np.eye(3)*self.ba_cov
		self.Q[9:12,9:12] = np.eye(3)*self.gravity_cov
		#self.Q[9:12,9:12] = np.eye(3)*self.bm_cov
		self.R[0:3,0:3] *= self.gravity_cov
		self.R[3:6,3:6] *= self.gyro_cov
		#self.R[6:9,6:9] *= self.mag_cov

		self.initialized = False
		self.imu_initialized = False
		self.magnetic_initialized = False

	def predict(self, gyro, acc, t):#t is the time we read data from sensor
		#print ("gyro before rotation: ", gyro)
		#print ("acc before rotation: ", acc)
		gyro = np.dot(Rotation_mat,gyro)# transform coordinate to NEU coordinate.
		acc = np.dot(Rotation_mat,acc)
		#print ("gyro after rotation: ", gyro)
		#print ("acc after rotation: ", acc)
		if self.imu_initialized == False:
			self.imu_initialized = True
			self.initialized = True
			self.current_t = t 
			#phy = math.atan(acc[0],-acc[2])#initial eular angles by using first data from IMU 
			#theta = math.atan(-acc[1],-acc[2])
			#phy = math.atan2(acc[0],acc[2])#initial eular angles by using first data from IMU 
			#theta = math.atan2(acc[1],acc[2])
			phy = math.atan2(acc[1],acc[2])#initial eular angles by using first data from IMU 
			theta = math.atan2(acc[0],acc[2])
			phy=self.angle(phy)
			theta=self.angle(theta)
			phy1 = phy*180/math.pi
			theta1 = theta*180/math.pi	
			rpy = np.array([theta, phy, 0.0])
			print ("phy theta: ", phy1, theta1)
			q_init = mpl.euler2quaternion(rpy)# returns quaternion
			self.x[0] = q_init.w
			self.x[1:4] = q_init.x,q_init.y,q_init.z
		if t <= self.current_t: return

		dt = t - self.current_t #the time difference between reading time 
		#dt=0.001
		print ("dt:  ", dt)
		#dt=0.0001

		self.process(gyro,acc) # get state transition matrix. The input parameters are raw data from sensor
		#print "x_qian: ", self.x[10:16]
		self.x[0:4] += self.xdot[0:4]*dt
		self.x[7:10] += self.xdot[7:10]*dt
		self.x[10:13] += self.xdot[10:13]*dt
		#self.x[13:16] += self.xdot[13:16]*dt
		#print "x_hou: ", self.x[10:16]
		self.F[0:4,0:4]=np.eye(4)+self.F[0:4,0:4]*dt
		self.F[7:10,7:10]=np.eye(3)+self.F[7:10,7:10]*dt
		self.F[10:13,10:13]=np.eye(3)+self.F[10:13,10:13]*dt
		self.F[13:16,13:16] = np.eye(3)
		self.F[13:16, 16:19] *= dt
		self.F[16:19,16:19] = np.eye(3)
		self.F[16:19,0:4] *= dt
		self.F[16:19, 10:13] *= dt
		#self.F[13:16,13:16]=np.eye(3)+self.F[13:16,13:16]*dt
		#self.G=self.G*dt
		self.G[16:19, 9:12] *= dt
		self.P=np.dot(np.dot(self.F,self.P),self.F.transpose())+\
		np.dot(np.dot(self.G,self.Q),self.G.transpose())

		#!!!!normalize x first 4 terms,i.e. quaternions
		self.x[0:4] /= np.linalg.norm(self.x[0:4],ord = 2)
		print ("euler angle:  ", 180/math.pi*mpl.quaternion2euler(self.array2q(self.x[0:4])))

		self.current_t=t
		self.acc=acc
		self.gyro=gyro


	def process(self, gyro, acc):
		print ("gyro: ", gyro)
		print ("acc: ", acc)
		q=np.array([0.0,0.0,0.0,0.0])#share addtress just make another name
		omega=self.x[4:7]
		bw=self.x[7:10]#what is the initail value of bias?! maybe we could use the first 3 seconds average value# when the drone is static as init bias
		ba=self.x[10:13]
		p=self.x[13:16]
		v=self.x[16:19]
		#bm=self.x[13:16]
		q=self.x[0:4]
		print ("quaternion: ", self.x[0:4])
		print ("omega: ", self.x[4:7])
		print ("biasw: ", self.x[7:10])
		print ("biasa: ", self.x[10:13])
		print ("position: ", self.x[13:16])
		print ("velocity: ", self.x[16:19])
		#print ("biasm: ", self.x[13:16])

		gyro_q = np.array([0.0,0.0,0.0,0.0])
		gyro_q[1:4] = gyro - bw
		print ("gyro_q: ",gyro_q)
		q_dot = mpl.q_p(q,gyro_q) #matrix multiply this line is correct
		q_dot[0:4] = q_dot[0:4]*0.5
		self.xdot[0:4] = q_dot	
		print ("qdot[0:4]: ", self.xdot[0:4])
		self.x[4:7] = gyro_q[1:4]
	
		self.xdot[7:10] = -self.lamda*self.x[7:10]
		self.xdot[10:13] = -self.lamda*self.x[10:13]
		self.xdot[13:16] = v

		acc_b = np.zeros(4)
		acc_b[1:4] = acc - ba
		acc_n = mpl.q_p(mpl.q_p(self.q_inverse(q),acc_b),q)
		self.xdot[16:19] = acc_n[1:4] + GRAVITY


		#self.xdot[13:16] = -self.lamda*self.x[13:16]

		self.F[0:4,0:4] = 0.5*mpl.diff_pq_p(gyro_q)
		self.F[0:4,4:7] = 0.5*mpl.diff_pq_q(q)[0:4,1:4]
		self.F[4:7,7:10] = -np.eye(3)
		self.F[7:10,7:10] = self.lamda*np.eye(3)
		self.F[10:13,10:13] = self.lamda*np.eye(3)
		self.F[13:16, 16:19] = np.eye(3)
		self.F[16:19, 0:4] = mpl.diff_qstarvq_q(q, acc_b[1:4])# or acc[1:4]
		self.F[16:19, 10:13] = -mpl.diff_qstarvq_v(q)
		#self.F[13:16,13:16] = self.lamda*np.eye(3)

		#self.G[0:4,0:3] = -0.5*mpl.diff_pq_q(q)[0:4,1:4]
		#self.G[4:7,3:6] = np.eye(3)
		self.G[4:7,0:3] = np.eye(3)#noise of angular velocity
		self.G[7:10,3:6] = np.eye(3)#noise of bias omega
		self.G[10:13,6:9] = np.eye(3)#noise of bias acc
		self.G[16:19, 9:12] = mpl.diff_qstarvq_v(q)
		#self.G[13:16,9:12] = np.eye(3)


	def update(self, acc, gyro, mag, t):#acc is the raw data from IMU
		if self.initialized==False:
			self.initialized = True
			self.current_t = t
		if t < self.current_t: return
		gyro = np.dot(Rotation_mat,gyro)# transform coordinate to NEU coordinate.
		acc = np.dot(Rotation_mat,acc)
		#mag = np.dot(Rotation_mat,mag)

		z=np.zeros(6)
		z[0:3]=acc/np.linalg.norm(acc,ord=2)
		z[3:6]=gyro
		z[6:9]=np.zeros(3)+random.gauss(0,0.1)
		z[9:12]=np.zeros(3)+random.gauss(0,0.1)
		#z[6:9]=mag
		self.measurement()
		#print "self.H: ", self.H
		temp_K = np.linalg.inv(np.dot(self.H, np.dot(self.P,self.H.transpose()))+self.R)
		#print "temp_K: ", temp_K
		self.K = np.dot(np.dot(self.P,self.H.transpose()),temp_K)
		#print "self.K: ",self.K
		self.x += np.dot(self.K,(z-self.zhat))
		print ("z-zhat: ", z-self.zhat)
		I=np.eye(19)
		print ("P qian: ", np.diag(np.mat(self.P)))
		self.P = np.dot((I - np.dot(self.K, self.H)), self.P)
		print ("P hou: ", np.diag(np.mat(self.P)))
		self.x[0:4] /= np.linalg.norm(self.x[0:4],ord = 2)

	def measurement(self): #acc is model result
		q=np.array([0.0,0.0,0.0,0.0])
		q=self.x[0:4]
		#ba=self.x[13:16]
		g_n_q = np.array([0.0,0.0,0.0,1.0])
		acc_q = mpl.q_p(mpl.q_p(q,g_n_q),self.q_inverse(q)) #????????normalize
		#geo_mag_field_local_var=np.array([0.0,geo_mag_field[0],geo_mag_field[1],geo_mag_field[2]])
		#mag_zhat = mpl.q_p(mpl.q_p(q,geo_mag_field_local_var),self.q_inverse(q))
		print ("acc_q: ", acc_q)
		self.zhat[0:3] = acc_q[1:4]+self.x[10:13] #acc+ba
		self.zhat[3:6] = self.x[4:7]+self.x[7:10] #wb+bw
		self.zhat[6:9] = self.x[13:16]
		self.zhat[9:12] = self.x[16:19]
		#self.zhat[6:9] = mag_zhat[1:4]+self.x[13:16]#mag+bm
		self.H[0:3,0:4] = mpl.diff_qvqstar_q(q, GRAVITY)
		self.H[0:3,10:13] = np.eye(3)
		self.H[3:6,4:7] = np.eye(3)
		self.H[3:6,7:10] = np.eye(3)
		self.H[6:9, 13:16] = np.eye(3)
		self.H[9:12, 16:19] = np.eye(3)

		#self.H[6:9,0:4] = mpl.diff_qvqstar_q(q,geo_mag_field)
		#self.H[6:9,13:16] = np.eye(3)

	def q_normalize(self, q):
		sum=math.sqrt(q.w**2+q.x**2+q.y**2+q.z**2)
		q.w/=sum
		q.x/=sum
		q.y/=sum
		q.z/=sum
		return q

	def q_inverse(self,q):
		q[1:4] = -q[1:4]
		 
		q=q/(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])
		return q

	def q2array(self, q):
		a=np.array([q.w,q.x,q.y,q.z])
		return a

	def array2q(self, a):
		q=np.quaternion(a[0],a[1],a[2],a[3])
		return q

	def angle(self,a):
		if a<-math.pi*0.5:
			print("in1")
			a+=math.pi
		elif a>math.pi*0.5:
			print("in2")
			a-=math.pi
		return a

	####