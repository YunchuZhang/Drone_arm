import numpy as np
import math
import random as random
#from numpy import quaternion
from manipulation import *
GRAVITY=np.array([0,0,9.8])
mpl = mpl()
class EKF:
	x=np.mat(np.zeros(16))#16 states q p v bw ba
	xdot=np.mat(np.zeros(16))#16 states derivatives
	z=np.mat(np.zeros(3))#real raw data from sensor
	zhat=np.mat(np.zeros(3))#H*x_bar
	P=np.mat(np.eye(16))#covariance matrix
	Q=np.mat(np.zeros((6,6)))#process noise covariance
	F=np.mat(np.zeros((16,16)))#state transition
	G=np.mat(np.zeros((16,6)))
	H=np.mat(np.zeros((3,16)))#observation Matrix
	R=np.mat(np.eye(3))#observation noise Matrix
	gyro_cov=0.0001
	acc_cov = 0.0001
	gravity_cov=5
	current_t=0
	gravity=np.mat(np.array([0,0,9.8]))

	initialized = False
	imu_initialized = False
	magnetic_initialized = False
	acc=np.zeros(3)
	gyro=np.zeros(3)


	def __init__(self):
		initialized = False
		self.x[0,0]=1
		self.Q[0:3,0:3]*=self.gyro_cov
		self.Q[3:6,3:6]*=self.acc_cov
		self.R*=self.gravity_cov
		self.x[:,10:13]=[0,0,0]
		self.x[:,13:16]=[0,0,0]

		self.initialized = False
		self.imu_initialized = False
		self.magnetic_initialized = False

	def predict(self, gyro, acc, t,bA,bb):#t is the time we read data from sensor
		if self.imu_initialized ==False:
			self.imu_initialized=True
			self.initialized = True
			self.current_t=t
			phy = math.atan2(acc[1],acc[2])#initial eular angles by using first data from IMU 
			theta = math.atan2(-acc[0],acc[2])
			phy1=phy*180/math.pi
			theta1=theta*180/math.pi	
			rpy=np.array([phy, theta, 0])
			print "phy theta: ", phy1, theta1
			q_init=mpl.euler2quaternion(rpy)# returns quaternion
			self.x[:,0] = q_init.w
			self.x[:,1:4] = q_init.x,q_init.y,q_init.z
		if t <= self.current_t: return

		#dt = t - self.current_t #the time difference between reading time 
		dt=0.0001

		self.process(gyro, acc,bA,bb) # get state transition matrix. The input parameters are raw data from sensor
		print "x_qian: ", self.x[10:16]
		self.x += self.xdot*dt
		print "x_hou: ", self.x[10:16]
		self.F=np.eye(16)+self.F*dt
		self.G=self.G*dt
		#self.P=np.dot(np.dot(self.F,self.P),self.F.transpose())+\
		#np.dot(np.dot(self.G,self.Q),self.G.transpose())
		self.P = self.F*self.P*self.F.T+self.G*self.Q*self.G.T

		#!!!!normalize x first 4 terms,i.e. quaternions
		self.x /= np.linalg.norm(self.x[0:4],ord = 2)
		self.current_t=t
		self.acc=acc
		self.gyro=gyro


	def process(self, gyro, acc,bA,bb):
		q=np.quaternion(0,0,0,0)#share addtress just make another name
		p=self.x[4:7]
		v=self.x[7:10]
		bw=self.x[10:13]#what is the initail value of bias?! maybe we could use the first 3 seconds average value
		ba=self.x[13:16]# when the drone is static as init bias
		q.w=self.x[0]
		q.x,q.y,q.z=self.x[1:4]
		print "quaternion: ", self.x[0:4]
		print "position: ", self.x[4:7]
		print "velocity: ", self.x[7:10]
		print "biasw: ", self.x[10:13]
		print "biasa: ", self.x[13:16]

		gyro_q=np.quaternion(0,0,0,0)
		gyro_q.x, gyro_q.y, gyro_q.z=gyro-bw#-random.gauss(0,0.01) #-bb#
		q_dot=q*gyro_q #matrix multiply this line is correct
		q_dot.w/=2.0
		q_dot.x/=2.0
		q_dot.y/=2.0
		q_dot.z/=2.0
		self.xdot[0] = q_dot.w
		self.xdot[1:4] = q_dot.x, q_dot.y, q_dot.z
		self.xdot[4:7] = v

		acc_b_q=np.zeros(4)
		acc_b_q[1:4]=acc-ba#-random.gauss(0,0.01)#ba-bA
		#print "acc_b_q: ", acc_b_q
		acc_b_q=self.array2q(acc_b_q)
		#print "acc_b_q quat: ", acc_b_q
		acc_n_q=self.q2array(q*acc_b_q*self.q_inverse(q))
		#print "acc_n_q array: ", acc_n_q
		self.xdot[7:10]=acc_n_q[1:4]-self.gravity
		#print "acc_before gravity minus: ", acc_n_q
		print "final acc_from_model: ", self.xdot[7:10]

		self.F[0:4,0:4]=0.5*mpl.diff_pq_p(gyro_q)
		self.F[0:4,10:13]=-0.5*mpl.diff_pq_q(q)[0:4,1:4]
		self.F[4:7,7:10]=np.eye(3)
		self.F[7:10,0:4]=mpl.diff_qvqstar_q(q,self.q2array(acc_b_q)[1:4])
		self.F[7:10,13:16]=-mpl.diff_qvqstar_v(q)

		self.G[0:4,0:3]=0.5*mpl.diff_pq_q(q)[0:4,1:4]
		self.G[7:10,3:6]=mpl.diff_qvqstar_v(q)


	def update(self, acc, t):#acc is the raw data from IMU
		if self.initialized==False:
			self.initialized = True
			self.current_t = t
		if t < self.current_t: return

		z=np.mat(acc/np.linalg.norm(acc,ord=2))
		self.measurement()
		#print "self.H: ", self.H
		#temp_K = np.linalg.inv(np.dot(self.H, np.dot(self.P,self.H.transpose()))+self.R)
		#print "temp_K: ", temp_K
		#self.K = np.dot(np.dot(self.P,self.H.transpose()),temp_K)
		#print "self.K: ",self.K
		#self.x += np.dot(self.K,(z-self.zhat))
		self.K = self.P*self.H.T*(self.H*self.P*self.H.T+self.R).I
		self.x += self.K*(z-self.zhat)
		print "z-zhat: ", z-self.zhat
		I=np.mat(np.eye(16))
		print "P qian: ", np.diag(np.mat(self.P))
		#self.P = np.dot((I - np.dot(self.K, self.H)), self.P)
		self.P = (I - self.K*self.H)*self.P
		print "P hou: ", np.diag(np.mat(self.P))
		self.x[0:4] = self.q2array(self.q_normalize(self.array2q(self.x[0:4])))

	def measurement(self): #acc is model result
		q=np.quaternion(0,0,0,0)
		q.w=self.x[0,0]
		q.x,q.y,q.z=self.x[0,1], self.x[0,2], self.x[0,3]
		#ba=self.x[13:16]
		g_n_q=np.quaternion(0,0,0,1)
		acc_q=self.q_inverse(q)*g_n_q*q #????????normalize
		#print "acc_q: ", acc_q
		self.zhat[0:3] = acc_q.x, acc_q.y, acc_q.z
		self.H[0:3,0:4] = mpl.diff_qstarvq_q(q, GRAVITY)

	def q_normalize(self, q):
		sum=math.sqrt(q.w**2+q.x**2+q.y**2+q.z**2)
		q.w/=sum
		q.x/=sum
		q.y/=sum
		q.z/=sum
		return q

	def q_inverse(self,q):
		q.x, q.y, q.z=-q.x, -q.y, -q.z
		 
		q=q/(q.w**2+q.x**2+q.y**2+q.z**2)
		return q

	def q2array(self, q):
		a=np.array([q.w,q.x,q.y,q.z])
		return a

	def array2q(self, a):
		q=np.quaternion(a[0],a[1],a[2],a[3])
		return q

	def q_vec(self, q):
		return q_vec

	####