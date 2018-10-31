import numpy as np
import math
from numpy import quaternion
GRAVITY=np.array([0,0,9.8])

class EKF:
	x=np.zeros(16)#16 states q p v bw ba
	xdot=np.zeros(16)#16 states derivatives
	z=np.zeros(6)#real raw data from sensor
	zhat=np.zeros(6)#H*x_bar
	P=np.eye(16)#covariance matrix
	Q=np.zeros((6,6))#process noise covariance
	F=np.zeros((16,16))#state transition
	G=np.zeros(16,6)
	H=np.zeros(6,16)#observation Matrix
	R=np.eye(3)#observation noise Matrix
	gyro_cov=0.01
	gravity_cov=5.0
	current_t=0
	gravity=np.array([0,0,9.8])

	acc=np.zeros(3)
	gyro=np.zeros(3)


	def __init__(self):
		initialized = False
		self.x[0]=1
		self.Q[0:3,0:3]*=gyro_cov
		self.Q[3:6,3:6]*=acc_cov
		self.R*=gravity_cov

		initialized = False
		imu_initialized = False
		magnetic_initialized = False

	def predict(self, gyro, acc, t):#t is the time we read data from sensor
		if imu_initialized ==False:
			imu_initialized=True
			initialized = True
			self.current_t=t
			phy = atan2(acc[1],acc[2])#initial eular angles by using first data from IMU 
			theta = atan2(-acc[0],acc[2])
			rpy=np.array([phy, theta, 0])
			q_init=mpl.euler2quaternion(rpy)# returns quaternion
			self.x[0] = q_init.w
			self.x[1:4] = q_init.x,q_init.y,q_init.z
		if t <= current_t: return

		dt = t - current_t #the time difference between reading time 

		self.process(gyro, acc) # get state transition matrix. The input parameters are raw data from sensor

		self.x += self.xdot*dt
		self.F=np.eye(16)+self.F*dt
		self.G=self.G*dt

		self.P=np.dot(np.dot(self.F,self.P),self.F.transpose())+\
		np.dot(np.dot(self.G,self.Q),self.G.transpose())
		#!!!!normalize x first 4 terms,i.e. quaternions
		self.x /= np.linalg.norm(self.x[0:4],ord = 2)
		self.current_t=t
		self.acc=acc
		self.gyro=gyro


	def process(self, gyro, acc):
		q=np.quaternion(0,0,0,0)#share addtress just make another name
		p=self.x[4:7]
		v=self.x[7:10]
		ba=self.x[10:13]#what is the initail value of bias?! maybe we could use the first 3 seconds average value
		bw=self.x[13:16]# when the drone is static as init bias
		q.w=self.x[0]
		q.x,q.y,q.z=self.x[1:4]

		gyro_q=np.quaternion(0,0,0,0)
		gyro_q.x, gyro_q.y, gyro_q.z=gyro-bw#
		q_dot=q*gyro_q #matrix multiply
		q_dot.w/=2
		q_dot.x/=2
		q_dot.y/=2
		q_dot.z/=2
		self.xdot[0] = q_dot.w
		self.xdot[1:4] = q_dot.x, q_dot.y, q_dot.z
		self.xdot[4:7] = v

		acc_b_q=np.zeros(4)
		acc_b_q[1:4]=acc-ba
		acc_b_q=self.array2q(acc_b_q)
		acc_n_q=q2array(q*acc_b_q*q_inverse(q))
		self.xdot[7:10]=acc_n_q[1:4]-self.gravity

		self.F[0:4,0:4]=0.5*mpl.diff_pq_q(gyro_q)
		self.F[0:4,10:13]=-0.5*mpl.diff_pq_q(q)[0:4,1:4]
		self.F[4:7,7:10]=np.eye(3)
		self.F[7:10,0:4]=diff_qvqstar_q(q,q2array(acc_b_q)[1:4])
		self.F[7:10,13:16]=-diff_qvqstar_v(q)

		self.G[0:4,0:3]=0.5*diff_pq_q(q)[0:4,1:4]
		self.G[7:10,3:6]=diff_qvqstar_v(q)

	def update(self, acc, t):#acc is the raw data from IMU
		if initialized==False:
			initialized = True
			self.current_t = t
		if t < self.current_t: return

		z=acc/np.linalg.norm(acc,ord=2)
		self.measurement()
		temp_K = np.linalg.inv(np.dot(self.H, np.dot(self.P,self.H.transpose()))+self.R)
		self.K = np.dot(np.dot(self.P*self.H.transpose()),temp_K)
		self.x += K*(z-self.zhat)
		I=np.eye(16)
		self.P = ((I - np.dot(self.K, self.H)), self.P)
		self.x[0:4] = q2array(self.q_normalize(array2q(self.x[0:4])))

	def measurement(self): #acc is model result
		q=np.quaternion(0,0,0,0)
		q.w=self.x[0]
		q.x,q.y,q.z=self.x[1:4]
		#???ba
		g_n_q=np.quaternion(0,0,0,1)
		acc_q=self.q_inverse(q)*g_n_q*q #????????normalize
		self.zhat[0:3] = acc_q.x, acc_q.y, acc_q.z
		self.H[0:3,0:4] = diff_qstarvq_q(q, GRAVITY)

	def q_normalize(self, q):
		sum=math.sqrt(q.w**2+q.x**2+q.y**2+q.z**2)
		q.w/=sum
		q.x/=sum
		q.y/=sum
		q.z/=sum
		return q

	def q_inverse(self,q):
		q.x, q.y, q.z=-q.x, -q.y, -q.z
		return q=q/math.sqrt(q.w**2+q.x**2+q.y**2+q.z**2)

	def q2array(self, q):
		return a=np.array([q.w,q.x,q.y,q.z])

	def array2q(self, a):
		return q=np.quaternion(a[0],a[1],a[2],a[3])

	def q_vec(self, q):
		return q_vec
