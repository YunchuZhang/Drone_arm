import numpy as np
import math
#from numpy import quaternion
import quaternion
class mpl:
	def __init__(self):
		pass
	def skew_symmetric(self, v):
		m = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
		return m

	def diff_pq_q(self, p):
		p0 = p.w
		pv = np.array([p.x,p.y,p.z])
		D = np.zeros((4,4))
		D[0,0] = p0
		D[0,1:4] = -1*pv
		D[1:4,0] = pv.transpose()
		D[1:4,1:4] = np.eye(3)*p0 + skew_symmetric(pv)
		return D
	def diff_pq_p(self, q):
		q0 = q.w
		qv = np.array([q.x,q.y,q.z])
		D = np.zeros((4,4))
		D[0,0] = q0
		D[0,1:4] = -1*qv
		D[1:4,0] = qv.transpose()
		D[1:4,1:4] = np.eye(3)*q0 - skew_symmetric(qv)
		return D
	def diff_qvqstar_q(self, q, v):
		q0 = q.w
		qv = np.array([q.x,q.y,q.z])
		D = np.zeros((3,4))
		D[:,0] = 2*(q0*v.transpose() + np.dot(skew_symmetric(qv),v.transpose()))
		D[:,1:4] = 2*(np.dot(-1*v.transpose(),qv) + ((v*qv).sum())*np.eye(3) - q0*skew_symmetric(v))
		return D
	def diff_qstarvq_q(self, q, v):
		q0 = q.w
		qv = np.array([q.x,q.y,q.z])
		D = np.zeros((3,4))
		D[:,0] = 2*(q0*v.transpose() - np.dot(skew_symmetric(qv),v.transpose()))
		D[:,1:4] = 2*(np.dot(-1*v.transpose(),qv) + ((v*qv).sum())*np.eye(3) + q0*skew_symmetric(v))
		return D
	def diff_qvqstar_v(self, q):
		q0 = q.w
		qv = np.array([q.x,q.y,q.z])
		#D = np.zeros((3,3))
		D = (q0*q0 - (qv*qv).sum())*np.eye(3) + 2*np.dot(qv.transpose(),qv) + 2*q0*skew_symmetric(qv)
		return D
	def euler2quaternion(self, euler):
		cr = math.cos(euler[0]/2.0)
		sr = math.sin(euler[0]/2.0)
		cp = math.cos(euler[1]/2.0)
		sp = math.sin(euler[1]/2.0)
		cy = math.cos(euler[2]/2.0)
		sy = math.sin(euler[2]/2.0)
		q = np.quaternion(0,0,0,0)
		q.w = cr*cp*cy + sr*sp*sy
		q.x = sr*cp*cy - cr*sp*sy
		q.y = cr*sp*cy + sr*cp*sy
		q.z = cr*cp*sy - sr*sp*cy
		return q
	def quaternion2mat(self, q):
		m = np.zeros((3,3))
		a = q.w
		b = q.x
		c = q.y
		d = q.z
		m = np.array([[a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c)],
			[2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b)],
			[2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d]])
		return m
	def mat2eular(self, m):
		r = math.atan2(m[2, 1], m[2, 2])
		p = math.asin(-1*m[2, 0])
		y = math.atan2(m[1, 0], m[0, 0])
		np.array([r, p, y])
		return rpy
	def mat2quaternion(self, m):
		q = np.quaternion(0,0,0,0)
		a = math.sqrt(1 + m[0, 0] + m[1, 1] + m[2, 2])/2.0
		b = (m[2, 1] - m[1, 2])/(4.0*a)
		c = (m[0, 2] - m[2, 0])/(4.0*a)
		d = (m[1, 0] - m[0, 1])/(4.0*a)
		q.w = a
		q.x = b
		q.y = c
		q.z = d
		return q
	def euler2mat(self, euler):
		cr = math.cos(euler[0]/1.0)
		sr = math.sin(euler[0]/1.0)
		cp = math.cos(euler[1]/1.0)
		sp = math.sin(euler[1]/1.0)
		cy = math.cos(euler[2]/1.0)
		sy = math.sin(euler[2]/1.0)
		m = np.zeros((3,3))
		m = np.array([[cp*cy,  -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy], 
			[cp*sy,  cr*cy + sr*sp*sy,  -sr*cy + cr*sp*sy], 
			[-sp,    sr*cp,             cr*cp]])
		return m
	def quaternion2euler(self, q):
		return mat2euler(quaternion2mat(q))














	