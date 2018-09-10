#!/usr/bin/env python
"""
Lab 3, task 1
"""


import numpy as np
# import scipy as sp
import kin_func_skeleton as kfs

def twist(q,w):	    
	t = np.zeros(6)
	t[0:3] = np.cross(-w,q)
	t[3:6] = w
	return t

def lab3():
	q = np.ndarray((3,8))
	w = np.ndarray((3,7))
    
	q[0:3,0] = [0.0635, 0.2598, 0.1188]
	q[0:3,1] = [0.1106, 0.3116, 0.3885]
	q[0:3,2] = [0.1827, 0.3838, 0.3881]
	q[0:3,3] = [0.3682, 0.5684, 0.3181]
	q[0:3,4] = [0.4417, 0.6420, 0.3177]
	q[0:3,5] = [0.6332, 0.8337, 0.3067]
	q[0:3,6] = [0.7152, 0.9158, 0.3063]
	q[0:3,7] = [0.7957, 0.9965, 0.3058]

	w[0:3,0] = [-0.0059,  0.0113,  0.9999]
	w[0:3,1] = [-0.7077,  0.7065, -0.0122]
	w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
	w[0:3,3] = [-0.7077,  0.7065, -0.0122]
	w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
	w[0:3,5] = [-0.7077,  0.7065, -0.0122]
	w[0:3,6] = [ 0.7065,  0.7077, -0.0038]
	R = np.array([[0.0076, 0.0001, -1.0000],[-0.7040, 0.7102, -0.0053],[0.7102, 0.7040, 0.0055]]).T
	t = np.zeros((6,7))
	for i in range(0,7):
		t[:,i] = twist(q[0:3,i],w[0:3,i])
	theta = np.array([0,0,0,0,0,0])
	trans = kfs.prod_exp(t,theta)
	twist_arm = np.ndarray((4,4))
	twist_arm[0:3,0:3] = R
	twist_arm[0:3,3] = q[0:3,7]
	twist_arm[3,3] = 1
	trans = np.dot(trans,twist_arm)
	print(trans)


if __name__ == "__main__":
    print('Lab 3')
    lab3()
