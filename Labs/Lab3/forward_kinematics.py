#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
import kin_func_skeleton as kfs
from sensor_msgs.msg import JointState
import numpy as np
import math

'''The Code below is borrowed from openCV at
https://www.learnopencv.com/rotation-matrix-to-euler-angles/
to check to see if our rotation matrix does indeed produce valid RPY angles'''

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def twist(q,w):     
    t = np.zeros(6)
    t[0:3] = np.cross(-w,q)
    t[3:6] = w
    return t

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):
    
    #Print the contents of the message to the console
    # left arm angles
    theta = [message.position[4], message.position[5], message.position[2], message.position[3], message.position[6], message.position[7], message.position[8]]
    print('Angles list (left hand)\n')
    print(theta)
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
    trans = kfs.prod_exp(t,theta)
    twist_arm = np.ndarray((4,4))
    twist_arm[0:3,0:3] = R
    twist_arm[0:3,3] = q[0:3,7]
    twist_arm[3,3] = 1
    trans = np.dot(trans,twist_arm)
    print(trans)

    rotation = trans[0:3,0:3]
    print('Calculated RPY angles')
    print rotationMatrixToEulerAngles(rotation)

#Define the method which contains the node's main functionality
def forward_kinematics():

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('forward_kinematics', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("/robot/joint_states", JointState, callback, queue_size=10)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    forward_kinematics()









