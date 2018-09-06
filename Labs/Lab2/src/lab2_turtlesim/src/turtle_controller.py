#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy

#Import topic to run the turtle
from geometry_msgs.msg import Twist
import sys


#Define the method which contains the main functionality of the node.
def talker():
  turtleToControl = 'turtle1'
  if len(sys.argv) >1:
    print("New Turtle, it's name is "+ sys.argv[1])
    turtleToControl = sys.argv[1]
  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('talker', anonymous=True)

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type std_msgs/String to the topic as listed below by inspection using
  # "rosnode info /teleop_turtle 
  pub = rospy.Publisher(turtleToControl + '/cmd_vel', Twist, queue_size=10)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  oldTwist = Twist() 

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    print("CMDS are WSADQE, W forward, S back, A strafe right, B strafe left, Q rotate left, E rotate right")
    s = raw_input('-->')
    
    
    # Construct a string that we want to publish
    #pub_string = "hello world %s"%rospy.get_time()
    
    #using instructions on "Moving the base" of :mini_max" turotiral of ros.org

    twist = Twist()
    print("Command was " + s)
    if s == 'w':
        twist.linear.x = oldTwist.linear.x + 1
    elif s == 's':
        twist.linear.x = oldTwist.linear.x - 1
    else:
        twist.linear.x = oldTwist.linear.x

    #lateral doesn't seem to work fore the tiome being
    if s == 'a':
        twist.linear.y = oldTwist.linear.y - 1
    elif s == 'd':
        twist.linear.y = oldTwist.linear.y + 1
    else:
        twist.linear.y = oldTwist.linear.y

    if s == 'q':
        twist.angular.z = oldTwist.angular.z + 1
    elif s == 'e':
        twist.angular.z = oldTwist.angular.z - 1
    else:
        twist.angular.z = oldTwist.angular.z


    oldTwist = twist

    twist.linear.z = 0  #should always be zero, we're not levitating
    twist.angular.x = 0  #rotate about the x axis... this should be zero
    twist.angular.y = 0  #should also always be zero

    # Publish our string to the 'chatter_talk' topic
    pub.publish(twist)
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker methodo
  try:
    talker()
  except rospy.ROSInterruptException: pass
  
