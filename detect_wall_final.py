#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians
import tf
import time
import PyKDL

tf_listener = tf.TransformListener()
angular_tolerance = radians(2)

def callback(msg):

#If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
  if msg.ranges[360] > 1:
      move.linear.x = 0.5
      move.angular.z = 0.0

#If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
 #If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
  print "Number of readings: "  
  print len(msg.ranges)
  print "Reading at position 360:", msg.ranges[360]
  if msg.ranges[360] < 1:
      move.linear.x = 0.0
      move.angular.z = 0.0

  pub.publish(move)

rospy.init_node('rotw5_node')
sub = rospy.Subscriber('/scan', LaserScan, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

rospy.spin()

def get_odom():

  # Get the current transform between the odom and base frames
  tf_ok = 0
  while tf_ok == 0 and not rospy.is_shutdown():
    try:
      tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(), rospy.Duration(1.0))
      tf_ok = 1
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        pass

    try:
      (trans, rot) = tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
      rospy.loginfo("TF Exception")
      return

    quat = Quaternion(*rot)
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)     #to convert quaternion to angle

    return (Point(*trans), rot.GetRPY()[2])


def normalize_angle(angle):
  res = angle
  while res > pi:
    res -= 2.0 * pi
  while res < -pi:
    res += 2.0 * pi
  return res

def rotate(degrees):

  position = Point()
  (position, rotation) =  get_odom() #getting the current position
  if degrees > 0:
    move.angular.z = 0.3
  else:
    move.angular.z = -0.3 

  last_angle = rotation
  turn_angle = 0
  goal_angle = radians(degrees)
  # Begin the rotation
  while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
    # Publish the Twist message and sleep 1 cycle
    pub.publish(move)
    rospy.Rate(10).sleep()
    # Get the current rotation
    (position, rotation) = get_odom()
    # Compute the amount of rotation since the last loop
    delta_angle = normalize_angle(rotation - last_angle)
    turn_angle += delta_angle
    last_angle = rotation
    
  move.linear.x = 0.0
  move.angular.z = 0.0



  #! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, pi
import tf
import time
import PyKDL
rospy.init_node('rotw5_node')
# We subscribe to the laser's topic

tf_listener = tf.TransformListener()
angular_tolerance = radians(2)

move = Twist()

rospy.spin()


def get_odom():

    # Get the current transform between the odom and base frames
    tf_ok = 0
    while tf_ok == 0 and not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                '/base_link', '/odom', rospy.Time(), rospy.Duration(1.0))
            tf_ok = 1
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            pass

        try:
            (trans, rot) = tf_listener.lookupTransform(
                'odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        quat = Quaternion(*rot)
        # to convert quaternion to angle
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)

        return (Point(*trans), rot.GetRPY()[2])


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


def rotate(degrees):

    position = Point()
    (position, rotation) = get_odom()  # getting the current position
    if degrees > 0:
        move.angular.z = 0.3
    else:
        move.angular.z = -0.3

    last_angle = rotation
    turn_angle = 0
    goal_angle = radians(degrees)
    # Begin the rotation
    while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
        # Publish the Twist message and sleep 1 cycle
        pub.publish(move)
        rospy.Rate(10).sleep()
        # Get the current rotation
        (position, rotation) = get_odom()
        # Compute the amount of rotation since the last loop
        delta_angle = normalize_angle(rotation - last_angle)
        turn_angle += delta_angle
        last_angle = rotation

    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)


def callback(msg):

    # If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
    # if msg.ranges[360] > 1:
    #    move.linear.x = 0.5
    #    move.angular.z = 0.0

    # If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
   # If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
    print("Number of readings: ")
    # print len(msg.ranges)
    print("Reading at position 360:", msg.ranges[360])
    # if msg.ranges[360] < 1:
    #    move.linear.x = 0.0
    #    move.angular.z = 0.0
    rotate(90)
    pub.publish(move)


sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)
