#! /usr/bin/env python

import rospy
# Messages
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
# Math
from math import pi, exp, sin, sqrt
import numpy as np
# Quaternions tools
import PyKDL

## STATES NUMBERS
# 'pos'         ->  1.0
# 'rate'        ->  2.0
# 'vib'         ->  3.0
# 'center'      ->  4.0
# 'desyn'       ->  5.0
# 'collision'   ->  6.0

class Force_handler:

  def __init__(self):
      
    # Read all the parameters from the parameter server
    # Generic Parameters
    master_name = self.read_parameter('~master_name', 'phantom')
    robot_name = self.read_parameter('~robot_name', 'grips')
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.ref_frame = self.read_parameter('~ref_frame', 'world')

    # Topic names
    self.master_pose_topic = '/master_%s/pose' % master_name
    self.master_vel_topic = '/master_%s/velocity' % master_name
    self.feedback_topic = '/master_%s/force_feedback' % master_name
    self.sm_control_topic = '/teleop_controller/current_sm'
    self.ext_forces_topic = '/robot_%s/external_forces' % robot_name
    
    # Transformation axes pose parameter
    self.position_axes = [0, 1, 2]
    self.position_sign = np.array([1.0, 1.0, 1.0])
    self.axes_mapping = self.read_parameter('~axes_mapping', ['x', 'y' ,'z'])   
    if len(self.axes_mapping) != 3:
      rospy.logwarn('The invalid number of values in [axes_mapping]. Received 3, expected %d' % len(self.axes_mapping))
    for i, axis in enumerate(self.axes_mapping):
      axis = axis.lower()
      if '-' == axis[0]:
        axis = axis[1:]
        self.position_sign[i] = -1.0
      if axis not in ('x','y','z'):
        rospy.logwarn('Invalid axis %s given in [axes_mapping]' % axis)
      self.position_axes[i] = ['x','y','z'].index(axis)
    
    # Vibration parameters
    self.vib_a = self.read_parameter('~vibration/a', 2.0)             # Amplitude (mm)
    self.vib_c = self.read_parameter('~vibration/c', 5.0)             # Damping
    self.vib_freq = self.read_parameter('~vibration/frequency', 30.0) # Frequency (Hz)
    self.vib_time = self.read_parameter('~vibration/duration', 0.3)   # Duration (s)
    self.vib_start_time = 0.0
    
    # Force feedback parameters
    self.k_center = self.read_parameter('~k_center', 0.1)
    self.b_center = self.read_parameter('~b_center', 0.003)
    self.k_rate = self.read_parameter('~k_rate', 0.05)
    self.b_rate = self.read_parameter('~b_rate', 0.003)

    # Setup Subscribers/Publishers
    self.feedback_pub = rospy.Publisher(self.feedback_topic, WrenchStamped)
    rospy.Subscriber(self.master_pose_topic, PoseStamped, self.cb_master_pose)
    rospy.Subscriber(self.master_vel_topic, TwistStamped, self.cb_master_velocity)
    rospy.Subscriber(self.sm_control_topic, Float64, self.cb_sm_control) 
    rospy.Subscriber(self.ext_forces_topic, WrenchStamped, self.cb_ext_forces)
    
    # Initial values
    self.master_pose = np.zeros(3)
    self.master_velocity = np.zeros(3)
    self.ext_forces = np.zeros(3)
    self.force_feedback = np.zeros(3)
    self.sm_control = 4.0 # GO_TO_CENTER
    self.new_vib_signal = False
    self.vib_start_time = 0.0

    #Timer for publish forces
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.send_feedback)

    rospy.spin()
    
  def normalize_vector(self, v):
    result = np.array(v)
    norm = np.sqrt(np.sum((result ** 2)))
    if norm:
      result /= norm
    return result    
            
  def cb_master_pose(self, msg):
    self.master_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
  def cb_master_velocity(self, msg):
    self.master_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    self.master_dir = self.normalize_vector(self.master_vel)   
  def cb_sm_control(self, msg):
    if ((msg.data != self.sm_control) and (msg.data == 3.0)):
        self.new_vib_signal = True
    self.sm_control = msg.data
  def cb_ext_forces(self, msg):
    self.ext_forces = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
    
  def change_force_axes(self, array, index=None, sign=None):
    if index == None:
      index = self.position_axes
    if sign == None:
      sign = self.position_sign
    result = np.zeros(len(array))
    for i, idx in enumerate(index):
      result[i] = array[idx] * sign[i]
    return result
    
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)


  def send_feedback(self, event):
    # POSITION_CONTROL
    if (self.sm_control == 1.0):
        self.force_feedback = change_force_axes(self.ext_forces)
    # RATE_CONTROL
    elif (self.sm_control == 2.0):
        self.force_feedback = change_force_axes(self.ext_forces) + (self.k_rate * self.master_pos + self.b_rate * self.master_vel) * -1.0
    # VIBRATORY_PHASE
    elif (self.sm_control == 3.0):
      if (self.new_vib_signal):
        self.new_vib_signal = False
        self.vib_start_time = rospy.get_time()
      else:
        t = rospy.get_time() - self.vib_start_time
        amplitude = -self.vib_a*exp(-self.vib_c*t)*sin(2*pi*self.vib_freq*t);
        self.force_feedback = amplitude * self.master_dir
    # GO_TO_CENTER
    elif (self.sm_control == 4.0):
        self.force_feedback = (self.k_center * self.master_pos + self.b_center * self.master_vel) * -1.0
    # REST OF THEM (No forces)
    else :
        self.force_feedback = np.zeros(3)

    # Publish force_feedback
    f_msg = WrenchStamped()
    f_msg.header.frame_id = self.ref_frame
    f_msg.header.stamp = rospy.Time.now()
    f_msg.wrench.force.x = self.force_feedback[0]
    f_msg.wrench.force.y = self.force_feedback[1]
    f_msg.wrench.force.z = self.force_feedback[2]
    self.feedback_pub.publish(f_msg) 
           

if __name__ == '__main__':
    rospy.init_node('Force_handler', log_level=rospy.WARN)
    loader = Force_handler()
