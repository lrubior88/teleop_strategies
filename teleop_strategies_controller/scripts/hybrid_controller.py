#! /usr/bin/env python

import rospy
# Messages
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker

# Math
from math import pi, exp, sin, sqrt
import numpy as np
import tf.transformations as tr
# Quaternions tools
import PyKDL
import time

class HybridController:
  def __init__(self):

    # Read all the parameters from the parameter server
    # Generic Parameters
    master_name = self.read_parameter('~master_name', 'phantom')
    robot_name = self.read_parameter('~robot_name', 'grips')
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.ref_frame = self.read_parameter('~ref_frame', 'world')
    # Rate-pos parameters
    self.strategy = self.read_parameter('~strategy','bubble_rate_pos')
    self.bubble_radius = self.read_parameter('~bubble_radius', 0.5)
    self.scale = self.read_parameter('~scale', 4)
    
    # Topic names
    self.master_pose_topic = '/master_%s/pose' % master_name
    self.master_vel_topic = '/master_%s/velocity' % master_name
    self.sm_control_topic = '/teleop_controller/current_sm'
    self.robot_pose_topic = '/robot_%s/pose' % robot_name
    self.ik_mc_topic = '/robot_%s/ik_command' % robot_name

    # Rate parameters
    self.rate_pivot = np.zeros(3)
    self.rate_gain = self.read_parameter('~rate_gain', 1.0)    

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
        
    # Rotation between Slave and Master
    self.axes_arot_1 = self.read_parameter('~axes_arot_1', [0, 0, 0])
    self.angle_arot_1 = self.read_parameter('~angle_arot_1', 0.0)
    self.axes_arot_2 = self.read_parameter('~axes_arot_2', [0, 0, 0])
    self.angle_arot_2 = self.read_parameter('~angle_arot_2', 0.0)
    self.axes_arot_3 = self.read_parameter('~axes_arot_3', [0, 0, 0])
    self.angle_arot_3 = self.read_parameter('~angle_arot_3', 0.0)
    self.arot_is = (self.angle_arot_1 + self.angle_arot_2 + self.angle_arot_3 > 0.0)
    if (self.arot_is):
        q_a1 = tr.quaternion_about_axis(self.angle_arot_1, self.axes_arot_1)
        q_a2 = tr.quaternion_about_axis(self.angle_arot_2, self.axes_arot_2)
        q_a3 = tr.quaternion_about_axis(self.angle_arot_3, self.axes_arot_3)
        self.q_arot_value = tr.quaternion_multiply(q_a1, tr.quaternion_multiply(q_a2,q_a3))
    
    self.axes_mrot_1 = self.read_parameter('~axes_mrot_1', [0, 0, 0])
    self.angle_mrot_1 = self.read_parameter('~angle_mrot_1', 0.0)
    self.axes_mrot_2 = self.read_parameter('~axes_mrot_2', [0, 0, 0])
    self.angle_mrot_2 = self.read_parameter('~angle_mrot_2', 0.0)
    self.axes_mrot_3 = self.read_parameter('~axes_mrot_3', [0, 0, 0])
    self.angle_mrot_3 = self.read_parameter('~angle_mrot_3', 0.0)   
    self.mrot_is = (self.angle_mrot_1 + self.angle_mrot_2 + self.angle_mrot_3 > 0.0)
    if (self.mrot_is):
        q_m1 = tr.quaternion_about_axis(self.angle_mrot_1, self.axes_mrot_1)
        q_m2 = tr.quaternion_about_axis(self.angle_mrot_2, self.axes_mrot_2)
        q_m3 = tr.quaternion_about_axis(self.angle_mrot_3, self.axes_mrot_3)
        self.q_mrot_value = tr.quaternion_multiply(q_m3, tr.quaternion_multiply(q_m2,q_m1))
    
    # Initial values
    self.master_pos = None
    self.master_rot = np.array([0.0, 0.0, 0.0, 1.0])
    self.robot_pos = None
    self.robot_rot = np.array([0.0, 0.0, 0.0, 1.0])
    self.master_vel = np.array([0.0, 0.0, 0.0])
    self.master_synch_pos = np.zeros(3)
    self.robot_synch_pos = np.zeros(3)
    self.robot_synch_rot = np.array([0.0, 0.0, 0.0, 1.0])
    self.sm_control = 0.0
    self.new_sm_signal = True
    
    # Setup Subscribers/Publishers
    self.ik_mc_pub = rospy.Publisher(self.ik_mc_topic, PoseStamped)
    self.vis_pub = rospy.Publisher('visualization_marker', Marker)
    rospy.Subscriber(self.master_pose_topic, PoseStamped, self.cb_master_pose)
    rospy.Subscriber(self.master_vel_topic, TwistStamped, self.cb_master_vel)
    rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.cb_robot_pose)
    rospy.Subscriber(self.sm_control_topic, Float64, self.cb_sm_control)
    
    rospy.loginfo('Waiting for [%s] topic' % (self.master_pose_topic))
    rospy.loginfo('Waiting for [%s] topic' % (self.robot_pose_topic))
    while not rospy.is_shutdown():
      if (self.master_pos == None) or (self.robot_pos == None):
        rospy.sleep(0.01)
      else:
        rospy.loginfo('Rate position controller running')
        # Register rospy shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        break
    
    #Timer for publish ik_commands
    rospy.loginfo('Publisher frequency: [%f]' % self.publish_frequency)
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_command)
    rospy.spin()

  def change_axes(self, array, index=None, sign=None):
    if index == None:
      index = self.position_axes
    if sign == None:
      sign = self.position_sign
    result = np.zeros(len(array))
    for i, idx in enumerate(index):
      result[i] = array[idx] * sign[idx]
    return result

  def change_force_axes(self, array, index=None, sign=None):
    if index == None:
      index = self.position_axes
    if sign == None:
      sign = self.position_sign
    result = np.zeros(len(array))
    for i, idx in enumerate(index):
      result[i] = array[idx] * sign[i]
    return result

  def normalize_vector(self, v):
    result = np.array(v)
    norm = np.sqrt(np.sum((result ** 2)))
    if norm:
      result /= norm
    return result
    
  # DO NOT print to the console within this function
  def cb_master_pose(self, msg):
    pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.master_pos = self.change_axes(pos)

    real_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    if (self.arot_is):
        q1_aux = tr.quaternion_multiply(real_rot, self.q_arot_value)
    else:
        q1_aux = real_rot
        
    if (self.mrot_is):
        q2_aux = tr.quaternion_multiply(self.q_mrot_value, q1_aux)
    else:
        q2_aux = q1_aux
        
    self.master_rot = q2_aux

  def cb_master_vel(self, msg):
    vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    self.master_vel = self.change_axes(vel)
    #Normalize velocitity
    self.master_dir = self.normalize_vector(self.master_vel)
    
  def cb_robot_pose(self, msg):
    self.robot_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.robot_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

  def cb_sm_control(self,msg):
    if (msg.data != self.sm_control):
        self.new_sm_signal = True
    self.sm_control = msg.data

  def shutdown_hook(self):
    # Stop the publisher timer
    self.timer.shutdown()

  def publish_command(self, event):   
    # POSITION_CONTROL
    if (self.sm_control == 1.0):
      if (self.new_sm_signal):
        self.new_sm_signal = False
        self.robot_synch_pos = np.array(self.robot_pos)
        self.robot_synch_rot = np.array(self.robot_rot)
        self.command_pos = np.array(self.robot_pos)
        self.command_rot = np.array(self.robot_rot)
        if (self.strategy == 'bubble_rate_pos'):
            self.draw_position_region(self.robot_synch_pos)
        elif (self.strategy == 'indexing'):
			self.master_synch_pos = self.master_pos
      else:
        self.command_pos = self.robot_synch_pos + ((self.master_pos - self.master_synch_pos) * self.scale)
        self.command_rot = np.array(self.master_rot)

    # RATE_CONTROL
    elif (self.sm_control == 2.0):
      if (self.new_sm_signal):
        self.new_sm_signal = False
        self.command_pos = np.array(self.robot_pos)
        self.command_rot = np.array(self.robot_rot)
        self.robot_synch_pos = np.array(self.robot_pos)
        self.robot_synch_rot = np.array(self.robot_rot)
        self.rate_pivot = np.array(self.master_pos)
        #~ if (self.strategy == 'bubble_rate_pos'):
            #~ self.delete_position_region()
      else:
        distance = sqrt(np.sum((self.master_pos - self.rate_pivot) ** 2))* self.scale
        self.command_pos += (self.rate_gain * distance * self.normalize_vector(self.master_pos))
        self.command_rot = np.array(self.robot_synch_rot)
        
    # REST OF THEM (No master values)
    else:
      if (self.new_sm_signal):
        self.new_sm_signal = False
        self.robot_synch_pos = np.array(self.robot_pos)
        self.robot_synch_rot = np.array(self.robot_rot)
      self.command_pos = np.array(self.robot_synch_pos)
      self.command_rot = np.array(self.robot_synch_rot)     
    
    # PoseStamped message
    position, orientation = self.command_pos, self.command_rot
    ik_mc_msg = PoseStamped()
    ik_mc_msg.header.frame_id = self.ref_frame
    ik_mc_msg.header.stamp = rospy.Time.now()
    ik_mc_msg.pose.position = Point(*position)
    ik_mc_msg.pose.orientation = Quaternion(*orientation)
    self.ik_mc_pub.publish(ik_mc_msg)

  def draw_position_region(self, center_pos):
    marker = Marker()
    marker.header.frame_id = self.ref_frame
    marker.header.stamp = rospy.Time.now()
    marker.id = 0;
    marker.type = marker.SPHERE
    marker.ns = 'position_region'
    marker.action = marker.ADD
    marker.pose.position.x = center_pos[0]
    marker.pose.position.y = center_pos[1]
    marker.pose.position.z = center_pos[2]
    #~ Workspace ellipsoid: self.workspace
    marker.scale.x = 2 * self.bubble_radius
    marker.scale.y = 2 * self.bubble_radius
    marker.scale.z = 2 * self.bubble_radius
    marker.color.a = 0.5
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.2
    #~ Publish
    self.vis_pub.publish(marker)
    
  #~ def delete_position_region(self, center_pos):
    #~ marker = Marker()
    #~ marker.header.frame_id = self.ref_frame
    #~ marker.header.stamp = rospy.Time.now()
    #~ marker.id = 0;
    #~ marker.type = marker.SPHERE
    #~ marker.ns = 'position_region'
    #~ marker.action = marker.DELETE
    #~ self.vis_pub.publish(marker)
    
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

if __name__ == '__main__':
    rospy.init_node('Hybrid_Controller', log_level=rospy.WARN)
    loader = HybridController()
