#! /usr/bin/env python

import rospy
# Messages
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
#~ from geometry_msgs.msg import Vector3, Point, Quaternion, Transform
from teleop_msgs.msg import Button2

# State Machine
import smach
import smach_ros
from smach import CBState
# Math
from math import pi, exp, sin, sqrt
import numpy as np
# Quaternions tools
import PyKDL
import time

## STATES NUMBERS
# 'pos'         ->  1.0
# 'rate'        ->  2.0
# 'vib'         ->  3.0
# 'center'      ->  4.0
# 'desyn'       ->  5.0
# 'collision'   ->  6.0


class StateMachine:
  STATES = ['GO_TO_CENTER', 'POSITION_CONTROL', 'VIBRATORY_PHASE', 'RATE_CONTROL', 'RATE_COLLISION', 'DESYNCHRONIZED']
  def __init__(self):
    # Create a SMACH state machine
    self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with self.sm:
      # Add states to the state machine
      smach.StateMachine.add('GO_TO_CENTER', CBState(self.go_to_center, cb_args=[self]),
                             transitions={'center': 'GO_TO_CENTER', 'pos': 'POSITION_CONTROL', 'rate': 'RATE_CONTROL', 'aborted': 'aborted'})
      smach.StateMachine.add('POSITION_CONTROL', CBState(self.position_control, cb_args=[self]),
                             transitions={'pos': 'POSITION_CONTROL', 'center': 'GO_TO_CENTER', 'vib': 'VIBRATORY_PHASE', 'rate': 'RATE_CONTROL',
                                            'desyn': 'DESYNCHRONIZED', 'aborted': 'aborted'})
      smach.StateMachine.add('VIBRATORY_PHASE', CBState(self.vibratory_phase, cb_args=[self]),
                             transitions={'vib': 'VIBRATORY_PHASE', 'rate': 'RATE_CONTROL', 'pos': 'POSITION_CONTROL', 'center': 'GO_TO_CENTER', 'aborted': 'aborted'})
      smach.StateMachine.add('RATE_CONTROL', CBState(self.rate_control, cb_args=[self]),
                             transitions={'rate': 'RATE_CONTROL', 'center': 'GO_TO_CENTER', 'collision': 'RATE_COLLISION', 'desyn': 'DESYNCHRONIZED', 'aborted': 'aborted'})
      smach.StateMachine.add('RATE_COLLISION', CBState(self.rate_collision, cb_args=[self]),
                             transitions={'center': 'GO_TO_CENTER', 'vib': 'VIBRATORY_PHASE', 'aborted': 'aborted'})
      smach.StateMachine.add('DESYNCHRONIZED', CBState(self.rate_collision, cb_args=[self]),
                             transitions={'desyn': 'DESYNCHRONIZED', 'pos': 'POSITION_CONTROL', 'rate': 'RATE_CONTROL', 'aborted': 'aborted'})

    # Read all the parameters from the parameter server
    # Generic Parameters
    master_name = self.read_parameter('~master_name', 'phantom')
    robot_name = self.read_parameter('~robot_name', 'grips')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.ref_frame = self.read_parameter('~ref_frame', 'world')
    self.strategy = self.read_parameter('~strategy','bubble_rate_pos')
    self.n_button = self.read_parameter('~n_button','2')
    
    # More Parameters
    self.hysteresis = self.read_parameter('~hysteresis', 3.0)
    self.bubble_radius = self.read_parameter('~bubble_radius', 80)
    self.vib_time = self.read_parameter('~vibration/duration', 0.3)   # Duration (s)
    
    # Topic names
    self.master_pose_topic = '/%s/pose' % master_name
    if (self.n_button > 0):
        self.buttons_topic = self.read_parameter('~buttons_topic', '/teleop_controller/buttons')
    self.sm_control_topic = '/teleop_controller/current_sm'
    self.robot_gripper_topic = '/%s/gripper_cmd' % robot_name
    
    
    # Initial values
    self.vib_start_time = 0.0
    self.master_pos = None
    self.previous_state = "center"
    self.activated_button = dict{}
    for in in range(self.n_button):
		self.activated_button[i] = False
    

    # Subscribers and Publisher
    self.sm_control_pub = rospy.Publisher(self.sm_control_topic, Float64)
    self.robot_gripper_pub = rospy.Publisher(robot_gripper_topic, Float64)
    rospy.Subscriber(self.master_pose_topic, PoseStamped, self.cb_master_pose)
    if (self.n_button == 1):
        rospy.Subscriber(self.buttons_topic, Float64, self.cb_buttons)
    elif (self.n_button == 2):
        rospy.Subscriber(self.buttons_topic, Button2, self.cb_buttons)

    rospy.loginfo('Waiting for [%s] topic' % (self.master_pose_topic))
    while not rospy.is_shutdown():
      if (self.master_pos == None):
        rospy.sleep(0.01)
      else:
        rospy.loginfo('Rate position controller running')
        # Register rospy shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        break






    #~ self.center_pos = np.array([0, 0, 0])
    #~ self.colors = TextColors()
    #~ self.gripper_cmd = 0.0
    #~ self.master_pos = None
    #~ self.master_rot = np.array([0, 0, 0, 1])
    #~ self.master_vel = np.zeros(3)
    #~ self.master_dir = np.zeros(3)
    #~ self.slave_pos = None
    #~ self.slave_rot = np.array([0, 0, 0, 1])
    #~ self.slave_collision = False
    #~ self.control_desired = 1.0
    #~ ### <FORCE COUPLED>
    #~ self.force_feedback = np.zeros(3)
    #~ self.ext_forces = np.zeros(3)

    # Synch
    #~ self.slave_synch_pos = np.zeros(3)
    #~ self.slave_synch_rot = np.array([0, 0, 0, 1])
    #~ self.master_synch_rot = np.array([0, 0, 0, 1])



    # Start the timer that will publish the ik commands
    rospy.loginfo('Publisher frequency: [%f]' % self.publish_frequency)
    rospy.loginfo('State machine state: GO_TO_CENTER')
    
    
  @smach.cb_interface(outcomes=['center', 'pos', 'rate', 'aborted'])
  def go_to_center(user_data, self):
      
    if not np.allclose(np.zeros(3), self.master_pos, atol=self.hysteresis):
      self.sm_control_pub.publish(4.0)
      return 'center'
      
    else:
      if (self.strategy = "bubble_rate_pos"):
        self.sm_control_pub.publish(1.0)
        self.previous_state = "center"
        rospy.loginfo('State machine transitioning: GO_TO_CENTER-->POSITION_CONTROL')
        return 'pos'
        
      elif (self.strategy = "button_rate_pos"):
        if (self.previous_state == "pos"):
          self.sm_control_pub.publish(2.0)
          self.previous_state = "center"
          rospy.loginfo('State machine transitioning: GO_TO_CENTER-->RATE_CONTROL')
          return 'rate'
        else:
          self.sm_control_pub.publish(1.0)
          self.previous_state = "center"
          rospy.loginfo('State machine transitioning: GO_TO_CENTER-->POSITION_CONTROL')
          return 'pos'
            
      elif (self.strategy = "indexing"):
        self.sm_control_pub.publish(1.0)
        self.previous_state = "center"
        rospy.loginfo('State machine transitioning: GO_TO_CENTER-->POSITION_CONTROL')
        return 'pos'
        
      else:
        rospy.logwarn('GO_TO_CENTER in (%s) strategy it is not defined' % (self.strategy))


  @smach.cb_interface(outcomes=['pos', 'center', 'vib', 'rate', 'desyn', 'aborted'])
  def position_control(user_data, self):
      
    if (self.strategy = "bubble_rate_pos"):
      if self.inside_workspace(self.master_pos):
        self.sm_control_pub.publish(1.0)
        return 'pos'
      else:
        self.sm_control_pub.publish(3.0)
        self.previous_state = "pos"
        self.vib_start_time = rospy.get_time()
        rospy.loginfo('State machine transitioning: POSITION_CONTROL-->VIBRATORY_PHASE')
        return 'vib'  
        
    elif (self.strategy = "button_rate_pos"):
      if (self.self.activated_button[0]):
        self.sm_control_pub.publish(2.0)
        self.previous_state = "pos"
        rospy.loginfo('State machine transitioning: POSITION_CONTROL-->RATE_CONTROL')
        return 'rate'      
      else:
        self.sm_control_pub.publish(1.0)
        return 'pos'
            
    elif (self.strategy = "indexing"):
      if (self.self.activated_button[0]):
        self.sm_control_pub.publish(5.0)
        self.previous_state = "pos"
        rospy.loginfo('State machine transitioning: POSITION_CONTROL-->DESYNCHRONIZED')
        return 'desyn'      
      else:
        self.sm_control_pub.publish(1.0)
        return 'pos'
    else:
      rospy.logwarn('POSITION_CONTROL in (%s) strategy it is not defined' % (self.strategy))
      self.sm_control_pub.publish(1.0)
      return 'pos'


  @smach.cb_interface(outcomes=['vib', 'rate', 'pos', 'center', 'aborted'])
  def vibratory_phase(user_data, self):
    if rospy.get_time() < self.vib_start_time + self.vib_time:
      self.sm_control_pub.publish(3.0)
      return 'vib'

    else:
      if (self.strategy = "bubble_rate_pos"):
        self.sm_control_pub.publish(2.0)
        self.previous_state = "vib"
        rospy.loginfo('State machine transitioning: VIBRATORY_PHASE-->POSITION_CONTROL')
        return 'rate'
        
      elif (self.strategy = "button_rate_pos"):
        self.sm_control_pub.publish(2.0)
        self.previous_state = "vib"
        rospy.loginfo('State machine transitioning: VIBRATORY_PHASE-->GO_TO_CENTER')
        return 'center'
               
      else:
        rospy.logwarn('VIBRATORY_PHASE in (%s) strategy it is not defined' % (self.strategy))
        return 'vib'

  @smach.cb_interface(outcomes=['rate', 'center', 'collision', 'desyn', 'aborted'])
  def rate_control(user_data, self):
    if not self.slave_collision:
      if (self.strategy = "bubble_rate_pos"):
        if not self.inside_workspace(self.master_pos):
          self.sm_control_pub.publish(2.0)
          return 'rate'
        else:
          self.sm_control_pub.publish(4.0)
          self.previous_state = "rate"
          rospy.loginfo('State machine transitioning: RATE_CONTROL-->GO_TO_CENTER')
          return 'vib'  
        
      elif (self.strategy = "button_rate_pos"):
        if (self.self.activated_button[0]):
          self.sm_control_pub.publish(2.0)
          self.previous_state = "pos"
          rospy.loginfo('State machine transitioning: RATE_CONTROL-->RATE_CONTROL')
          return 'rate'      
        else:
          self.sm_control_pub.publish(1.0)
          return 'pos'
            
      elif (self.strategy = "indexing"):
        if (self.self.activated_button[0]):
          self.sm_control_pub.publish(5.0)
          self.previous_state = "pos"
          rospy.loginfo('State machine transitioning: RATE_CONTROL-->DESYNCHRONIZED')
          return 'desyn'      
        else:
          self.sm_control_pub.publish(1.0)
          return 'pos'
      else:
        rospy.logwarn('RATE_CONTROL in (%s) strategy it is not defined' % (self.strategy))
        self.sm_control_pub.publish(1.0)
        return 'pos'

    else:
      self.sm_control_pub.publish(6.0)
      rospy.loginfo('State machine transitioning: RATE_CONTROL->RATE_COLLISION')
        
      return 'collision'

  @smach.cb_interface(outcomes=['succeeded', 'aborted'])
  def rate_collision(user_data, self):
    self.control_desired = 1.0
    self.loginfo('State machine transitioning: RATE_COLLISION:succeeded-->GO_TO_CENTER')

    return 'succeeded'

  def execute(self):
    self.sm.execute()

  def shutdown_hook(self):
    # Stop the state machine
    self.sm.request_preempt()
    # Stop the publisher timer
    self.timer.shutdown()

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)


  def normalize_vector(self, v):
    result = np.array(v)
    norm = np.sqrt(np.sum((result ** 2)))
    if norm:
      result /= norm
    return result

  def inside_workspace(self, point):
    return math.sqrt(np.sum(point**2)) < self.bubble_radius


  # DO NOT print to the console within this function
  def cb_master_state(self, msg):
    self.master_real_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) - self.center_pos
    vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
    self.master_pos = self.change_axes(pos)
    self.master_vel = self.change_axes(vel)
    
    real_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    #~ # Synchronize rotation axis
    if (self.angle_arot_1 == 0.0):
        aux_arot_1 = real_rot
    else:
        q_a1 = tr.quaternion_about_axis(self.angle_arot_1, self.axes_arot_1)
        aux_arot_1 = tr.quaternion_multiply(real_rot, q_a1)
    
    if (self.angle_arot_2 == 0.0):
        aux_arot_2 = aux_arot_1
    else:
        q_a2 = tr.quaternion_about_axis(self.angle_arot_2, self.axes_arot_2)
        aux_arot_2 = tr.quaternion_multiply(aux_arot_1, q_a2)

    if (self.angle_arot_3 == 0.0):
        aux_arot_3 = aux_arot_2
    else:
        q_a3 = tr.quaternion_about_axis(self.angle_arot_3, self.axes_arot_3)
        aux_arot_3 = tr.quaternion_multiply(aux_arot_2, q_a3)
        
    # Synchronize rotation movement
    if (self.angle_mrot_1 == 0.0):
        aux_mrot_1 = aux_arot_3
    else:
        q_m1 = tr.quaternion_about_axis(self.angle_mrot_1, self.axes_mrot_1)
        aux_mrot_1 = tr.quaternion_multiply(q_m1, aux_arot_3)
    
    if (self.angle_mrot_2 == 0.0):
        aux_mrot_2 = aux_mrot_1
    else:
        q_m2 = tr.quaternion_about_axis(self.angle_mrot_2, self.axes_mrot_2)
        aux_mrot_2 = tr.quaternion_multiply(q_m2, aux_mrot_1)

    if (self.angle_mrot_3 == 0.0):
        aux_mrot_3 = aux_mrot_2
    else:
        q_m3 = tr.quaternion_about_axis(self.angle_mrot_3, self.axes_mrot_3)
        aux_mrot_3 = tr.quaternion_multiply(q_m3, aux_mrot_2)    
    
    self.master_rot = aux_mrot_3


    #Normalize velocitity
    self.master_dir = self.normalize_vector(self.master_vel)

  def cb_slave_state(self, msg):
    self.slave_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.slave_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

  def cb_control_desired(self,msg):
    self.control_desired = msg.data

  def cb_slave_collision(self, msg):
    self.slave_collision = msg.data

  ### <FORCE COUPLED>
  def cb_ext_forces(self, msg):
    self.ext_forces = self.change_force_axes(np.array([msg.force.x, msg.force.y, msg.force.z]))

  def publish_command(self, event):

    position, orientation = self.command_pos, self.command_rot
    ik_mc_msg = PoseStamped()
    ik_mc_msg.header.frame_id = self.frame_id
    ik_mc_msg.header.stamp = rospy.Time.now()
    ik_mc_msg.pose.position = Point(*position)
    ik_mc_msg.pose.orientation = Quaternion(*orientation)

    try:
      self.ik_mc_pub.publish(ik_mc_msg)
      self.send_feedback()
    except rospy.exceptions.ROSException:
      pass
      


if __name__ == '__main__':
  rospy.init_node('state_machine', log_level=rospy.WARN)
  try:
    controller = StateMachine()
    controller.execute()
  except rospy.exceptions.ROSInterruptException:
      pass
