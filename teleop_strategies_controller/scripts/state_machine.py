#! /usr/bin/env python

import rospy
# Messages
from std_msgs.msg import Float64, Bool
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

class TextColors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'

  def disable(self):
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''

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
      smach.StateMachine.add('DESYNCHRONIZED', CBState(self.desynchronized, cb_args=[self]),
                             transitions={'desyn': 'DESYNCHRONIZED', 'pos': 'POSITION_CONTROL', 'rate': 'RATE_CONTROL', 'aborted': 'aborted'})

    # Read all the parameters from the parameter server
    # Generic Parameters
    master_name = self.read_parameter('~master_name', 'phantom')
    robot_name = self.read_parameter('~robot_name', 'grips')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.strategy = self.read_parameter('~strategy','bubble_rate_pos')
    self.n_button = self.read_parameter('~n_button','0')
    self.scale = self.read_parameter('~scale', 4)
    
    # More Parameters
    self.hysteresis = self.read_parameter('~hysteresis', 3.0)
    self.bubble_radius = self.read_parameter('~bubble_radius', 80)
    self.vib_time = self.read_parameter('~vibration/duration', 0.3)   # Duration (s)
    
    # Topic names
    self.master_pose_topic = '/master_%s/pose' % master_name
    if (self.n_button > 0):
        self.buttons_topic = self.read_parameter('~buttons_topic', '/teleop_controller/buttons')
    self.sm_control_topic = '/teleop_controller/current_sm'
    self.robot_collision_topic = '/robot_%s/collision' % robot_name
    self.robot_gripper_topic = '/robot_%s/gripper_cmd' % robot_name
      
    # Initial values
    self.colors = TextColors()
    self.vib_start_time = 0.0
    self.master_pos = None
    self.previous_state = "center"
    self.robot_collision = False
    self.button_value = dict()
    for i in range(self.n_button):
        self.button_value[i] = 0.0
    self.activated_button = dict()
    for j in range(self.n_button):
        self.activated_button[j] = False
    
    # Subscribers and Publisher
    self.sm_control_pub = rospy.Publisher(self.sm_control_topic, Float64)
    self.robot_gripper_pub = rospy.Publisher(self.robot_gripper_topic, Float64)
    rospy.Subscriber(self.master_pose_topic, PoseStamped, self.cb_master_pose)
    rospy.Subscriber(self.robot_collision_topic, Bool, self.cb_robot_collision)
    if (self.n_button == 1):
        rospy.Subscriber(self.buttons_topic, Float64, self.cb_button1)
    elif (self.n_button == 2):
        rospy.Subscriber(self.buttons_topic, Button2, self.cb_button2)

    self.loginfo('Waiting for [%s] topic' % (self.master_pose_topic))
    while not rospy.is_shutdown():
      if (self.master_pos == None):
        rospy.sleep(0.01)
      else:
        self.loginfo('Rate position controller running')
        # Register rospy shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        break

    # Start the timer that will publish the ik commands
    self.loginfo('Publisher frequency: [%f]' % self.publish_rate)
    self.loginfo('Strategy: [%s]' % self.strategy)
    self.loginfo('State machine state: GO_TO_CENTER')
    
    
  @smach.cb_interface(outcomes=['center', 'pos', 'rate', 'aborted'])
  def go_to_center(user_data, self):
      
    if not np.allclose(np.zeros(3), self.master_pos, atol=self.hysteresis):
      self.sm_control_pub.publish(4.0)
      return 'center'
      
    else:
      if (self.strategy == "bubble_rate_pos"):
        self.sm_control_pub.publish(1.0)
        self.previous_state = "center"
        self.loginfo('State machine transitioning: GO_TO_CENTER-->POSITION_CONTROL')
        return 'pos'
        
      elif (self.strategy == "button_rate_pos"):
        if (self.previous_state == "pos"):
          self.sm_control_pub.publish(2.0)
          self.previous_state = "center"
          self.loginfo('State machine transitioning: GO_TO_CENTER-->RATE_CONTROL')
          return 'rate'
        else:
          self.sm_control_pub.publish(1.0)
          self.previous_state = "center"
          self.loginfo('State machine transitioning: GO_TO_CENTER-->POSITION_CONTROL')
          return 'pos'
            
      elif (self.strategy == "indexing"):
        self.sm_control_pub.publish(1.0)
        self.previous_state = "center"
        self.loginfo('State machine transitioning: GO_TO_CENTER-->POSITION_CONTROL')
        return 'pos'
        
      else:
        rospy.logwarn('GO_TO_CENTER in (%s) strategy it is not defined' % (self.strategy))


  @smach.cb_interface(outcomes=['pos', 'center', 'vib', 'rate', 'desyn', 'aborted'])
  def position_control(user_data, self):
      
    if (self.strategy == "bubble_rate_pos"):
      if self.inside_workspace(self.master_pos*self.scale):
        self.sm_control_pub.publish(1.0)
        return 'pos'
      else:
        self.sm_control_pub.publish(3.0)
        self.previous_state = "pos"
        self.vib_start_time = rospy.get_time()
        self.loginfo('State machine transitioning: POSITION_CONTROL-->VIBRATORY_PHASE')
        return 'vib'  
        
    elif (self.strategy == "button_rate_pos"):
      if (self.activated_button[0]):
        self.activated_button[0] = False
        self.sm_control_pub.publish(2.0)
        self.previous_state = "pos"
        self.loginfo('State machine transitioning: POSITION_CONTROL-->RATE_CONTROL')
        return 'rate'      
      else:
        self.sm_control_pub.publish(1.0)
        return 'pos'
            
    elif (self.strategy == "indexing"):
      if (self.activated_button[0]):
        self.activated_button[0] = False
        self.sm_control_pub.publish(5.0)
        self.previous_state = "pos"
        self.loginfo('State machine transitioning: POSITION_CONTROL-->DESYNCHRONIZED')
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
      if (self.strategy == "bubble_rate_pos"):
        if (self.previous_state == "collision"):
            self.sm_control_pub.publish(4.0)
            self.previous_state = "vib"
            self.loginfo('State machine transitioning: VIBRATORY_PHASE-->GO_TO_CENTER')
            return 'center'
        else:
            self.sm_control_pub.publish(2.0)
            self.previous_state = "vib"
            self.loginfo('State machine transitioning: VIBRATORY_PHASE-->RATE_CONTROL')
            return 'rate'
        
      elif (self.strategy == "button_rate_pos"):
        self.sm_control_pub.publish(4.0)
        self.previous_state = "vib"
        self.loginfo('State machine transitioning: VIBRATORY_PHASE-->GO_TO_CENTER')
        return 'center'
               
      else:
        rospy.logwarn('VIBRATORY_PHASE in (%s) strategy it is not defined' % (self.strategy))
        return 'vib'

  @smach.cb_interface(outcomes=['rate', 'center', 'collision', 'desyn', 'aborted'])
  def rate_control(user_data, self):
    if not self.robot_collision:
      if (self.strategy == "bubble_rate_pos"):
        if not self.inside_workspace(self.master_pos*self.scale):
          self.sm_control_pub.publish(2.0)
          return 'rate'
        else:
          self.sm_control_pub.publish(4.0)
          self.previous_state = "rate"
          self.loginfo('State machine transitioning: RATE_CONTROL-->GO_TO_CENTER')
          return 'center'  
        
      elif (self.strategy == "button_rate_pos"):
        if (self.activated_button[0]):
          self.activated_button[0] = False
          self.sm_control_pub.publish(1.0)
          self.previous_state = "rate"
          self.loginfo('State machine transitioning: RATE_CONTROL-->POSITION_CONTROL')
          return 'pos'      
        else:
          self.sm_control_pub.publish(2.0)
          return 'rate'
          
      else:
        rospy.logwarn('RATE_CONTROL in (%s) strategy it is not defined' % (self.strategy))
        self.sm_control_pub.publish(2.0)
        return 'rate'

    else:
      self.sm_control_pub.publish(6.0)
      self.previous_state = "rate"
      self.loginfo('State machine transitioning: RATE_CONTROL->RATE_COLLISION')     
      return 'collision'

  @smach.cb_interface(outcomes=['center', 'vib', 'aborted'])
  def rate_collision(user_data, self):
    self.sm_control_pub.publish(3.0)
    self.previous_state = "vib"
    self.vib_start_time = rospy.get_time()
    self.loginfo('State machine transitioning: RATE_COLLISION-->VIBRATORY_PHASE')
    return 'vib' 

  @smach.cb_interface(outcomes=['desyn', 'pos', 'rate', 'aborted'])
  def desynchronized(user_data, self):
    if (self.strategy == "indexing"):
      if (self.activated_button[0]):
        self.activated_button[0] = False
        self.sm_control_pub.publish(1.0)
        self.previous_state = "desyn"
        self.loginfo('State machine transitioning: DESYNCHRONIZED-->POSITION_CONTROL')
        return 'pos'      
      else:
        self.sm_control_pub.publish(5.0)
        return 'desyn'
    else:
      rospy.logwarn('DESYNCHRONIZED in (%s) strategy it is not defined' % (self.strategy))
      self.sm_control_pub.publish(5.0)
      return 'desyn' 

  def execute(self):
    self.sm.execute()

  def shutdown_hook(self):
    # Stop the state machine
    self.sm.request_preempt()

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

  def inside_workspace(self, point):
    return sqrt(np.sum(point**2)) < self.bubble_radius

  # DO NOT print to the console within this function
  def cb_master_pose(self, msg):
    self.master_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

  def cb_robot_collision(self, msg):
    self.robot_collision = msg.data
    
  def cb_button1(self,msg):
    button_value = msg.data
    if ((button_value != 0.0) and (button_value != self.button_value[0])):
        self.activated_button[0] = True
    self.button_value[0] = button_value

  def cb_button2(self,msg):
    button_value_1 = msg.button_1
    button_value_2 = msg.button_2
    if ((button_value_1 != 0.0) and (button_value_1 != self.button_value[0])):
        self.activated_button[0] = True
    self.button_value[0] = button_value_1   
    if ((button_value_2 != 0.0) and (button_value_2 != self.button_value[0])):
        self.activated_button[1] = True
    self.button_value[1] = button_value_2

  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)

if __name__ == '__main__':
  rospy.init_node('state_machine', log_level=rospy.WARN)
  try:
    controller = StateMachine()
    controller.execute()
  except rospy.exceptions.ROSInterruptException:
      pass
