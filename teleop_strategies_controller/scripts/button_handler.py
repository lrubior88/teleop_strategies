#! /usr/bin/env python

import rospy
# Messages
from std_msgs.msg import Float64, Bool
from teleop_msgs.msg import Button2, Button5

class ButtonHandler:
  def __init__(self):
      
    # Read Parameters
    master_name = self.read_parameter('~master_name', 'phantom')
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.n_button = self.read_parameter('~n_button','0')
    self.buttons_in = self.read_parameter('~buttons_in', [1, 2])
    
    self.buttons_in_topic = '/master_%s/button' % master_name
    self.buttons_out_topic = self.read_parameter('~buttons_topic', '/teleop_controller/buttons')
    
    # Setup Subscribers/Publishers
    self.buttons_pub = rospy.Publisher(self.buttons_out_topic, Button5)
    for i in range(self.n_button):
        rospy.Subscriber(self.buttons_in_topic + '_' + str(self.buttons_in[i]), Float64, self.cb_button, i)
    
    #Init values
    self.button = dict()
    for i in range(5):
        self.button[i] = 0.0
        
    #Timer for publish Button5
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.buttons_publishing)
    
    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)

    rospy.spin()
        
  def cb_button(self,msg,n):
     self.button[n] = msg.data

  def shutdown_hook(self):
    # Stop the publisher timer
    self.timer.shutdown()

  def buttons_publishing(self, event):
    # Publish buttons
    b_msg = Button5()
    b_msg.button_1 = self.button[0]
    b_msg.button_2 = self.button[1]
    b_msg.button_3 = self.button[2]
    b_msg.button_4 = self.button[3]
    b_msg.button_5 = self.button[4]
    
    self.buttons_pub.publish(b_msg) 
    
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

if __name__ == '__main__':
    rospy.init_node('Button_Handler', log_level=rospy.WARN)
    loader = ButtonHandler()
