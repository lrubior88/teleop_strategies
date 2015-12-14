#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from teleop_msgs.msg import Button5

import roslib.message

_struct_1d = struct.Struct("<1d")
_struct_2d = struct.Struct("<2d")
_struct_3d = struct.Struct("<3d")
_struct_4d = struct.Struct("<4d")
_struct_5d = struct.Struct("<5d")
_struct_6d = struct.Struct("<6d")
_struct_7d = struct.Struct("<7d")
_struct_10d = struct.Struct("<10d")


class Receive_udp_pose:
  def __init__(self):
      
    # Read parameters
    self.read_ip = self.read_parameter('~read_ip', '127.0.0.1')
    self.read_port = self.read_parameter('~read_port', '34900')
    self.type_info = self.read_parameter('~type_info', 'pose')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.frame_id = self.read_parameter('~reference_frame', 'world')
    
    if (self.type_info == "pose"):
        self.pose_topic = self.read_parameter('~pose_topic', '/master_phantom/pose')
        self.pose_topic_pub = rospy.Publisher(self.pose_topic, PoseStamped)
    elif (self.type_info == "pose_vel"):
        self.pose_topic = self.read_parameter('~pose_topic', '/master_phantom/pose')
        self.vel_topic = self.read_parameter('~vel_topic', '/master_phantom/velocity')
        self.pose_topic_pub = rospy.Publisher(self.pose_topic, PoseStamped)
        self.vel_topic_pub = rospy.Publisher(self.vel_topic, TwistStamped)
    elif ((self.type_info == "button1")or(self.type_info == "button2")or(self.type_info == "button3")or(self.type_info == "button4")or(self.type_info == "button5")):
        self.button_topic = self.read_parameter('~button_topic', '/master_phantom/button')
        self.button_topic_pub = rospy.Publisher(self.button_topic, Button5)
    elif ((self.type_info == "force3")or(self.type_info == "force6")):
        self.force_topic = self.read_parameter('~force_topic', '/master_phantom/force')
        self.force_topic_pub = rospy.Publisher(self.force_topic, WrenchStamped)
    else:
        rospy.logwarn('Type_info: [%s] not found' % (self.type_info))
    

    # Setup read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind((self.read_ip, self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))

    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)

    while True:

        data,addr=self.read_socket.recvfrom(1024)
        if data:
            
            if (self.type_info == "pose"):
            
                (pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w) = _struct_7d.unpack(data[0:56])
                
                cmd_msg = PoseStamped()
                cmd_msg.header.frame_id = self.frame_id
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.pose.position.x = pos_x
                cmd_msg.pose.position.y = pos_y
                cmd_msg.pose.position.z = pos_z
                cmd_msg.pose.orientation.x = rot_x
                cmd_msg.pose.orientation.y = rot_y
                cmd_msg.pose.orientation.z = rot_z
                cmd_msg.pose.orientation.w = rot_w
                self.pose_topic_pub.publish(cmd_msg)
                
            elif (self.type_info == "pose_vel"):
            
                (pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w, vel_x, vel_y, vel_z) = _struct_10d.unpack(data[0:80])
                
                cmd_msg = PoseStamped()
                cmd_msg.header.frame_id = self.frame_id
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.pose.position.x = pos_x
                cmd_msg.pose.position.y = pos_y
                cmd_msg.pose.position.z = pos_z
                cmd_msg.pose.orientation.x = rot_x
                cmd_msg.pose.orientation.y = rot_y
                cmd_msg.pose.orientation.z = rot_z
                cmd_msg.pose.orientation.w = rot_w
                self.pose_topic_pub.publish(cmd_msg)
                
                vel_msg = TwistStamped()
                vel_msg.header.frame_id = self.frame_id
                vel_msg.header.stamp = cmd_msg.header.stamp
                vel_msg.twist.linear.x = vel_x
                vel_msg.twist.linear.y = vel_y
                vel_msg.twist.linear.z = vel_z
                self.vel_topic_pub.publish(vel_msg)
                
            elif (self.type_info == "button1"): 
                
                (but_1) = _struct_1d.unpack(data[0:8])
                but_msg = Button5()
                but_msg.n = 1
                but_msg.button_1 = but_1
                
            elif (self.type_info == "button2"): 
                
                (but_1, but2) = _struct_2d.unpack(data[0:16])
                but_msg = Button5()
                but_msg.n = 2
                but_msg.button_1 = but_1
                but_msg.button_2 = but_2
                
            elif (self.type_info == "button3"): 
                
                (but_1, but2, but3) = _struct_3d.unpack(data[0:24])
                but_msg = Button5()
                but_msg.n = 3
                but_msg.button_1 = but_1
                but_msg.button_2 = but_2
                but_msg.button_3 = but_3
                
            elif (self.type_info == "button4"): 
                
                (but_1, but2, but3, but4) = _struct_4d.unpack(data[0:32])
                but_msg = Button5()
                but_msg.n = 4
                but_msg.button_1 = but_1
                but_msg.button_2 = but_2
                but_msg.button_3 = but_3
                but_msg.button_4 = but_4
                
            elif (self.type_info == "button5"): 
                
                (but_1, but2, but3, but4, but5) = _struct_5d.unpack(data[0:40])
                but_msg = Button5()
                but_msg.n = 5
                but_msg.button_1 = but_1
                but_msg.button_2 = but_2
                but_msg.button_3 = but_3
                but_msg.button_4 = but_4
                but_msg.button_5 = but_5
                
            elif (self.type_info == "force3"):
                
                (f_x, f_y, f_z) = _struct_3d.unpack(data[0:24])
                f_msg = WrenchStamped()
                f_msg.header.frame_id = self.frame_id
                f_msg.header.stamp = rospy.Time.now()
                f_msg.wrench.force.x = f_x
                f_msg.wrench.force.y = f_y
                f_msg.wrench.force.z = f_z
                
            elif (self.type_info == "force6"):
                
                (f_x, f_y, f_z, tau_x, tau_y, tau_z) = _struct_6d.unpack(data[0:48])
                f_msg = WrenchStamped()
                f_msg.header.frame_id = self.frame_id
                f_msg.header.stamp = rospy.Time.now()
                f_msg.wrench.force.x = f_x
                f_msg.wrench.force.y = f_y
                f_msg.wrench.force.z = f_z
                f_msg.wrench.torque.x = tau_x
                f_msg.wrench.torque.y = tau_y
                f_msg.wrench.torque.z = tau_z               
                
                
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
        
  #~ def loginfo(self, msg):
    #~ rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)

  def shutdown_hook(self):
    # Do some cleaning depending on the app
    self.read_socket.close()
    pass

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = Receive_udp_pose()
