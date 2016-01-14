#! /usr/bin/env python
import rospy, time, math, os
import numpy as np
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped

import roslib.message

_struct_d = dict()
for i in range(10):
  _struct_d[i] = struct.Struct("<" + str(i+1) + "d")


class UDP_receive_info:
  def __init__(self):
      
    # Read parameters
    self.read_ip = self.read_parameter('~read_ip', '127.0.0.1')
    self.read_port = self.read_parameter('~read_port', '34900')
    self.type_info = self.read_parameter('~type_info', 'pose')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.ref_frame = self.read_parameter('~ref_frame', 'world')
    self.units = self.read_parameter('~units', 'm')
    
    if (self.units == "mm"):
        self.unit_scale = 0.001
    elif (self.units == "cm"):
        self.unit_scale = 0.01
    elif (self.units == "dm"):
        self.unit_scale = 0.1
    else:
        self.unit_scale = 1
    
    if (self.type_info == "pose"):
        self.pose_topic = self.read_parameter('~pose_topic', '/master_phantom/pose')
        self.pose_topic_pub = rospy.Publisher(self.pose_topic, PoseStamped)
    elif (self.type_info == "pose_vel"):
        self.pose_topic = self.read_parameter('~pose_topic', '/master_phantom/pose')
        self.vel_topic = self.read_parameter('~vel_topic', '/master_phantom/velocity')
        self.pose_topic_pub = rospy.Publisher(self.pose_topic, PoseStamped)
        self.vel_topic_pub = rospy.Publisher(self.vel_topic, TwistStamped)
    elif (self.type_info.split('_')[0] == "button"):
        self.button_topic = self.read_parameter('~button_topic', '/master_phantom/button')
        n = int(self.type_info.split('_')[1])
        self.button_topic_pub = dict()
        for i in range (n):
            self.button_topic_pub[i] = rospy.Publisher(self.button_topic + '_' + str(i+1), Float64)
    elif (self.type_info.split('_')[0] == "force"):
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
            
                (pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w) = _struct_d[6].unpack(data[0:56])
                
                cmd_msg = PoseStamped()
                cmd_msg.header.frame_id = self.ref_frame
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.pose.position.x = pos_x * self.unit_scale
                cmd_msg.pose.position.y = pos_y * self.unit_scale
                cmd_msg.pose.position.z = pos_z * self.unit_scale
                cmd_msg.pose.orientation.x = rot_x
                cmd_msg.pose.orientation.y = rot_y
                cmd_msg.pose.orientation.z = rot_z
                cmd_msg.pose.orientation.w = rot_w
                self.pose_topic_pub.publish(cmd_msg)
                
            elif (self.type_info == "pose_vel"):
            
                (pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w, vel_x, vel_y, vel_z) = _struct_d[9].unpack(data[0:80])
                
                cmd_msg = PoseStamped()
                cmd_msg.header.frame_id = self.ref_frame
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.pose.position.x = pos_x * self.unit_scale
                cmd_msg.pose.position.y = pos_y * self.unit_scale
                cmd_msg.pose.position.z = pos_z * self.unit_scale
                cmd_msg.pose.orientation.x = rot_x
                cmd_msg.pose.orientation.y = rot_y
                cmd_msg.pose.orientation.z = rot_z
                cmd_msg.pose.orientation.w = rot_w
                self.pose_topic_pub.publish(cmd_msg)
                
                vel_msg = TwistStamped()
                vel_msg.header.frame_id = self.ref_frame
                vel_msg.header.stamp = cmd_msg.header.stamp
                vel_msg.twist.linear.x = vel_x * self.unit_scale
                vel_msg.twist.linear.y = vel_y * self.unit_scale
                vel_msg.twist.linear.z = vel_z * self.unit_scale
                self.vel_topic_pub.publish(vel_msg)
                
            elif (self.type_info.split('_')[0] == "button"):
                n = int(self.type_info.split('_')[1])
                bits = 8*n
                
                buttons = _struct_d[n-1].unpack(data[0:bits])
        
                for i in range(n):
                    self.button_topic_pub[i].publish(Float64(buttons[i]))
                
            elif (self.type_info.split('_')[0] == "force"):
                n = int(self.type_info.split('_')[1])
                bits = 8*n
                forces = _struct_d[n-1].unpack(data[0:bits])
                
                f_msg = WrenchStamped()
                f_msg.header.frame_id = self.ref_frame
                f_msg.header.stamp = rospy.Time.now()
                f_msg.wrench.force.x = forces[0] * self.unit_scale
                f_msg.wrench.force.y = forces[1] * self.unit_scale
                f_msg.wrench.force.z = forces[2] * self.unit_scale
                if (n == 6):
                    f_msg.wrench.torque.x = forces[3] * self.unit_scale
                    f_msg.wrench.torque.y = forces[4] * self.unit_scale
                    f_msg.wrench.torque.z = forces[5] * self.unit_scale             
                
                self.force_topic_pub.publish(f_msg)
                
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
        
  def shutdown_hook(self):
    # Do some cleaning depending on the app
    self.read_socket.close()
    pass

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = UDP_receive_info()
