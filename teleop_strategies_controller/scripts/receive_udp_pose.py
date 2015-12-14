#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Vector3
from omni_msgs.msg import OmniFeedback

import roslib.message

_struct_7d = struct.Struct("<7d")


class Receive_udp_pose:
  def __init__(self):
      
    # Read parameters
    self.read_ip = self.read_parameter('~read_ip', '127.0.0.1')
    self.read_port = self.read_parameter('~read_port', '34900')
    self.topic_name = self.read_parameter('~topic_name', '/master_phantom/pose')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.frame_id = self.read_parameter('~reference_frame', 'world')

    # Setup Publishers
    self.topic_pub = rospy.Publisher(self.topic_name, PoseStamped)

    # Setup read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind((self.read_ip, self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))

    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)

    while True:

        data,addr=self.read_socket.recvfrom(1024)
        if data:
			
			cmd_msg = PoseStamped()
			self.deserialize(self.ser_mode, data, cmd_msg) 
        
			(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w) = _struct_7d.unpack(data[0:56])
			
			cmd_msg.header.frame_id = self.frame_id
			cmd_msg.header.stamp = rospy.Time.now()
			cmd_msg.pose.position.x = pos_x
			cmd_msg.pose.position.y = pos_y
			cmd_msg.pose.position.z = pos_z
			cmd_msg.pose.orientation.x = rot_x
			cmd_msg.pose.orientation.y = rot_y
			cmd_msg.pose.orientation.z = rot_z
			cmd_msg.pose.orientation.w = rot_w

			self.topic_pub.publish(cmd_msg)

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

  def deserialize(self, mode, data, msg):
    if (mode == "ros"):
		
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




