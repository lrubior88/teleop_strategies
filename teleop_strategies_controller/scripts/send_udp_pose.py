#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

_struct_7d = struct.Struct("<7d")

class Send_udp_pose:
  def __init__(self):
      
    # Read parameters
    self.write_ip = self.read_parameter('~write_ip', '127.0.0.1')
    self.write_port = self.read_parameter('~write_port', '34900')
    self.topic_name = self.read_parameter('~topic_name', '/phantom/pose')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
      
    # Setup Subscriber
    rospy.Subscriber(self.topic_name, PoseStamped, self.pose_command_cb)

    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rospy.spin()

  def pose_command_cb(self, msg):
    try:
        # Serialize a PoseStamped msg
        buff = StringIO()
        buff.write(_struct_7d.pack(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

        # Send values
        self.write_socket.sendto(buff.getvalue(), (self.write_ip, self.write_port))
   
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)
    except socket.error, e:
        result = self.write_socket.bind((self.write_ip,self.write_port))
        if result:
            rospy.logwarn('Connection refused')

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = Send_udp_pose()
