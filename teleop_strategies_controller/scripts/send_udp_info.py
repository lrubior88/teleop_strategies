#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped

_struct_d = dict()
for i in range(10):
  _struct_d[i] = struct.Struct("<" + str(i+1) + "d")

class Send_udp_info:
  def __init__(self):
      
    # Read parameters
    self.write_ip = self.read_parameter('~write_ip', '127.0.0.1')
    self.write_port = self.read_parameter('~write_port', '34900')
    self.type_info = self.read_parameter('~type_info', 'pose')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    
    rospy.loginfo('UDP Socket writing on port [%d]' % (self.write_port))
    self.pose_msg = None
    self.vel_msg = None
    self.button_msg = None
    self.force_msg = None
    
    if (self.type_info == "pose"):
        self.pose_topic = self.read_parameter('~pose_topic', '/master_phantom/pose')
        rospy.Subscriber(self.pose_topic_name, PoseStamped, self.pose_command_cb)
        while not rospy.is_shutdown():
            if (self.pose_msg == None):
                rospy.sleep(0.01)
            else:
                break
    elif (self.type_info == "pose_vel"):
        self.pose_topic = self.read_parameter('~pose_topic', '/master_phantom/pose')
        self.vel_topic = self.read_parameter('~vel_topic', '/master_phantom/velocity')
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_command_cb)        
        rospy.Subscriber(self.vel_topic, TwistStamped, self.vel_command_cb)
        while not rospy.is_shutdown():
            if (self.pose_msg == None) or (self.vel_msg == None):
                rospy.sleep(0.01)
            else:
                break
    elif (self.type_info.split('_')[0] == "button"):
        self.button_topic = self.read_parameter('~button_topic', '/master_phantom/button')
        rospy.Subscriber(self.button_topic, Float64, self.button_command_cb)
        while not rospy.is_shutdown():
            if (self.button_msg == None):
                rospy.sleep(0.01)
            else:
                break
    elif (self.type_info.split('_')[0] == "force"):
        self.force_topic = self.read_parameter('~force_topic', '/master_phantom/force_feedback')
        rospy.Subscriber(self.force_topic, WrenchStamped, self.force_command_cb)
        while not rospy.is_shutdown():
            if (self.force_msg == None):
                rospy.sleep(0.01)
            else:
                break
    else:
        rospy.logwarn('Type_info: [%s] not found' % (self.type_info))

    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.send_udp)
    rospy.spin()

  def pose_command_cb(self, msg):
    self.pose_msg = msg

  def vel_command_cb(self, msg):
    self.vel_msg = msg

  def button_command_cb(self, msg):
    self.button_msg = msg

  def force_command_cb(self, msg):
    self.force_msg = msg
    
  def send_udp(self, event):
    try:
        buff = StringIO()
        
        if (self.type_info == "pose"):
            
            # Serialize a PoseStamp msg
            pos_x = self.pose_msg.pose.position.x
            pos_y = self.pose_msg.pose.position.y 
            pos_z = self.pose_msg.pose.position.z 
            rot_x = self.pose_msg.pose.orientation.x
            rot_y = self.pose_msg.pose.orientation.y
            rot_z = self.pose_msg.pose.orientation.z
            rot_w = self.pose_msg.pose.orientation.w
            
            buff.write(_struct_d[6].pack(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w))

        elif (self.type_info == "pose_vel"):
            
            # Serialize a PoseStamp msg
            pos_x = self.pose_msg.pose.position.x
            pos_y = self.pose_msg.pose.position.y 
            pos_z = self.pose_msg.pose.position.z 
            rot_x = self.pose_msg.pose.orientation.x
            rot_y = self.pose_msg.pose.orientation.y
            rot_z = self.pose_msg.pose.orientation.z
            rot_w = self.pose_msg.pose.orientation.w
            
            # Serialize a TwistStamp msg
            vel_x = self.vel_msg.twist.linear.x
            vel_y = self.vel_msg.twist.linear.y 
            vel_z = self.vel_msg.twist.linear.z 
            
            buff.write(_struct_d[9].pack(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w, vel_x, vel_y, vel_z))
                
        elif (self.type_info.split('_')[0] == "button"):
            
            buff.write(_struct_d[0].pack(self.button_msg))
                
        elif (self.type_info.split('_')[0] == "force"):
            
            n = int(self.type_info.split('_')[1])
            rospy.loginfo('n: %d',(n))
            
            # Serialize a WrenchStamp msg
            force_x = self.force_msg.wrench.force.x
            force_y = self.force_msg.wrench.force.y 
            force_z = self.force_msg.wrench.force.z 
            torque_x = self.force_msg.wrench.torque.x 
            torque_y = self.force_msg.wrench.torque.y 
            torque_z = self.force_msg.wrench.torque.z
            
            if (n==6):
                buff.write(_struct_d[5].pack(force_x, force_y, force_z, torque_x, torque_y, torque_z))
            else:
                buff.write(_struct_d[2].pack(force_x, force_y, force_z))
        
        # Send Buffer value
        rospy.logwarn('value: %s',(buff.getvalue()))
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
  interface = Send_udp_info()
