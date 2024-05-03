#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

class JoyToTwistCmd:
    """
    Input topic: joy
    Output topic: ackermann_cmd(ackermann_msgs/AckermannDriveStamped)
    """
    def __init__(self):
        """
        Initialize the node.
        Subscriber: 
            joy (sensor_msgs/Joy)
        Publisher: 
            ackermann_cmd(ackermann_msgs/AckermannDriveStamped)
        """
        self.declare_params()
        self.joy_topic_name = self.get_param('joy_topic')
        self.twist_topic_name = self.get_param('twist_topic')
        self.drive_index = self.get_param('drive_index')
        self.steering_index = self.get_param('steering_index')
        self.max_angular_velocity = self.get_param('max_angular_velocity')
        self.max_velocity = self.get_param('max_velocity')

        self.sub = rospy.Subscriber(self.joy_topic_name, Joy, self.joy_callback)
        self.pub = rospy.Publisher(self.twist_topic_name, Twist, queue_size=1)
        self.twist_cmd = Twist()
    
    def get_param(self, name:str):
        """
        Get rosparam.
        Args:
            name (str): rosparam name
        """
        return rospy.get_param("/"+name)
    
    def declare_params(self)->None:
        """
        Initialize rosparam.
        """
        parameters = {
            'joy_topic': 'joy',
            'twist_topic': 'cmd_vel',
            'drive_index': 1,
            'steering_index': 3,
            'max_angular_velocity': 0.785,
            'max_velocity': 1.0
        
        }
        for key, value in parameters.items():
            if not rospy.has_param("/"+key):
                rospy.set_param("/"+key, value)
    
    def joy_callback(self, msg):
        """
        Callback function for joy subscriber.
        """
        self.twist_cmd.linear.x = self.max_velocity * msg.axes[self.drive_index]
        self.twist_cmd.angular.z = self.max_angular_velocity * msg.axes[self.steering_index]
        self.pub.publish(self.twist_cmd)

if __name__ == '__main__':
    rospy.init_node('joy_to_twist_cmd')
    joy_to_twist_cmd = JoyToTwistCmd()
    rospy.spin()