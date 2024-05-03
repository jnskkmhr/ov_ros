#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class JoyToAckermannCmd:
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
        self.ackermann_topic_name = self.get_param('ackermann_topic')
        self.drive_index = self.get_param('drive_index')
        self.steering_index = self.get_param('steering_index')
        self.max_steering_angle = self.get_param('max_steering_angle')
        self.max_velocity = self.get_param('max_velocity')

        self.sub = rospy.Subscriber(self.joy_topic_name, Joy, self.joy_callback)
        self.pub = rospy.Publisher(self.ackermann_topic_name, AckermannDriveStamped, queue_size=1)
        self.ackermann_cmd = AckermannDriveStamped()
    
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
            'ackermann_topic': 'ackermann_cmd',
            'drive_index': 1,
            'steering_index': 3,
            'max_steering_angle': 0.785,
            'max_velocity': 1.0
        }
        for key, value in parameters.items():
            if not rospy.has_param("/"+key):
                rospy.set_param("/"+key, value)
    
    def joy_callback(self, msg):
        """
        Callback function for joy subscriber.
        """
        self.ackermann_cmd.drive.speed = self.max_velocity * msg.axes[self.drive_index]
        self.ackermann_cmd.drive.steering_angle = self.max_steering_angle * msg.axes[self.steering_index]
        self.ackermann_cmd.header.stamp = rospy.Time.now()
        self.pub.publish(self.ackermann_cmd)

if __name__ == '__main__':
    rospy.init_node('joy_to_ackermann_cmd')
    joy_to_ackermann_cmd = JoyToAckermannCmd()
    rospy.spin()