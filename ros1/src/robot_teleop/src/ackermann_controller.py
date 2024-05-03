#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
import math

# steer_joint_name = ['right_front_steering_joint', 'left_front_steering_joint']
# drive_joint_name = ['right_front_wheel_joint', 'right_rear_wheel_joint', 'left_front_wheel_joint', 'left_rear_wheel_joint']
# radius = 0.09
# width = 0.453
# length = 0.673
# gamma = 1.0

class AckermannController:
    """
    Input topic: joy
    Output topic: steering_command, drive_command(JointState)
    In future, we will use ackermann_msgs/AckermannDriveStamped
    We have two nodes: joy to ackermann and ackermann to joint state
    """
    def __init__(self):
        """
        Initialize the node.
        Subscriber: 
            ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
        Publisher: 
            steering_command(sensor_msgs/JointState)
            drive_command(sensor_msgs/JointState)
        """
        self.declare_params()
        self.steer_joint_name = self.get_param('steering_joint_name')
        self.drive_joint_name = self.get_param('drive_joint_name')
        self.length = self.get_param('length')
        self.width = self.get_param('width')
        self.radius = self.get_param('radius')
        self.gamma = self.get_param('gamma')

        self.sub = rospy.Subscriber(self.get_param('ackermann_cmd'), AckermannDriveStamped, self.callback)
        self.steer_pub = rospy.Publisher(self.get_param('drive_joint_topic_name'), JointState, queue_size=1)
        self.drive_pub = rospy.Publisher(self.get_param('steering_joint_topic_name'), JointState, queue_size=1)

        self.steer_joint_state = JointState()
        self.drive_joint_state = JointState()
        self.steer_joint_state.name = self.steer_joint_name
        self.steer_joint_state.position = [0.0] * len(self.steer_joint_name)
        self.drive_joint_state.name = self.drive_joint_name
        self.drive_joint_state.velocity = [0.0] * len(self.drive_joint_name)

        self.joy_steer = 0.0
        self.joy_driving = 0.0
    
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
            'drive_joint_name': ['right_front_wheel_joint', 'right_rear_wheel_joint', 'left_front_wheel_joint', 'left_rear_wheel_joint'],
            'steering_joint_name': ['right_front_steering_joint', 'left_front_steering_joint'],
            'drive_joint_topic_name': 'drive_command',
            'steering_joint_topic_name': 'steering_command',
            'ackermann_cmd': 'ackermann_cmd',
            'length': 0.673,
            'width': 0.453,
            'radius': 0.09, 
            'gamma': 1.0,
        }
        for key, value in parameters.items():
            if not rospy.has_param("/"+key):
                rospy.set_param("/"+key, value)
        
    
    def callback(self, msg):
        """
        Callback function for ackermann_cmd subscriber.
        Args:
            msg (ackermann_msgs/AckermannDriveStamped): Ackermann drive command"""
        self.update_joint_pos(self.steer_to_steer_ack(msg.drive.steering_angle))
        self.update_joint_vel(self.vel_to_omega(msg.drive.speed))
        self.steer_pub.publish(self.steer_joint_state)
        self.drive_pub.publish(self.drive_joint_state)
    
    def update_joint_pos(self, steer_ack):
        """
        Update the joint position (steering angle) based on ackermann steering.
        Args:
            steer_ack (float): command_steering/ackermann_ratio"""
        steer_left = math.atan(self.width*math.tan(steer_ack)/(self.width + 0.5*self.length*math.tan(steer_ack)))
        steer_right = math.atan(self.width*math.tan(steer_ack)/(self.width - 0.5*self.length*math.tan(steer_ack)))
        self.steer_joint_state.position = [steer_left, steer_right]
        self.steer_joint_state.header.stamp = rospy.Time.now()

    def update_joint_vel(self, velocity):
        """
        Update the joint velocity.
        Args:
            value (float): joint velocity in rad/s"""
        self.drive_joint_state.velocity = [velocity] * len(self.drive_joint_name)
        self.drive_joint_state.header.stamp = rospy.Time.now()

    def vel_to_omega(self, vel):
        """
        Convert linear velocity to angular velocity.
        Args:
            vel (float): linear velocity in m/s
        Returns:
            float: angular velocity in rad/s"""
        return vel / self.radius
    
    def steer_to_steer_ack(self, steer):
        """
        Convert steering angle to ackermann steering.
        Args:
            steer (float): steering angle in rad
        Returns:
            float: ackermann steering"""
        return steer/self.gamma

if __name__ == '__main__':
    rospy.init_node('ackermann_controller')
    node = AckermannController()
    rospy.spin()