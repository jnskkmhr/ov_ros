#!/usr/bin/env python3
# coding: utf-8
from typing import List
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class SkidSteerController:
    """
    Take twist message as an input, 
    Return joint velocities for skid steer robot.
    """
    def __init__(self):
        self.declare_params()
        self.twist_sub = rospy.Subscriber(self.get_param("twist_topic_name"), Twist, self.drive_callback)
        self.joint_pub = rospy.Publisher(self.get_param("joint_topic_name"), JointState, queue_size=10)
        self.base = self.get_param("base")
        self.radius = self.get_param("radius")
        self.joint_name = self.get_param("joint_name")
        self.num_pairs = int(len(self.get_param("joint_name"))/2)
        assert len(self.joint_name) % 2 == 0, "Number of joints should be even."
        self.initialize_joint_state()
    
    def initialize_joint_state(self):
        """
        Create empty JointState object.
        """
        self.joint_state = JointState()
        self.joint_state.name = self.joint_name
        self.joint_state.velocity = [0.0] * len(self.joint_name)

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
            'joint_name': ['joint_1', 'joint_2'],
            'twist_topic_name': 'cmd_vel',
            'joint_topic_name': 'drive_command',
            'base': 0.453,
            'radius': 0.09
        
        }
        for key, value in parameters.items():
            if not rospy.has_param("/"+key):
                rospy.set_param("/"+key, value)
    
    def drive_callback(self, msg:Twist)->None:
        """
        msg (Twist): Twist message
        """
        scale = 0.3
        linear_velocity = msg.linear.x * scale
        angular_velocity = msg.angular.z * scale
        self.twist_to_joint_cmd(linear_velocity, angular_velocity)
        self.joint_pub.publish(self.joint_state)
    
    def twist_to_joint_cmd(self, linear_velocity:float, angular_velocity:float)->None:
        """
        Convert Twist to angular velocities
        Args:
            linear_velocity (float): linear velocity
            angular_velocity (float): angular velocity
        """
        omega_left = (linear_velocity - angular_velocity*self.base/2)/self.radius
        omega_right = (linear_velocity + angular_velocity*self.base/2)/self.radius
        self.joint_state.velocity = [omega_left, omega_right] * self.num_pairs
        self.joint_state.header.stamp = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('skid_steer_controller')
    r = rospy.Rate(20)
    node = SkidSteerController()
    rospy.spin()