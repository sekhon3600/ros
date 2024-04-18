#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        self.goal_distance = 0
        self.goal_angle = 0
        self.dist_goal_active = False
        self.angle_goal_active = False
        self.linear_speed = 1.0   # Adjust as needed
        self.angular_speed = 1.0  # Adjust as needed

        rospy.init_node('straights_and_turns_turtle_node', anonymous=True)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        rospy.loginfo("Initialized node!")
        rospy.spin()

    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        self.dist_goal_active = True if self.goal_distance != 0 else False

    def goal_angle_callback(self, msg):
        self.goal_angle = msg.data
        self.angle_goal_active = True if self.goal_angle != 0 else False

    def timer_callback(self, event):
        if self.dist_goal_active:
            if abs(self.goal_distance) > 0.1:
                # Move forward or backward
                linear_direction = 1 if self.goal_distance > 0 else -1
                self.publish_velocity(self.goal_distance, 0)
                self.dist_goal_active = False

            else:
                # Stop
                self.publish_velocity(0, 0)
                self.dist_goal_active = False

        if self.angle_goal_active:
            if abs(self.goal_angle) > 0.1:
                # Rotate left or right
                angular_direction = 1 if self.goal_angle > 0 else -1
                self.publish_velocity(0, self.goal_angle)
                self.angle_goal_active = False
                # If turning 270 degrees, start moving backward after turning 90 degrees
                if abs(self.goal_angle) > 180:
                    linear_direction = -1
                    self.publish_velocity(0, self.goal_angle)
                    self.angle_goal_active = False
            else:
                # Stop
                self.publish_velocity(0, 0)
                self.angle_goal_active = False

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velocity_publisher.publish(twist)

if __name__ == '__main__':
    try:
        turtlesim_straights_and_turns = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass
