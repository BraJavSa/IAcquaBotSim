#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class CmdVelRepublisher:
    def __init__(self):
        rospy.init_node('cmd_vel_republisher')
        self.last_cmd_vel = Twist()
        self.last_cmd_vel.linear.x = 0.0
        self.last_cmd_vel.linear.y = 0.0
        self.last_cmd_vel.linear.z = 0.0
        self.last_cmd_vel.angular.x = 0.0
        self.last_cmd_vel.angular.y = 0.0
        self.last_cmd_vel.angular.z = 0.0
        rospy.Subscriber("iacquabot/cmd_vel", Twist, self.cmd_vel_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(50)

    def cmd_vel_callback(self, msg):
        
        self.last_cmd_vel = msg

    def run(self):
        
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.last_cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        republisher = CmdVelRepublisher()
        republisher.run()
    except rospy.ROSInterruptException:
        pass
