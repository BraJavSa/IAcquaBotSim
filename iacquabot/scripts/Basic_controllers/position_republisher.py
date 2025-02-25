#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

class PositionRepublisher:
    def __init__(self):
        rospy.init_node('position_republisher')
        self.last_desired_position = Pose()
        self.last_desired_position.position.x = 0.0
        self.last_desired_position.position.y = 0.0
        self.last_desired_position.position.z = 0.0
        self.last_desired_position.orientation.x = 0.0
        self.last_desired_position.orientation.y = 0.0
        self.last_desired_position.orientation.z = 0.0
        self.last_desired_position.orientation.w = 1.0
        rospy.Subscriber("iacquabot/desired_position", Pose, self.position_callback)
        self.position_pub = rospy.Publisher("/desired_position", Pose, queue_size=10)
        self.rate = rospy.Rate(50)

    def position_callback(self, msg):
        self.last_desired_position = msg

    def run(self):
        while not rospy.is_shutdown():
            self.position_pub.publish(self.last_desired_position)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        republisher = PositionRepublisher()
        republisher.run()
    except rospy.ROSInterruptException:
        pass
