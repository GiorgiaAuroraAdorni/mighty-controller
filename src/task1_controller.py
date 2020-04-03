#!/usr/bin/env python

from math import sin, cos, atan2, pi

import rospy
from geometry_msgs.msg import Pose2D
from thymio_controller import ThymioController


class Task1(ThymioController):

    def __init__(self):
        super(Task1, self).__init__()

        self.radius = 1
        self.period = 30

    def compute_pose(self, time_delta):
        """
        :param time_delta:
        :return:
        """

        progress = ((2 * pi) / self.period) * time_delta

        x = self.radius * sin(progress) * cos(progress)
        y = self.radius * sin(progress)

        dx = self.radius * (cos(progress) ** 2 - sin(progress) ** 2)
        dy = self.radius * cos(progress)
        theta = atan2(dy, dx)

        pose = Pose2D(x, y, theta)

        return pose

    def run(self):
        """Controls the Thymio."""

        # Sleep until the first time update is received
        self.sleep()

        start_time = rospy.Time.now()
        estimated_pose = None

        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now() - start_time

            next_time = (elapsed_time + self.step).to_sec()

            next_pose = self.compute_pose(next_time)

            if estimated_pose is not None:
                self.vel_msg.linear.x = self.linear_vel(next_pose, estimated_pose)
                self.vel_msg.angular.z = self.angular_vel(next_pose, estimated_pose)

                self.velocity_publisher.publish(self.vel_msg)

                self.sleep()

            # Check if the target velocity exceeds the theoretical max speed for the kinematics
            # if abs(self.vel_msg.linear.x) > 0.14:
            #     print("Going too fast, kinematics undefined!", elapsed_time.to_sec(), self.vel_msg.linear.x)

            estimated_pose = next_pose


if __name__ == '__main__':
    controller = Task1()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
