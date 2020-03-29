#!/usr/bin/env python

import rospy
from math import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class ThymioController:

    def __init__(self):
        # Creates a node with name 'thymio_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('thymio_controller', anonymous=True)

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # Velocity publisher which will publish to the topic '/~robot_name/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % self.name, Twist, queue_size=10)

        # Pose subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/%s/odom' % self.name, Odometry, self.log_odometry)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.vel_msg = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(60)

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

    @staticmethod
    def human_readable_pose2d(pose):
        """Converts pose message to a human readable pose tuple.
        :param pose:
        :return:
        """

        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.vel_msg = data.twist.twist

        printable_pose = self.human_readable_pose2d(self.pose)

        # log robot's pose
        rospy.loginfo_throttle(
            period=5,  # log every 10 seconds
            msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        )

    @staticmethod
    def get_control():
        """

        :return:
        """
        return Twist(
            linear=Vector3(
                .2,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                .0
            )
        )

    def euclidean_distance(self, new_pose, estimated_pose):
        """
        :param goal_pose
        :return: Euclidean distance between current pose and the goal pose
        """
        return sqrt(pow((new_pose.x - estimated_pose.x), 2) +
                    pow((new_pose.y - estimated_pose.y), 2))

    def linear_vel(self, new_pose, estimated_pose, constant=4):
        """
        :param goal_pose
        :param constant
        :return: clipped linear velocity
        """
        velocity = constant * self.euclidean_distance(new_pose, estimated_pose)
        return min(max(-5, velocity), 5)

    def angle_difference(self, new_pose, estimated_pose):
        """
        :param goal_pose:
        :return: the difference between the current angle and the goal angle
        """
        return atan2(sin(new_pose.theta - estimated_pose.theta), cos(new_pose.theta - estimated_pose.theta))

    def angular_vel(self, new_pose, estimated_pose, constant=12):
        """
        :param goal_pose:
        :param constant:
        :return: the angular velocity computed using the angle difference
        """
        return constant * self.angle_difference(new_pose, estimated_pose)

    # FIXME
    def run(self):
        """Controls the Thymio."""

        while not rospy.is_shutdown():
            # decide control action
            velocity = self.get_control()

            # publish velocity message
            self.velocity_publisher.publish(velocity)

            self.sleep()

    def sleep(self):
        """Sleep until next step and if rospy is shutdown launch an exception"""
        self.rate.sleep()
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException

    def stop(self):
        """Stops our robot"""
        self.velocity_publisher.publish(Twist())
        self.sleep()


if __name__ == '__main__':
    controller = ThymioController()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
