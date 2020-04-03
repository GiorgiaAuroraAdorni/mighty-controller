#!/usr/bin/env python

import rospy
from math import sqrt, sin, cos, atan2
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class ThymioController(object):

    def __init__(self):
        # Creates a node with name 'thymio_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('thymio_controller', anonymous=True)

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # Velocity publisher which will publish to the topic '/~robot_name/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % self.name, Twist, queue_size=10)

        # Pose subscriber to the topic '/~robot_name/odom': call self.log_odometry.
        self.pose_subscriber = rospy.Subscriber('/%s/odom' % self.name, Odometry, self.log_odometry)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose2D()

        # initialize linear and angular velocities to 0
        self.vel_msg = Twist()

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # set node update frequency in Hz
        frequency = 20.0
        self.rate = rospy.Rate(frequency)
        self.step = rospy.Duration.from_sec(1.0 / frequency)  # 1/60 sec

    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple.
        :param pose:
        :return: 2d pose
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

        result = Pose2D(
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""
        self.vel_msg = data.twist.twist
        self.pose = self.human_readable_pose2d(data.pose.pose)

        # log robot's pose
        rospy.loginfo_throttle(
            period=5,  # log every 10 seconds
            msg=self.name + ' (%.3f, %.3f, %.3f) ' % (self.pose.x, self.pose.y, self.pose.theta)  # message
        )

    def euclidean_distance(self, new_pose, estimated_pose):
        """
        :param new_pose:
        :param estimated_pose:
        :return: Euclidean distance between current pose and the goal pose
        """
        return sqrt(pow((new_pose.x - estimated_pose.x), 2) +
                    pow((new_pose.y - estimated_pose.y), 2))

    def linear_vel(self, new_pose, estimated_pose):
        """
        :param new_pose
        :param estimated_pose
        :return: clipped linear velocity
        """
        distance = self.euclidean_distance(new_pose, estimated_pose)
        velocity = distance / self.step.to_sec()

        return velocity

    def angular_difference(self, estimated_pose, new_pose):
        """

        :param estimated_pose:
        :param new_pose:
        :return: Angle difference
        """
        return atan2(sin(new_pose.theta - estimated_pose.theta), cos(new_pose.theta - estimated_pose.theta))

    def angular_vel(self, new_pose, estimated_pose):
        """

        :param new_pose:
        :param estimated_pose:
        :return: the angular velocity computed using the angle difference
        """
        ang_difference = self.angular_difference(estimated_pose, new_pose)
        velocity = ang_difference / self.step.to_sec()

        return velocity

    def sleep(self):
        """Sleep until next step and if rospy is shutdown launch an exception"""
        self.rate.sleep()

        if rospy.is_shutdown():
            raise rospy.ROSInterruptException

    def stop(self):
        """Stops our robot"""
        self.velocity_publisher.publish(Twist())
        self.sleep()
