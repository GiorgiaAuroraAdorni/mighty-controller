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

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # set node update frequency in Hz
        frequency = 20.0
        self.rate = rospy.Rate(frequency)
        self.step = rospy.Duration.from_sec(1.0 / frequency)  # 1/60 sec

        self.radius = 1
        self.period = 30

    def human_readable_pose2d(self, pose):
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
