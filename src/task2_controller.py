#!/usr/bin/env python

import rospy
from math import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion

from pid import PID


class ThymioController:

    # Max range of the Thymio's proximity sensors
    OUT_OF_RANGE = 0.12

    # Target distance of the robot from the wall
    TARGET_DISTANCE = OUT_OF_RANGE - 0.01

    # Target difference between the distance measured by the two distance sensors
    TARGET_ERROR = 0.001

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

        # Subscribe to the updates of the proximity sensors.
        self.proximity_sensors = ["left", "center_left", "center", "center_right", "right"]
        self.proximity_subscribers = [
            rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor)
            for sensor in self.proximity_sensors
        ]
        self.proximity_distances = dict()

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

        # PID controller to control the angular velocity, with the objective of minimizing the difference in distance
        # measured by the two center-left/-right proximity sensors.
        self.rotation_controller = PID(5, 0, 1)

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

    def update_proximity(self, data, sensor):
        self.proximity_distances[sensor] = data.range

    def euclidean_distance(self, new_pose, estimated_pose):
        """
        :param new_pose:
        :param estimated_pose:
        :return: Euclidean distance between current pose and the goal pose
        """
        return sqrt(pow((new_pose.x - estimated_pose.x), 2) +
                    pow((new_pose.y - estimated_pose.y), 2))

    def angular_difference(self, estimated_pose, new_pose):
        """

        :param estimated_pose:
        :param new_pose:
        :return: Angle difference
        """
        return atan2(sin(new_pose.theta - estimated_pose.theta), cos(new_pose.theta - estimated_pose.theta))

    def run(self):
        """Controls the Thymio."""

        # Sleep until the first update is received for the clock and each proximity sensor
        while not rospy.is_shutdown():
            self.sleep()

            if len(self.proximity_distances) == len(self.proximity_sensors):
                break

        # Start moving straight
        while not rospy.is_shutdown():
            # Check if the robot reached the wall. Waiting until two sensors have the wall in range ensures that one of
            # them is the center_left or center_right one, allowing us to detect in which direction we should turn.
            distances = self.proximity_distances.values()

            if sum(distance < self.TARGET_DISTANCE for distance in distances) >= 2:
                break

            # Just move with constant velocity
            self.vel_msg.linear.x = 0.3
            self.vel_msg.angular.z = 0

            self.velocity_publisher.publish(self.vel_msg)

            self.sleep()

        # Stop the robot
        self.stop()

        # Align with respect to the wall
        count = 0

        while not rospy.is_shutdown():
            # Use the difference between the distances measured by the two proximity sensors to detect whether the robot
            # is facing the wall
            target_diff = 0
            diff = self.proximity_distances["center_left"] - self.proximity_distances["center_right"]

            error = target_diff - diff

            # Ensure that the error stays below target for a few cycles to smooth out the noise a bit
            if abs(error) <= self.TARGET_ERROR:
                count += 1
            else:
                count = 0

            if count == 3:
                break

            # Use the PID controller to minimize the distance difference
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = self.rotation_controller.step(error, self.step.to_sec())

            self.velocity_publisher.publish(self.vel_msg)

            self.sleep()

        # Final pose reached
        self.stop()

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
