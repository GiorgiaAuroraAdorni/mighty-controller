#!/usr/bin/env python
from copy import copy
from math import *

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from pid import PID
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion


class ThymioController:

    # Max range of the Thymio's proximity sensors
    OUT_OF_RANGE = 0.12

    # Target distance of the robot from the wall
    TARGET_DISTANCE = OUT_OF_RANGE - 0.02

    # Target difference between the distance measured by the two distance sensors
    TARGET_ANGLE_ERROR = 0.001
    TARGET_DISTANCE_ERROR = 0.01

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

        self.proximity_sensors = ["left", "center_left", "center", "center_right", "right", "rear_left", "rear_right"]
        self.proximity_subscribers = [
            rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor)
            for sensor in self.proximity_sensors
        ]
        self.proximity_distances = dict()

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

        self.rotation_controller = PID(5, 0, 1)
        self.move_straight_controller = PID(3, 0, 0.3, max_out=0.3)

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

    # FIXME
    def get_control(self):
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
            front_sensors = ["left", "center_left", "center", "center_right", "right"]

            if sum(self.proximity_distances[sensor] < self.TARGET_DISTANCE for sensor in front_sensors) >= 2:
                break

            # Just move with constant velocity
            self.vel_msg.linear.x = 0.3
            self.vel_msg.angular.z = 0

            self.velocity_publisher.publish(self.vel_msg)

            self.sleep()

        # Stop the robot
        self.stop()

        # Use the difference between the distances measured by the two proximity sensors to detect whether the robot
        # is facing the wall and decide in which direction turn the robot
        diff = self.proximity_distances["center_left"] - self.proximity_distances["center_right"]

        constant = 1
        if diff < 0:
            constant = -1

        # Start rotating
        while not rospy.is_shutdown():
            # Check if the back sensor are in range, in this case start the alignment
            back_sensors = ["rear_left", "rear_right"]
            if sum(self.proximity_distances[sensor] < self.TARGET_DISTANCE for sensor in back_sensors) > 0:
                break

            # Rotate with constant velocity
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = constant * 0.3

            self.velocity_publisher.publish(self.vel_msg)

            self.sleep()

        count = 0
        while not rospy.is_shutdown():
            # Use the difference between the distances measured by the two proximity sensors to detect whether the robot
            # is facing the wall
            target_diff = 0
            diff = self.proximity_distances["rear_left"] - self.proximity_distances["rear_right"]

            error = diff - target_diff

            # Ensure that the error stays below target for a few cycles to smooth out the noise a bit
            if abs(error) <= self.TARGET_ANGLE_ERROR:
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

        # TODO
        #  - [ ] Poi ti allontani a 2 metri ma il sensore non prende percio
        #  - [ ] usi odometry e calcoli la coordinata a chi vuoi arrivare
        #  - [ ] PID che minimizza l errore rispetto a quella coordinata

        wall_distance = self.proximity_distances["rear_left"]
        target_distance = 2.0 - wall_distance

        target_pose = copy(self.pose)
        target_pose.x += target_distance * cos(self.pose.theta)
        target_pose.y += target_distance * sin(self.pose.theta)

        while not rospy.is_shutdown():
            error = self.euclidean_distance(target_pose, self.pose)

            if error <= self.TARGET_DISTANCE_ERROR:
                break

            self.vel_msg.linear.x = self.move_straight_controller.step(error, self.step.to_sec())
            self.vel_msg.angular.z = 0.0

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
