#!/usr/bin/env python
from copy import copy
from math import sin, cos

import rospy
from pid import PID
from sensor_msgs.msg import Range
from thymio_controller import ThymioController


class Task3(ThymioController):

    # Max range of the Thymio's proximity sensors
    OUT_OF_RANGE = 0.12

    # Target distance of the robot from the wall
    TARGET_DISTANCE = OUT_OF_RANGE - 0.02

    # Target difference between the distance measured by the two distance sensors
    TARGET_ANGLE_ERROR = 0.001
    TARGET_DISTANCE_ERROR = 0.01

    def __init__(self):
        super(Task3, self).__init__()

        # Subscribe to the updates of the proximity sensors.
        self.proximity_sensors = ["left", "center_left", "center", "center_right", "right", "rear_left", "rear_right"]
        self.proximity_subscribers = [
            rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor)
            for sensor in self.proximity_sensors
        ]
        self.proximity_distances = dict()

        self.rotation_controller = PID(5, 0, 1)
        self.move_straight_controller = PID(3, 0, 0.3, max_out=0.3)

    def update_proximity(self, data, sensor):
        self.proximity_distances[sensor] = data.range

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

        # Move that robot in such a way its reference frame is as close as possible to a point that is 2 meters
        # from the wall, in this case relying on odometry, and then stop it
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


if __name__ == '__main__':
    controller = Task3()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
