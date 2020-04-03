#!/usr/bin/env python

import random

import rospy
from pid import PID
from sensor_msgs.msg import Range
from thymio_controller import ThymioController


class Task4(ThymioController):

    # Max range of the Thymio's proximity sensors
    OUT_OF_RANGE = 0.12

    # Target distance of the robot from the wall
    TARGET_DISTANCE = OUT_OF_RANGE - 0.01

    # Target difference between the distance measured by the two distance sensors
    TARGET_ERROR = 0.001

    def __init__(self):
        super(Task4, self).__init__()

        # Subscribe to the updates of the proximity sensors.
        self.front_sensors = ["center_left", "center", "center_right"]
        self.lateral_sensors = ["left", "right"]
        self.rear_sensors = ["rear_left", "rear_right"]
        self.proximity_sensors = self.front_sensors + self.lateral_sensors + self.rear_sensors
        self.proximity_distances = dict()
        self.proximity_subscribers = [
            rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor)
            for sensor in self.proximity_sensors
        ]

        # PID controller to control the angular velocity, with the objective of minimizing the difference in distance
        # measured by the two center-left/-right proximity sensors.
        self.rotation_controller = PID(5, 0, 1)

    def update_proximity(self, data, sensor):
        self.proximity_distances[sensor] = data.range

    def run(self):
        """Controls the Thymio."""

        # Sleep until the first update is received for the clock and each proximity sensor
        while not rospy.is_shutdown():
            self.sleep()

            if len(self.proximity_distances) == len(self.proximity_sensors):
                break

        while not rospy.is_shutdown():
            # Start moving straight
            while not rospy.is_shutdown():
                # Check if the robot reached an obstacle it cannot pass through.
                if any(self.proximity_distances[sensor] < self.TARGET_DISTANCE for sensor in self.front_sensors):
                    break

                # Just move with constant velocity
                self.vel_msg.linear.x = 0.3
                self.vel_msg.angular.z = 0

                self.velocity_publisher.publish(self.vel_msg)

                self.sleep()

            # Stop the robot
            self.stop()

            # Rotate in a random direction until the robot is clear from obstacles
            direction = random.sample([-1, 1], 1)[0]

            while not rospy.is_shutdown():
                if all(self.proximity_distances[sensor] >= self.OUT_OF_RANGE - 0.001 for sensor in self.front_sensors):
                    break

                # Just move with constant velocity
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = direction * 3.0

                self.velocity_publisher.publish(self.vel_msg)

                self.sleep()

            # Stop the robot
            self.stop()


if __name__ == '__main__':
    controller = Task4()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
