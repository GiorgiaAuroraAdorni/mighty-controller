#!/usr/bin/env python

import rospy
from pid import PID
from sensor_msgs.msg import Range
from thymio_controller import ThymioController


class Task2(ThymioController):
    # Max range of the Thymio's proximity sensors
    OUT_OF_RANGE = 0.12

    # Target distance of the robot from the wall
    TARGET_DISTANCE = OUT_OF_RANGE - 0.01

    # Target difference between the distance measured by the two distance sensors
    TARGET_ERROR = 0.001

    def __init__(self):
        super(Task2, self).__init__()

        # Subscribe to the updates of the proximity sensors.
        self.proximity_sensors = ["left", "center_left", "center", "center_right", "right"]
        self.proximity_subscribers = [
            rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor)
            for sensor in self.proximity_sensors
        ]
        self.proximity_distances = dict()

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


if __name__ == '__main__':
    controller = Task2()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
