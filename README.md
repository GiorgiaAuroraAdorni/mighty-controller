# mighty-controller
> Second assignment for Robotics course @ USI 19/20. An open loop controller that moves a Thymio in Gazebo.

#### Contributors

**Giorgia Adorni** - giorgia.adorni@usi.ch  [GiorgiaAuroraAdorni](https://github.com/GiorgiaAuroraAdorni)

**Elia Cereda** - elia.cereda@usi.ch  [EliaCereda](https://github.com/EliaCereda)

#### Prerequisites

- Python 2
- ROS
- Gazebo

#### Installation

Clone the repository

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/GiorgiaAuroraAdorni/mighty-controller
```

Build the ROS packages and update the environment

```sh
$ catkin build
$ re.
```

#### Usage

##### Tasks

1. The file `task1_controller.launch` is a launch file that contains the instruction to configure the node
   `task1_controller.py`, that implements an open loop controller to move the MyT along an "8" trajectory.  
   
   The execution of **Gazebo** with one simulated Thymio can be started by running:
    ```sh
    $ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=empty 
    ```

   In order to see the Thymio moving in Gazebo run the command:

   ```sh
   $ roslaunch mighty-controller task1_controller.launch
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`. A demonstration of the robot behavior can be found at the following link: [task 1](https://youtu.be/EYP36fpk4nQ).
   
   Being an open-loop controller, we can see the robot correctly drawing an eight figure but gradually drifting away 
   from the origin. This is already visible after one iteration and tends to get worse with time.

2. The file `task2_controller.launch` is a launch file that contains the instruction to configure the node `task2_controller.py`, that moves the MyT straight ahead until it detects a wall without hitting it, using proximity sensors. Then turns the robot in place in such a way that the robot's x axis is orthogonal to the wall.  
   Open Gazebo using the world 'wall':
   ```sh
   $ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=wall 
   ```

   To execute the task, run the command:
   ```sh
   $ roslaunch mighty-controller task2_controller.launch
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`.

   We saw that the robot takes quite a bit of time to stop, so in the end we decided to stop the robot as soon as we detect an obstacle on the proximity sensors. This prevents the robot from bumping into the wall.

3. The file `compulsory.launch` is a launch file that contains the instruction to configure the node `task3_controller.py`, that move the MyT straight ahead until it is close to an obstacle without hitting it, using the proximity sensors. Then turns the robot in such a way that it is facing opposite to the wall, then move and stop in such a way that its reference frame is as close as possible to a point that is 2 meters from the wall, in this case relying on odometry.  
   In order to execute the final task, open Gazebo using the world 'wall':
   ```sh
   $ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=wall 
   ```

    Then, run the command:
   ```sh
   $ roslaunch mighty-controller compulsory.launch 
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`. A demonstration of the robot behavior can be found at the following link: [compulsory task](https://youtu.be/feHJWSdJr3k).

##### Bonus Tasks

1. The file `task4_controller.launch` is a launch file that contains the instructions to configure the node `task4_controller.py`, that makes the MyT randomly explores an environment, avoiding any obstacles. In particular, it moves the MyT straight ahead until it is close to an obstacle, turning it in place (in a random direction) until it clears the obstacle and continuing to move straight.  

   Deploy the robot in the arena world (it may take a few minutes for gazebo to load the world):
   ```sh
   $ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=arena 
   ```

   In order to see the Thymio moving in Gazebo run the command:

   ```sh
   $ roslaunch mighty-controller task4_controller.launch
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`.

   One problem that we saw executing this task is that sometimes, when turning, the robot gets so close to a wall that the
   proximity sensors seemingly pass through to the other side and don't detect the obstacle anymore (for instance, in 
   [this video](https://youtu.be/Rw8m6o7LnGo?t=125)).

2. The file `bonus.launch` is a launch file that contains the instruction to launch the same node of the previous task: it executes the `task4_controller.py`, but this time simulating two or more robots at once.
   
   Deploy the first robot in the arena world (it may take a few minutes for gazebo to load the world):
   ```sh
   $ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=arena 
   ```
   
   In order to spawn the second Thymio and start moving both in Gazebo, run the command:
   ```sh
   $ roslaunch mighty-controller bonus.launch
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`. A demonstration of the robot behavior can be found at the following link: [bonus task](https://youtu.be/ErVaC6nk9DU).

   The same weird behaviour we saw in task 4 is also present here.

   Adding a second robot on the other hand works well without any change to the code. In our experience, often one robot continues in its path without stopping, while the other stops and changes direction. When they both stop, if they randomly decide to turn in the same direction, they may meet again before being able to part ways but the probability this happens more than a few times decreases exponentially.

   The only drawback that we see in our controller is that it tends to follow the walls, so the middle of the room is explored less than the borders. On the other hand, this approach allows the robot to quickly move away from the obstacles and is robust even in case the robot gets stuck in a corner: it will just keep rotating in place until it's clear to go (in extreme cases, rotating 180º and going back from where it came). 
