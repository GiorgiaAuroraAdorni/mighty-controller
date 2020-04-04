# mighty-controller
> Second assignment for Robotics course @ USI 19/20. An open loop controller that moves a Thymio in Gazebo.

#### Contributors

**Giorgia Adorni** - giorgia.adorni@usi.ch  [GiorgiaAuroraAdorni](https://github.com/GiorgiaAuroraAdorni)

**Elia Cereda** - elia.cereda@usi.ch  [EliaCereda](https://github.com/EliaCereda)

#### Prerequisites

- Python 2.0 
- ROS
- Gazebo

#### Installation

Clone the repository

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/GiorgiaAuroraAdorni/mighty-controller
```

build ROS packages and update the environment

```sh
$ catkin build
$ re.
```

#### Usage

The execution of **Gazebo** with the simulated Thymio can be started by running:

```sh
$ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=empty 
```

##### Tasks

1. The file `task1_controller.launch` is a launch file that contains the instruction to configure the node
   `task1_controller.py`, that implements an open loop controller to move the MyT along an "8" trajectory.  
   In order to see the Thymio moving in Gazebo run the command:

   ```sh
   $ roslaunch mighty-controller task1_controller.launch
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`.  
   A demonstration of the robot behavior can be found at the following link: [task 1](https://youtu.be/EYP36fpk4nQ).

2. The file `task2_controller.launch` is a launch file that contains the instruction to configure the node `task2_controler.py`, that moves the MyT straight ahead until it is close to an obstacle (wal) without hitting it (using proximity sensors). Then turns the robot in place in such a way that the robot's x axis should be orthogonal to the wall.  
   Run gazebo using the world 'wall':

   ```sh
   $ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=wall 
   ```

   To execute the task, run the command:

   ```sh
   $ roslaunch mighty-controller task2_controller.launch
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`.

3. The file `compulsory.launch` is a launch file that contains the instruction to configure the node `task3_controller.py`, that move the MyT straight ahead until it is close to an obstacle without hitting it, using the proximity sensors. Then turns the robot in such a way that it is facing opposite to the wall, then move and stop in such a way that its reference frame is as close as possible to a point that is 2 meters from the wall, in this case relying on odometry.  
   In order to execute the final task:

   ```sh
   $ roslaunch mighty-controller compulsory.launch 
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`.  
   A demonstration of the robot behavior can be found at the following link: [compulsory task](https://youtu.be/feHJWSdJr3k).

##### Bonus Tasks

1. The file `task4_controller.launch` is a launch file that contains the instruction to configure the node `task4_controller.py`, that makes the MyT randomly explores an environment, avoiding any obstacles. In particular, it moves the MyT straight ahead until it is close to an obstacle, turning it in place (in a random direction) until it clears the obstacle and continuing to move straight.  

   Deploy the robot in the arena world (it may take a few minutes for gazebo to load the world):

   ```sh
   $ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=arena 
   ```

   In order to see the Thymio moving in Gazebo run the command:

   ```sh
   $ roslaunch mighty-controller task4_controller.launch
   ```


   The execution of this task can be stopped by terminating the controller with `Ctrl + C`.

2. The file `bonus.launch` is a launch file that contains the instruction to launch the same node of the previous task: it executes the `task4._controller.py`, but this time simulating two or more robots at once.   
   In order to see the Thymio moving in Gazebo run the command:

   ```sh
   $ roslaunch mighty-controller bonus.launch
   ```

   The execution of this task can be stopped by terminating the controller with `Ctrl + C`.  
   A demonstration of the robot behavior can be found at the following link: [bonus task](https://youtu.be/ErVaC6nk9DU).

