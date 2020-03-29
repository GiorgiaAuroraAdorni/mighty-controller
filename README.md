# mighty-controller
> Second assignment for Robotics course @ USI 19/20. An open loop controller that moves a Thymio in Gazebo.

#### Contributors

**Giorgia Adorni** - giorgia.adorni@usi.ch  [GiorgiaAuroraAdorni](https://github.com/GiorgiaAuroraAdorni)

**Elia Cereda** - elia.cereda@usi.ch  [EliaCereda](https://github.com/EliaCereda)

#### Brief

#### Prerequisites

- Python 2.0 
- ROS
- Gazebo

#### Installation and usage

```sh
$ git clone https://github.com/GiorgiaAuroraAdorni/mighty-controller
```

The execution of **Gazebo** with the simulated Thymio can be started by running:

```sh
$ roslaunch thymio_course_skeleton thymio_gazebo_bringup.launch name:=thymio10 world:=empty 
```

##### Tasks

1. The file `eight_trajectory_controller.launch` is a launch file that contains the instruction to configure the node `eight_trajectory_controller.py`, that  implements an open loop controller that moves the MyT along an "8" trajectory.  
   In order to see the Thymio moving in Gazebo run the command:

   ```sh
   $ roslaunch mighty_controller eight_trajectory_controller.launch robot_name:=thymio10
   ```

   The execution of this task can be  stopped by terminating the controller with `ctrl + C`.







