# Go-chase-it

A ROS  project using two ROS packages to control a 3 wheeled robot mounted with a lidar and camera to detect a white ball and drive towards it. This project is in partial fulfillment of my Udacity Robotics software Engineer Nanodegree

## Github Repository Link
[https://github.com/cynepton/go-chase-it](https://github.com/cynepton/go-chase-it)

## Project Description
### Summary of Tasks
In this project, I created two ROS packages inside my `catkin_ws/src`: the `drive_bot` and the `ball_chaser`.[Intro](https://youtu.be/pzZKvUSFkgs) & [Preview](https://youtu.be/HxYGmwMp2uw). Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:

1. **`drive_bot`**:
  - I created a `my_robot` ROS package to hold the robot, the white ball, and the world.
  - Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. Or completely redesign the robot model! After all you want to impress your future employers :-D
  - House your robot inside the world you built in the Build My World projecta gazebo world.
  - Add a white-colored ball to your Gazebo world and save a new copy of this world.
  - The world.launch file should launch your world with the white-colored ball and your robot.

2. **`ball_chaser`**:
  - Create a `ball_chaser` ROS package to hold your C++ nodes.
  - Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
  - Write a `process_image` C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.

### Project Directory

    .go-chase-it                         # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── wood.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──        

## Setting up `my_robot`
The first task in this project is to create the `my_robot` ROS package. Inside `my_robot`, you will store and launch an empty Gazebo world file. As you proceed with the project, you will model and store a robot, as well as replace the empty world file with a custom gazebo world file. For now, follow these steps to set up `my_robot`.

*Note: Do not have more than one my_robot instance in the Gazebo world otherwise it would not be able to launch.*

### Create the `my_robot` Package
1. Create and initialize a `catkin_ws`. *The `catkin_ws`name is arbitrary.*

  ```
  mkdir -p /home/workspace/catkin_ws/src
  cd /home/workspace/catkin_ws/src
  catkin_init_workspace
  ```
2.  Navigate to the `src` directory of your `catkin_ws` and create the `my_robot` package:

  ```
  cd ~/catkin/path/catkin_ws/src/
  catkin_create_pkg my_robot
  ```
3. Next, create a `worlds` directory and a `launch` directory, that will further define the structure of your package:
  ```
  cd /home/workspace/catkin_ws/src/my_robot/
  mkdir launch
  mkdir worlds
  ```

### Create and Store an Empty Gazebo World File
Inside the `worlds` directory, create and store an empty Gazebo world file. As a reminder, in Gazebo a world is a collection of models, such as your robot, together with a specific environment. You can also define several other physical properties specific to this world.

1. Create an empty Gazebo world
  An empty world in Gazebo is a simple world, with no objects or models.

  ```
  cd /home/workspace/catkin_ws/src/my_robot/worlds/
  touch empty.world
  ```

2. Add the following to `empty.world`
  ```
    <?xml version="1.0" ?>

    <sdf version="1.4">

    <world name="default">

        <include>
        <uri>model://ground_plane</uri>
        </include>

        <!-- Light source -->
        <include>
        <uri>model://sun</uri>
        </include>

        <!-- World camera -->
        <gui fullscreen='0'>
        <camera name='world_camera'>
            <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
            <view_controller>orbit</view_controller>
        </camera>
        </gui>

    </world>
    </sdf>
  ```  

    The `.world` file uses the XML file format to describe all the elements with respect to the Gazebo environment. The simple world that you are creating here has the following elements:

    <sdf>: The base element which encapsulates the entire file structure and content.
    <world>: The world element defines the world description and several properties pertaining to that world. In this example, you are adding a ground plane, a light source, and a camera to your world. Each model or property can have further elements that add detail. For example, the camera has a pose element which defines its position and orientation.
    <include>: The include element, along with the <uri> element, provide a path to a particular model. In Gazebo there are several models that are available by default.

### Create a Launch File
Launch files in ROS allow us to execute more than one node simultaneously, which helps avoid the potentially tedious task of defining and launching several nodes in separate shells or terminals.

1. Create the world.launch file

  ```
  cd /home/workspace/catkin_ws/src/my_robot/launch/
  touch world.launch
  ```

2. Add the following to `world.launch`
    ```
    <?xml version="1.0" encoding="UTF-8"?>

    <launch>

    <!-- World File -->
    <arg name="world_file" default="$(find my_robot)/worlds/empty.world"/>

    <!-- Launch Gazebo World -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="use_sim_time" value="true"/>
        <arg name="debug" value="false"/>
        <arg name="gui" value="true" />
        <arg name="world_name" value="$(arg world_file)"/>
    </include>

    </launch>
    ```

As in the case of the `.world` file, the `.launch` files are also based on XML. The structure for the launch files has two parts -

- First, define arguments using the `<arg>` element. Each such element will have a `name` attribute and a `default` value.
- Second, include the `world.launch` file from the gazebo_ros package. The empty_world file includes a set of important definitions that are inherited by the world that we create. Using the world_name argument and the path to your .world file passed as the value to that argument, you will be able to launch your world in Gazebo.


<pre>
this is a <b>test</b>
</pre>
