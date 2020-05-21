# Go-chase-it

A ROS  project using two ROS packages to control a 3 wheeled robot mounted with a lidar and camera to detect a white ball and drive towards it. This project is in partial fulfillment of my Udacity Robotics software Engineer Nanodegree

## Github Repository Link
[https://github.com/cynepton/go-chase-it](https://github.com/cynepton/go-chase-it)

## Project Description
### Summary of Tasks
In this project, I created two ROS packages inside my `catkin_ws/src`: the `drive_bot` and the `ball_chaser`. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:

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

    .Project2                          # Go Chase It Project
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
    │   │   ├── <yourworld>.world
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