# Robotics Project 1
## _OMNIDIRECTIONAL ROBOT_  

### - TEAM MEMBERS:
 - Martin Molinaro 
 - 
 - 
 
### - FOLDER CONTENT DESCRIPTION: 
- this **instructions.md** file
- the **frames.pdf** file [tf tree structure]
- the **howToRun_calibration.txt** file [brief how to run instructions + history of run calibration tests]
- ROS pkg **ros_project1.pkg**:
    - ***cfg*** folder with the configuration file **integration.cfg** (for dynamic reconfigure)
    - ***launch*** folder with the bags (and their cvs file) and the launch file **launcher.launch**
    - ***msg*** folder with custom msg file **WheelSpeed.msg**
    - ***rviz*** folder with rviz file **ros1.rviz** (to display the given position wrt odometry)
    - ***src*** folder with node file **ComputeOdometry.cpp** 
    - ***srv*** folder with service files: 
             - **reset_pos.srv** (to reset the odometry to any given pose)
             - **reset_ticks.srv** (to reset the initial values of the ticks depending on the bag)
             - **calibration.srv** (to easily change the robot parameters during the calibration phase)
    - **CMakeLists.txt** file
    - **package.xml** file
### - ROS PARAMETERS:
- **InitialX** : initial x position (set from launch file)
- **InitialY** : initial y position (set from launch file)
- **InitialTheta** : initial theta position (set from launch file)
  ##### - *Robot parameters*:
     - r (wheel radius)
     - l (longitudinal semi-axis)
     - w (axial semi-axis)
     - T (gear ratio)
     - N (encoder CPR)
### - TF tree STRUCTURE: 
[see **frames.pdf**]

### - CUSTOM msg STRUCTURE:
  **WheelSpeed.msg**:

    Header header
    float64 rpm_fl
    float64 rpm_fr
    float64 rpm_rr
    float64 rpm_rl
### - HOW TO RUN:
*Terminal 1:*
  
    cd robotics 
    catkin_make && roslaunch ros1 launcher.launch
*Terminal 2*:

      cd robotics/src/ros_project1/launch
      rosbag play --clock bag1.bag && rosservice call /reset_ticks 17313 11359 15421 13209 && rosservice call /reset_pos 0.0 0.0 0.0
   
**"robotics"=ws name*

###### *To run the other bags* 
*Use the following commands*:

    rosbag play --clock bag2.bag && rosservice call /reset_ticks 17269 11412 15462 13165 && rosservice call /reset_pos 0.0 0.0 0.0
    *or*
    rosbag play --clock bag3.bag && rosservice call /reset_ticks 23501 17600 21727 19381 && rosservice call /reset_pos 0.0 0.0 0.0
*! If you encounter any running problem check the Running fixes in the ADDITIONAL INFO below*
##### CALIBRATION
Using the **calibration.srv** we set the robot parameters to obtain a better tracking performance of the odometry wrt the GT pose

We suggest to set the integration method to Runge-Kutta (1) for better tracking.
You can run the calibrated project with these commands (based on the bag):
*Terminal 2*:

      cd robotics/src/ros_project1/launch
      rosbag play --clock bag1.bag && rosservice call /reset_ticks 17313 11359 15421 13209 && rosservice call /reset_pos 0.0 0.0 0.0 && rosservice call /calibration 0.077 0.18 0.17 5 43
      *or*
      rosbag play --clock bag2.bag && rosservice call /reset_ticks 17269 11412 15462 13165 && rosservice call /reset_pos 0.0 0.0 0.0 && rosservice call /calibration 0.077 0.18 0.17 5 43
      *or*
      rosbag play --clock bag3.bag && rosservice call /reset_ticks 23501 17600 21727 19381 && rosservice call /reset_pos 0.0 0.0 0.0 && rosservice call /calibration 0.077 0.18 0.17 5 43
### - ADDITIONAL INFO
##### ! Running fixes:
- *When you want to change the bag to play, some rviz simulation's issues can happen, in that case try with the following procedure (at least a couple of times):*
      - "kill" every running bag
      - "reset" the rviz (bottom left)
      - copy and paste the right command for the bag and run it
      - **!** If you see an initial offset in rviz redo the previous steps (usually run smoothly after the second time)
       (*Provided video of a run of calibrated bags:* https://polimi-it.zoom.us/rec/play/11Ke6NR7-RaQuf1hA8ZnaKpdOm-ttyocnUCpH00xcEBF9FOOxggZPU9fE-JgZc8YMk0uMKkZANv987xg.49wl-JRDVpbTK6mH?startTime=1651247326000 )
##### Launcher file features:
- it launches the node file **ComputeOdometry.cpp** 
- it launches the **dynamic reconfigurator** to set the integration method
- it sets the **initial ros parameters** (for the initial pose we chose the initial values from bag1, but they're similar to the ones of the other bags)
- it launches a custom **rviz** to display the computed /odom (red arrow) vs the given /pose (green arrow)
- it launches a custom **rqt_plot** to compare the computed velocities in /wheels_rpm vs the given ones in /wheel_states/velocity [compared in rad/min]
##### Control velocities comparison comment:
From the rqt_plot we can see that the topic of the computed velocities (/wheels_rpm) follows the topic of given velocities (/wheel_states/velocity) apart from some noise
##### Reset services comment:
In the commands provided to play the bags we used:
- **/reset_pos** that can set the initial pose to a desired value (in our case to zero)
- **/reset_ticks** that set the initial values of the ticks depending on the chosen bag
- **/calibration** used to easily perform the calibration tests without recompile the node code
