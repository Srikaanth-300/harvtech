# **Description:**

The package has the necessary codes to spawn up a turtlebot in the simulation and cover a given field. Other packages and libraries like [Fields2Cover](https://fields2cover.github.io/index.html), [GracefulController](https://github.com/mikeferguson/graceful_controller), [Ublox_driver (for ROS)](https://github.com/KumarRobotics/ublox), turtlebot (for robot model) were used. Another controller package (Regulater_pure_pursuit_controller) has been tried out.

## **Steps to launch the system:**

Launch the gazebo simulation with the turtlebot3 (burger model) spawned up in an empty world.

`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

Then in a new terminal launch the navigation stack which launches the localization node as well as a RViz display window.

`roslaunch navigation complete_launch.launch`

This would load up the saved map and the plan config files. 20 seconds (buffer time) after launching the navigation stack (actual time and not ros time) the system would start to follow the plan generated.  

### **File tree:**

**Robot description:**

The robot's description files are in `turtlebot3 -> turtlebot3_description -> urdf`. Additional link and a joint was added in the _turtlebot3_burger.urdf.xacro_ urdf to simulate GPS sensor values. GPS plugin for gazebo was added in the file _turtlebot3_burger.gazebo.xacro_ with the latitude, longitude, update_rate, link_names specified.

**Localization:**

The localization of the system is done only using GPS and IMU. Robot_localization package was used to fuse data from GPS and IMU to get odometry message. _X,Y_ from GPS and _Yaw_ angle from IMU is fused together. Two nodes are run -navsat_transform_node and ekf node. To keep the system in accordance with _map->odom->base_footprint_ format, a dummy _odom->base_footprint_ is published. The launch file for localization is _`world_coordinates.launch`_ `(navigation/launch)`. And the parameter file to edit the parameters for the localization is _`gps.yaml`_ `(navigation/config)`.

**Mapping:**

Once we get odometry from GPS, the boundary points are saved as a CSV file to be used for planning. The script **_`save_xy_points.py`_** `(global_planner_f2c/scripts)` currently subscribes to _odometry/filtered_ and saves the points only when the node is killed at the current directory where the script is run. This CSV file can be used with Fields2Cover library for path generation.

The map must be saved as a pgm and a yaml file. So there are two ways to do it.
  
  1) Once the boundary points are saved as a CSV file publish it as a PoseStamped message in ROS, then subscribe to those poses, generate a map and save the map using map_server package. **_`path_publisher_to_map.py`_** `(global_planner_f2c/scripts)` publishes the points in a topic and **_`map_generator.py`_** `(global_planner_f2c/scripts)` creates an occupancy grid map with the boundaries marked as obstacles. Then save the map file using _map_server_ package with appropriate filename. (Crosscheck the map topic name).
 
  2) Create a map while getting the boundary points. This would save those boundary points as well but directly subscribes to those odometry messages and then generates the map using the same **_`map_generator.py`_** `(global_planner_f2c/scripts)` script. For this uncomment the commented lines of code in the file and change the topic accordingly. (NOTE: This is not preferred because it directly subscribes to odometry message, so ensure that the script starts only when the robot is at the  boundaries)

**Ublox_driver:**

GPS data from ublox devices are sent to the main PC as a topic. The nodes to run are in the _ublox_ package. To launch the nodes after confirming the fix status in the rover board (through leds), run  `roslaunch ublox_gps ublox_device.launch`. Ensure that appropriate params are set in the _`zed_f9p.yaml`_ file. With the existing params we get position,heading as a topic.

**Note:** Proper remapping of topics is required while using the actual GPS, IMU hardware for localization.

**Navigation:**

  **Global planner:**
  
  Fields2Cover library is used as the plan generator for a given boundary field. Install the library as mentioned in the official documentation. Run the script **_`plan_generator_without_map.py`_** `(global_planner_f2c/scripts)` specifying the location of the boundary points saved as a CSV file (while mapping), the filename for PNG and CSV file to be saved. This would give the plan to cover the field given the headland width, robot width, tool width (tutorials can be found on the official documentation) which is saved as a CSV file. The plan points are discretized for 10 cm. 

  This CSV file is read, a path is appended to the list of plan points using A* algorithm and then published as a global_plan. This is written as a global_planner plugin to interface with move_base. Ensure **_`f2c_planner.cpp`_** `(global_planner_f2c/src)` reads the correct plan CSV file. The plan is run at 0.5 Hz and is published only for 8m (80 points).

  **Controller:**

  Gracefull controller is used and implemented in this system. The package is utilised for waypoint following and the parameters related to it can be changed in _`config_edited.yaml`_ `(navigation/config)`. 

  Also to start navigation a shell script **_`bash_delay.sh`_** waits for 20 sec and then runs the python script **_`goal_initializer.py`_** to publish a dummy goal to move_base.

#### **Params:**

**Move_base:**
  - _planner_frequency_: This estimates the frequency at which the global planner runs. Since there is no need for dynamic replanning and the path is already generated, keeping it low given the operational speed is low would be better. Default used is 0.2 Hz.
  - _file_path_: Specify the path of the CSV file of the generated path points (from  plan_generator_without_map.py).

**Graceful_controller:**
  - _max_vel_x_: Maximum velocity of the robot to account while controlling.
  - _min_vel_x_: Minimum velocity of the robot.
  - _max_vel_theta_ and _min_vel_theta_: These parameters determine the maximum and minimum angular velocity of the robot.
  - _min_in_place_vel_theta_: The minimum angular velocity if the robot executes an in place rotation.
  - _acc_lim_x_ and _acc_lim_theta_: These limit the linear acceleration and angular acceleration of the robot by generating appropriate velocities.
  - _max_lookahead_: Sets the maximum distance till which the robot can consider goal points. Keeping it smaller results in proper following of the goal points since we want all the points to be covered.
  - _min_lookahead_: Minimum distance below which the control law cannot find the goal points.
  - _odom_topic_: The odom topic which is to be used. The velocity from the odom topic is utilised to generate proper control commands accounting for the robot's feasibility.
  - _prefer_final_rotation_: The orientation of the final goal is achieved. Setting this to false keeps the robot orientaation of the previous pose.
  
  **Note:** Refer to the official documentation for a detailed explanation of the parameters.

**Localization:**
  - _magnetic_declination_radians_: This param is set to zero but fill out the actual value while using hardware. Sets out the direction (in degrees) of true north from magnetic north.
  - _datum_: Specify the base station's latitude, longitude and yaw degree (in radians). The yaw angle can be found by testing and the value used for simulation is 1.57 rad.
  - _yaw_offset_: Offset in angle for the IMU. Since we haven't used an IMU sensor it is set to zero. If an IMU is used later, check if gives zero degrees when facing east else specify that angle in here.
  - _zero_altitude_: Setting this param to true makes the navsat_tranform_node to discard altitude value while localizing.
  - _broadcast_cartesian_transform_: Publishes the transform from 'utm' frame to 'map' frame.
  - _broadcast_cartesian_transform_as_parent_frame_: Publishes the 'utm' frame as parent frame.
  - _wait_for_datum_: Setting this parameter true makes the node take values from 'datum' parameter.
  - _map_frame_, _odom_frame_, _world_frame_, _base_link_frame_ are specified accordingly.
  - _use_odometry_yaw_: This parameter determines from which source to fuse the yaw angle. Since we get angle from IMU (or the Dual antenna GPS from ublox_ros also gives heading in IMU message type), the param is set to false.
  - _cartesian_frame_id_: Specify the name of the utm frame.
  - _frequency_: The frequency at which the node runs.

  - _publish_tf_: Publishes the transform from 'odom_frame' to 'map_frame'.
  - _two_d_mode_: Fuses only variable which are required for 2D operation. Disabling it is not necessary because our robot works in 2D.
  - _use_control_: This parameter is set to false because control variables (velocity) is not being used to estimate the state of the robot.
  - _odom_relative_, _odom_differential_, _imu_relative_ and _imu_differential_ are all set to false because we are using absolute values to estimate the state of the robot.

  - _baudrate_: Specify the baudrate which was configured through the u-center app for the ardusimple boards.
  - _frame_id_: Specify the frame id which will be used for publishing the GPS data.
  - _device_: The port at which the GPS is connected. 
  - _publish/nav/all_: Setting this parameter to true publishes all the topics related to navigation from the GPS like heading, velocity, acceleraation.

**Costmap:**
  - _footprint_: Ensure to give proper dimensions as the footprint of the robot.
  - _InflationLayer_ and _StaticLayer_ are used as plugins.

**Note:** Refer the official documentation for more details about these parameters.

##### **Known Issues:**
  - In the ublox_gps package while running the driver an error pops up 'TF buffer allocation is not proper'. It had no issues in the sensor data, but the log message kept on printing.
  - The driver is modified to publish the heading in IMU frame instead of GPS frame. While both the fix and heading messages are published in the same frame, issues occured in localization. So the frame name has been changed. If a separate IMU is used to get data, either disable the heading parameter from GPS or change the name of the frame in which the IMU message is published.
  - Also check the angle in datum parameter. It was found using trial and test.
  - Give proper width of the robot and tool to be used in the script (plan_generator_without_map.py), headland width and the minimum turning radius. Altering these would give out different plans.
  - Make sure the plan stays inside the boundary by changing the headland width.
  - Also Dubins curve is used for changing from one swath to another, but the continuous Dubins curve feature generates smooth plan but at times it goes out of the boundary and depends on the boundary area also. In simulation Dubins curve path worked well and continuous feature is not required.
  - There can be times where the plan goes out of boundary, pick out the best parameter to change from the above listed ones to get appropriate plans.
  - Ensure the plan doesn't go closer to the boundary while picking these parameters. If it is closer, then the costmap 'inflation_radius' and the 'inflation_scaling_factor' can also be changed as a last option. But this is not recommended because the robot would move closer to the boundary.
  - Since no odometry is used (dummy transform from odom_frame to base_link is published), there might be some topic and localization parameters remapping be required to avoid conflicts.

**Future scope:**
Obstacle detection and stopping can be incoporated. Saving the last pose from the plan and retriving it when launching again if the system fails is an additional feature. 
