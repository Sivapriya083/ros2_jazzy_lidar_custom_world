# `ros2_jazzy_lidar_world`

This project extends the **TortoiseBot Gazebo** package by adding a **custom Gazebo world** with multiple obstacles and a **2D LIDAR sensor plugin** to the robot’s URDF. The LIDAR publishes laser scan data on the `/scan` topic, which can be visualized in **RViz2** for obstacle mapping and debugging.  

---

##  Features
- Custom Gazebo world (`custom_obstacle_world.world`) with at least **10 unique obstacles** (boxes, cylinders, walls).  
- Integrated **2D LIDAR plugin** in TortoiseBot URDF.  
- Laser scans published on `/scan`.  
- RViz2 visualization showing LIDAR beams reflecting from obstacles.  
- Teleop integration for manual control.  

---

##  System Requirements
- **Ubuntu 24.04**  
- **ROS 2 Jazzy**  
- **Gazebo Harmonic**  
- RViz2  

---

##  Installation & Setup

1. Create a ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
````

2. Clone this repository:

   ```bash
   git clone https://github.com/your_username/ros2_jazzy_lidar_world.git
   ```

3. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

---
````
## Custom Gazebo World

The world file `custom_obstacle_world.world` includes:

* 4 boxes (randomly placed)
* 3 cylinders
* 2 short walls
* 1 tall wall

This ensures varied obstacle layouts for testing.

---

##  LIDAR Plugin in URDF

The TortoiseBot URDF includes a **Gazebo Ray Sensor Plugin** simulating a 2D LIDAR. Example snippet inside URDF:

```xml
<gazebo>
  <sensor type="ray" name="lidar">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_laser.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

##  Simulation

1. Launch the world with TortoiseBot:

   ```bash
   ros2 launch tortoisebot_gazebo custom_obstacle_world.launch.py
  ```
![]()

 
2. Open RViz2 to visualize `/scan`:

   ```bash
   rviz2
   ```
 ![]()
 
   * Add **LaserScan** display.
   * Set topic: `/scan`.
   * You should see LIDAR rays reflecting off obstacles.

3. Control the robot using teleop:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```


---

##  Expected Results

* Gazebo shows TortoiseBot in a world with at least 10 obstacles.
* `/scan` topic actively publishing laser data.
* RViz2 displays beams reflecting from obstacles.
* Robot can be manually moved with teleop to observe changing scans.

---

##  License

MIT License

```

---

```
