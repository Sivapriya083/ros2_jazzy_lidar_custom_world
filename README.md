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
- **RViz2**  

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

* boxes (randomly placed)
* cylinders
* Sphere
* tall walls


---

##  Simulation

1. Launch the world with TortoiseBot:

   ```bash
   ros2 launch tortoisebot_gazebo custom_obstacle_world.launch.py
  ```
![](https://github.com/Sivapriya083/ros2_jazzy_lidar_custom_world/blob/main/ostacle_launch.png?raw=true)

 
2. Open RViz2 to visualize `/scan`:

   ```bash
   rviz2
   ```
 ![](https://github.com/Sivapriya083/ros2_jazzy_lidar_custom_world/blob/main/rviz2.png?raw=true)
 
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
