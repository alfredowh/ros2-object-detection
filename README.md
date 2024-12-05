# [Work In Progress] Object Detection using ROS2 and Gazebo

Repository for object detection project using ROS2 Humble and Gazebo Fortress.

## Development

``` bash
colcon build                           # Build the project
source install/local_setup.bash        # Sourcing the packages
```

## Folder Structure

The folder structure of the input data should look like this:

```bash
./
├── application/          
│   ├── data/                        # Generated images 
│   └── src/
│       ├── data_generation          # Package for image generation from gazebo
│       └── object_detection         # Package for YOLOv7 object detector
├── turtlebot_gazebo/                # Package for gazebo simulation
│   ├── include/ 
│   ├── launch/                      # Launch files for starting simulation
│   ├── models/                      # Description files for all necessary models
│   ├── params/                      # Parameter files for gazebo bridge etc.
│   ├── rviz/                        # Config files for rviz2 visualization
│   ├── src/                         # Source files
│   ├── urdf/                        # Description files for mobile robots
│   └── worlds/                      # World description
...

## Acknowledgements

The Gazebo simulation implementations in this project utilizes significant portions of the implementation from the [Turtlebot3 simulation repository](https://github.com/azeey/turtlebot3_simulations/tree/new_gazebo) by [Addisu Z. Taddese](https://github.com/azeey) and contributors. 
The repository includes mAP evaluation, scripts for label format conversion etc. which makes our cross model evaluations more efficient. For further details, please visit the [official repository](https://github.com/azeey/turtlebot3_simulations/tree/new_gazebo).
