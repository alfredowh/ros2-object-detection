# [Work In Progress] Object Detection using ROS2 and Gazebo

Repository for object detection project using ROS2 Humble and Gazebo Fortress.

## Packages
### Data Generation Package
This package is used to generate images from Gazebo simulation. It includes the function to drive the mobile robots following particular waypoints. With these waypoints on static environment (with same objects and same positions), identical driving scenarios on different environment can be achieved. Thereby, data annotation efforts can be reduced.

The images can be annotated in semi-automated way using visual foundation models or large-scale zero-shot object detector, e.g. [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO). This model is used by [Roboflow](https://roboflow.com/annotate) for automating data annotaion. Additionaly, [Roboflow](https://roboflow.com/annotate) provides annotation tools, which facilitates manual annotation and labels exports to different format.

![rosgraph_datageneration](https://github.com/user-attachments/assets/ff56ee0a-7244-45ad-9120-65892cb10c41)

### Object Detection Package [TODO]
This package utilizes YOLOv7 model to detect the objects in real-time while using the Gazebo simulation.

## Folder Structure

The folder structure of the input data should look like this:

``` bash
./
├── application/          
│   ├── data/                        # Generated images 
│   └── src/
│       ├── data_generation          # Package for image generation from gazebo
│       └── object_detection         # Package for YOLOv7 object detector
└── turtlebot_gazebo/                # Package for gazebo simulation
    ├── include/ 
    ├── launch/                      # Launch files for starting simulation
    ├── models/                      # Description files for all necessary models
    ├── params/                      # Parameter files for gazebo bridge etc.
    ├── rviz/                        # Config files for rviz2 visualization
    ├── src/                         
    ├── urdf/                        # Description files for mobile robots
    └── worlds/                      # World description
```

## Development

``` bash
Ubuntu 24.04.1 LTS

colcon build                           # Build the project
source install/local_setup.bash        # Sourcing the packages
```

## Data Generation
https://github.com/user-attachments/assets/eac1e242-f1b6-40f9-882d-a0d7f4a1298a

<br><br>

### Clear Weather
<div style="display: flex; justify-content: space-around; align-items: center; margin: auto; width: fit-content;">
  <img src="https://github.com/user-attachments/assets/2ec7ab73-e049-4c3d-977b-81d3f545386a" alt="gray_clear" width="300">
  <img src="https://github.com/user-attachments/assets/8b61ae24-a241-4a6d-b5a2-2dbb8b83d056" alt="asphalt_clear" width="300">
  <img src="https://github.com/user-attachments/assets/5cf5292e-d63e-41b4-8354-4597a52f5914" alt="gray_cloudy" width="300">
  <img src="https://github.com/user-attachments/assets/dbdf4483-5093-4fd1-b373-2dc8bc0cbf9b" alt="asphalt_cloudy" width="300">
  <img src="https://github.com/user-attachments/assets/3fc873f3-5cb2-4be0-acdb-036882f118df" alt="gray_night" width="300">
  <img src="https://github.com/user-attachments/assets/9a5e8970-ea13-405d-9b3b-e939c5317186" alt="gray_sunset" width="300">
</div>
<br><br>

### Rainy
<div style="display: flex; justify-content: space-around; align-items: center; margin: auto; width: fit-content;">
  <img src="https://github.com/user-attachments/assets/a4d3a210-f779-456a-8f4d-fe365279a871" alt="gray_clear_rain" width="300">
  <img src="https://github.com/user-attachments/assets/6de63620-3981-4f99-a5d3-6a698f0066c5" alt="asphalt_clear_rain" width="300">
  <img src="https://github.com/user-attachments/assets/58ac5759-724c-4360-bf6a-f4aa1960a46c" alt="gray_cloudy_rain" width="300">
  <img src="https://github.com/user-attachments/assets/723ed462-8b76-45d3-90c5-328012b6c0ac" alt="asphalt_cloudy_rain" width="300">
  <img src="https://github.com/user-attachments/assets/5b8aa0dc-3056-4588-aea1-50fe3b1c77c4" alt="gray_night_rain" width="300">
  <img src="https://github.com/user-attachments/assets/e777ecf7-250d-438f-98e9-d6d9c1ddac03" alt="gray_sunset_rain" width="300">
</div>
<br><br>

### Foggy
<div style="display: flex; justify-content: space-around; align-items: center; margin: auto; width: fit-content;">
  <img src="https://github.com/user-attachments/assets/49b08ffd-bb3c-4c11-8c1e-1a9a6829f39e" alt="gray_clear_fog" width="300">
  <img src="https://github.com/user-attachments/assets/e198e4e9-0e4d-4c28-bc8b-99550431476d" alt="asphalt_clear_fog" width="300">
  <img src="https://github.com/user-attachments/assets/36b74947-da65-438c-8593-af8268517d8b" alt="gray_cloudy_fog" width="300">
  <img src="https://github.com/user-attachments/assets/596f2e0a-99a5-40b4-b9ea-e927b358cbf3" alt="asphalt_cloudy_fog" width="300">
  <img src="https://github.com/user-attachments/assets/37438eeb-a17c-41d1-afe4-9a151cae8c1a" alt="gray_night_fog" width="300">
  <img src="https://github.com/user-attachments/assets/3d32691a-a96b-40d2-91b6-62fe32d6cece" alt="gray_sunset_fog" width="300">
</div>


## Acknowledgements

The Gazebo simulation implementations in this project utilizes significant portions of the implementation from the [Turtlebot3 simulation repository](https://github.com/azeey/turtlebot3_simulations/tree/new_gazebo) by [Addisu Z. Taddese](https://github.com/azeey) and contributors. 
The repository includes mAP evaluation, scripts for label format conversion etc. which makes our cross model evaluations more efficient. For further details, please visit the [official repository](https://github.com/azeey/turtlebot3_simulations/tree/new_gazebo).
