# Object Detector Using YOLO with ROS2
This README details the functionality of ROS2 nodes that integrate OpenCV for camera operations and harness a pre-trained YOLO model for object detection tasks. The nodes accept an object and a location as ROS parameters, highlight these elements within the camera feed by drawing rectangles, and subsequently publish the pixel coordinates of these regions to a distinct ROS topic.

![image](https://github.com/kamiab-yz/Object-Detector-Using-YOLO-with-ROS2/assets/83370141/341e770f-b9d2-4a1d-9cef-fe494f5e0a26)
## Dependencies

This project requires the following dependencies to be installed:

- **Python**: The core programming language used for the project. Python is essential for running the ROS2 nodes and other scripts.
- **ROS2 Humble**: The ROS2 distribution that provides the framework for running and managing the nodes.
- **OpenCV Library**: Used for camera operations and image processing tasks within the nodes.
- **Ultralytics YOLO Library**: A Python package for object detection with pre-trained YOLO models.

### Installation Instructions

#### Python

Python is required for this project. Ensure you have Python 3.8 or newer installed. You can download Python from the [official website](https://www.python.org/downloads/) or use your system's package manager to install it.

#### ROS2 Humble

Install ROS2 Humble by following the official ROS2 installation guide. Detailed instructions for various operating systems are available on the [ROS2 Documentation](https://docs.ros.org/en/humble/Installation.html) page.

#### OpenCV Library

Install OpenCV using pip, Python's package manager, by running the following command:

```
pip install opencv-python
```
#### Ultralytics YOLO Library

The Ultralytics YOLO library can be installed from PyPI using pip with the following command:

```
pip install ultralytics
```

## Getting Started

To get started with this project, you'll first need to clone the repository to your workspace. Then change the directory name to **src** and build the workspace using:

```
colcon build --symlink-install
```
Then run this code in every terminal you open :
```
source ~/your_ws_name/install/setup.bash
```
or add this line in your **.bashrc** file.

You should also change the path of **coco.txt** file both in **image_subscriber_node.py** and **obj_place_detector_node.py** with respect to your workspace.

## Run the launch file
In you terminal:
```
ros2 launch obj_detector_bringup demo.launch.py
```
This will launch the image publisher node and the object detector node with the default object parameter set to 'bottle' and the default place parameter set to 'chair'. You can view the coordinates being published on a different topic by running the following command in another terminal:
```
ros2 topic echo /obj_place_info
```

## Run the nodes

You also have the option to execute each node with additional parameters such as object name and place name. The permissible options are available in **coco.txt** within the utils directory. To execute the nodes:

```
ros2 run vision_pkg image_publisher 
```
then 

```
ros2 run vision_pkg obj_place_detector --ros-args -p object_name:="bottle" -p place_name:="chair"

```
you can change object name and place name by your chioce from the objects  in **coco.txt**.
Again you can view the coordinates being published on a different topic by running the following command in another terminal:

```
ros2 topic echo /obj_place_info
```
As a demo you can also run another node which detect all the detectable objects in the stream runing this in new twrminal:
```
ros2 run vision_pkg image_subscriber
```



