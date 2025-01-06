# Camera-Based Line Following and Mapping for Autonomous Navigation with ROS2

This project aims to enhance the Zumo robot to operate based on camera-based image processing. Using a camera, the robot will follow a predefined line. Image processing will be performed using OpenCV, and the integration of image recognition and robot control will be implemented with the ROS2 framework. As a potential extension, the robot can be equipped with wheel encoders to map the traversed line, which can then be further processed and visualized in ROS2.

## Project Tasks

1. **Line Following:** The Zumo robot detects the path of a line using its camera and OpenCV.
2. **Image Processing:** The detected line is processed in OpenCV and converted into coordinates.
3. **Control:** The coordinates are processed in ROS2 and converted into driving commands for the robot.
4. **(Optional) Mapping:** Implementing wheel encoders for data acquisition and creating a map of the traversed path in ROS2.
5. **Validation:** Testing and evaluating potential use cases.

## Required Hardware

- **Zumo Robot with Arduino Controller:** The base hardware, including motors. This project uses the Zumo Shield for Arduino v1.2, which does not come pre-equipped with encoders. However, encoders have been mounted onto the Zumo for this project.
- **External Camera:** A Linux-compatible webcam.
- **Laptop with ROS2:** For robot control, image processing, and ROS2 node execution.
- **Predefined Track:** A black line on a white background for line-following experiments.

## Technology Stack

- **Programming Languages:** Python (for OpenCV and ROS2 nodes).
- **Libraries:**
  - OpenCV for image processing.
  - ROS2 for robot control and node management.
- **Hardware Integration:**
  - Arduino-based Zumo robot for movement.
  - External camera for line detection.

## Important Commands

To set up the environment, the following commands can be used:

```bash
sudo apt install python3-pip
pip install opencv-python
pip install serial
sudo apt remove brltty
pip install pyserial
```

## Future Extensions

- Equip the Zumo robot with wheel encoders to improve navigation and mapping capabilities.
- Develop a feature for real-time visualization of the traversed line in ROS2.

---

### Note

This project is currently under development. Contributions, suggestions, and feedback are welcome!
