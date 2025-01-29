# ROS2 Image Publisher

This project implements a ROS2 image publisher node that publishes images at a specified rate. The node is designed to be easily configurable and can be integrated into larger robotic systems.

## Project Structure

```
ros2_image_publisher
├── src
│   ├── image_publisher.cpp  # Implementation of the image publisher node
│   └── CMakeLists.txt       # CMake configuration file
├── package.xml              # Package manifest
└── README.md                # Project documentation
```

## Dependencies

This project requires the following ROS2 packages:
- `rclcpp`
- `sensor_msgs`
- `image_transport`
- `cv_bridge`
- `OpenCV`

## Building the Project

To build the project, follow these steps:

1. Navigate to the root of the project directory:
   ```
   cd ros2_image_publisher
   ```

2. Install dependencies using `rosdep`:
   ```
   rosdep install -i --from-path src --rosdistro <your_ros2_distro> -y
   ```

3. Build the package using `colcon`:
   ```
   colcon build
   ```

4. Source the setup file:
   ```
   source install/setup.bash
   ```

## Running the Image Publisher Node

To run the image publisher node, use the following command:

```
ros2 run ros2_image_publisher image_publisher
```

Make sure to replace `image_publisher` with the actual name of the executable defined in your `CMakeLists.txt`.

## Configuration

You can configure the image publisher node by modifying parameters in the launch file or by passing them as command-line arguments. Check the source code for available parameters and their default values.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.