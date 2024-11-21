# Dronogeddon-ros

Very cool and efficitent autonomous security drone using Dji Tello.

### Used libraries

- `djitellopy`
- `torch` and `torchvision`
- `ROS`

### Setup/Installation
1. **Navigate to the 'ros_ws/' directory**

	```
	cd ros_ws
	```

2. **Setup rosdep and install dependencies**
	```
	sudo rosdep init
	rosdep update
    rosdep install --from-paths src --ignore-src -r -y
	```

3. **Build the workspace**
	*Build all packages*
	```
	colcon build
	```
	*Build specific packages*
	```
	colcon build --packages-select <name-of-pkg>
	```

4. **Source the setup script**
	```
	source install/setup.bash
	```

5. **Run nodes**
	*Manually*
	```
	ros2 run my_drone telemetry
	ros2 run my_drone video
	```

	*Using a launch file*
	```
	ros2 launch <path-to-xml>
	```

### Configuration files

| Config file        | Description                                    |
| ------------------ | ---------------------------------------------- |
| main_drone.xml     | main file for testing with drone               |
| main_undrone.xml   | main file for testing with fake drone (camera) |
| telemetry_test.xml | just for testing drone telemetry               |