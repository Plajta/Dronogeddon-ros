# Dronogeddon-ros




## How to setup

1. **Navigate to the 'ros_ws/' directory**

	cd ros_ws

2. **Install dependencies**
	
	rosdep update

    rosdep install --from-paths src --ignore-src -r -y

3. **Build the workspace**

	colcon build

4. **Source the setup script**

	source install/setup.bash

5. **Then you can run the nodes**

	ros2 run my_drone telemetry
	
	ros2 run my_drone video

