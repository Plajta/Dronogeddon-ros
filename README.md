# Dronogeddon-ros

Very cool and efficitent autonomous security drone using Dji Tello.

### Used libraries

All written in the `requirements.txt` file. Apart from that, we use the **ROS Jazzy** with **Python 3.10** in our codebase.

### Setup/Installation

1. **Navigate to the 'ros2_ws/' directory**

```
cd ros2_ws
```

2. **Source the global ROS setup script**

``` shell
source /opt/ros/jazzy/setup.bash
```

3. **Setup Python 3.10 version** <br>

*You can use PyEnv for that, if you already have python 3.10 installed globaly, then this step is entirely optional*

``` shell
pyenv install 3.10
pyenv local 3.10
```

4. **Setup rosdep and install dependencies**

``` shell
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

5. **Install other dependencies**

``` shell
pip install -r requirements.txt
```

6. **Build the workspace** <br>

*Build all packages*
``` shell
colcon build
```
*Build specific packages*
``` shell
colcon build --packages-select <name-of-pkg>
```

7. **Source the local ROS setup script**

```
source install/setup.bash
```

### Usage

##### Running nodes

*Manually*

```
ros2 run my_drone telemetry
ros2 run my_drone video
```

*Using a launch file*

```
ros2 launch <path-to-xml>
```

### Launch files

| Config file        | Description                                    |
| ------------------ | ---------------------------------------------- |
| main_drone.xml     | main file for testing with drone               |
| main_undrone.xml   | main file for testing with fake drone (camera) |
| telemetry_test.xml | just for testing drone telemetry               |