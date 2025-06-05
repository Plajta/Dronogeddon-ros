# Dronogeddon-ros

Very cool and efficitent autonomous security drone using Dji Tello.

### Used libraries

All written in the `requirements.txt` file. Apart from that, we use the **ROS** in our codebase, which you have to configure separately.

### Setup/Installation

1. **Navigate to the 'ros2_ws/' directory**

```
cd ros2_ws
```

2. **Source the setup script**

``` shell
source install/setup.bash
```

3. **Create virtual environment (optional)** <br>

*Either using venv*

``` shell
python3 -m venv dronogeddon_env --system-site-packages --symlinks
source dronogeddon_env/bin/activate
```

*Or using Pyenv virtualenv plugin*

``` shell
pyenv install 3.10
pyenv virtualenv 3.10 dronogeddon_env --system-site-packages --symlinks
pyenv local dronogeddon_env
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

```
python3 -m pip install -r requirements.txt
```

7. **Run nodes**

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