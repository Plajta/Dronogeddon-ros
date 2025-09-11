from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_mapper = localization.slam_mapper:main',
            'mission_coordinator = localization.mission_coordinator:main',
            'autonomous_explorer = localization.autonomous_explorer:main',
            'compass_mapping = localization.compass_mapping:main',
            'rviz_visualizer = localization.rviz_visualizer:main',
            'map_saver = localization.map_saver:main',
            'drone_simulator = localization.drone_simulator:main',
            'gazebo_tof_bridge = localization.gazebo_tof_bridge:main',
            'gazebo_drone_controller = localization.gazebo_drone_controller:main',
        ],
    },
)
