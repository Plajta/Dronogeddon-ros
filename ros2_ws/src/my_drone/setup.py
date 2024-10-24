from setuptools import find_packages, setup
from glob import glob

package_name = 'my_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/test', glob('test/*')),  
    ],
    install_requires=['setuptools','djitellopy', 'opencv-python'],
    zip_safe=True,
    maintainer='sles',
    maintainer_email='sles@todo.todo',
    description='tello',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_driver = my_drone.drone_driver:main',
            'visualization = my_drone.visualization:main',
            'static_commands = my_drone.static_commands:main',
            'commands_exec = my_drone.command_exec_nodrone_test:main',
            'fail_save = my_drone.fail_save:main',
            'action_server = my_drone.action_server:main',
            'action_client = my_drone.action_client:main',
            'movement = my_drone.lib.actions:main',
            'action_test = my_drone.action_send_test:main',
        ],
    },
)
