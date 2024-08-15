from setuptools import find_packages, setup

package_name = 'my_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'telemetry = my_drone.publisher_member_function:main',
            'video = my_drone.subscriber_member_function:main',
            'tello_test = my_drone.test:main',
        ],
    },
)
