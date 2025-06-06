from setuptools import find_packages, setup

package_name = 'visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tello.rviz']),
        ('share/' + package_name + '/config', ['config/camera_info.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LosVocelos',
    maintainer_email='janocelik@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_setup = visualization.rviz_setup:main',
        ],
    },
)
