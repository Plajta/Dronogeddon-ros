from setuptools import find_packages, setup

package_name = 'undrone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'webcam = undrone.webcam:main',
            'untelemetry = undrone.untelemetry:main',
            'drone = undrone.full_undrone:main',
            'keys = undrone.key_control:main'
        ],
    },
)
