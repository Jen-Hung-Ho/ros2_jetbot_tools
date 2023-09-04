from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'jetbot_tools'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all laumch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all param files
        (os.path.join('share', package_name, 'param'), glob('param/*params.yaml')),
        # Include all includ file
        (os.path.join('share', package_name, 'include'), glob('include/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jen-Hung Ho',
    maintainer_email='jetbot@todo.todo',
    description='ROS2 nodes for Jetbot tools',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'img_subscriber = jetbot_tools.script.ros2_image_subscriber:main',
        'detect_copilot = jetbot_tools.script.detect_copilot:main',
        'follow_copilot = jetbot_tools.script.follow_copilot:main',
        'laser_avoidance = jetbot_tools.script.laser_avoidance:main',
        'tf2_follow = jetbot_tools.script.tf2_listener_copilot:main',
        'calibrate_angular = jetbot_tools.script.calibrate_angular:main',
        'calibrate_linear = jetbot_tools.script.calibrate_linear:main'
        ],
    },
)
