from setuptools import find_packages, setup

package_name = 'jetbot_action_server'

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
    maintainer='jetbot',
    maintainer_email='jenhungho@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_command_action_server = jetbot_action_server.script.jetbot_action_server:main',
        ],
    },
)
