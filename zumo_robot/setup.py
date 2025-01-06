from setuptools import find_packages, setup

package_name = 'zumo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_zumo.py']),
    ],
        install_requires=[
        'setuptools', 
        'serial', 
        'opencv-python', 
        'rclpy', 
        'std_msgs',
        'cv_bridge',
        'PyQt5',
        'nav_msgs',
        'tf2_ros'
    ],  # Abhängigkeiten hier einfügen
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'test_led = zumo_robot.test_led:main',
            'camera_node = zumo_robot.camera_node:main',
            'pid_controller_node = zumo_robot.pid_controller_node:main',
            'arduino_node = zumo_robot.arduino_node:main',
            'qt_ros_interface = zumo_robot.qt_interface_node:main',
            'path_mapping_node = zumo_robot.mapping_node:main',
            'tf2_transform_node = zumo_robot.Tf2_Node:main',
            
        ],
    },
)
