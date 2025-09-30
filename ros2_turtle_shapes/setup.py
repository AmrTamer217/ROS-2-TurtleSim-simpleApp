from setuptools import setup

package_name = 'ros2_turtle_shapes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle_shapes_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amr Tamer',
    maintainer_email='amrtamersm@gmail.com',
    description='ROS2 package for drawing shapes with turtlesim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shape_node = ros2_turtle_shapes.shape_node:main',
            'turtle_commander = ros2_turtle_shapes.turtle_commander:main',
        ],
    },
)