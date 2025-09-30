import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amr/ros2_ws/ros2_turtle_shapes/install/ros2_turtle_shapes'
