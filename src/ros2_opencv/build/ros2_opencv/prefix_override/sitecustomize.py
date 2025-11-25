import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/minh/ws_ros2_camera/src/ros2_opencv/install/ros2_opencv'
