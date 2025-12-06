# ws_ros2_camera
Direction + skeleton arrays:
  [person_id, x0, y0, x1, y1, ..., x16, y16, vx, vy, direction_code, confidence, timestamp] 
  person_id: Unique ID to track same person
  17 keypoints: All skeleton joints (x, y)
  velocity: vx, vy in pixels/frame
  direction_code: 0-7 (Up, Up-Right, Right, ...)
  confidence: YOLO detection confidence 0-1
  timestamp: ROS2 time in seconds

CameraPublisher: publish camera data [ros2 run ros2_opencv publisher_node]
subscriberImage: subscriber camera data only [ros2 run ros2_opencv subscriber_node]
poseEstimation2: subscriber camera data + processing + show skeleton and direction detection + publish direction + skeleton data [ros2 run ros2_opencv pose_estimation2_node]
  Can change model (YOLOv8/YOLOv11)
  Can choose modes: View direction data or/and publish (self.enable_publish = True   # Set False to disable publishing pose data + self.enable_display = False   # Set False to disable CV2 windows)
poseDataSubscriber: subscribe direction data (Test data) [ros2 run ros2_opencv pose_data_subscriber_node]
  Data return: Person ID, Velocity (px/frame), Direction code (8 directions), Confidence, Nose position, Shoulder width. (Can add more)
  
