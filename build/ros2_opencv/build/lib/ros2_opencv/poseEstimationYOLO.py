import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge


class YOLOPoseEstimationSubscriber(Node):
    """
    YOLO-based pose estimation subscriber that receives camera frames
    and performs human pose detection with skeleton visualization and direction prediction
    """

    def __init__(self):
        super().__init__('yolo_pose_estimation_subscriber')
        
        self.bridgeObject = CvBridge()
        
        # Subscribe to the camera publisher topic
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        
        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callbackFunction,
            self.queueSize
        )
        self.subscription
        
        # Load YOLO pose model
        self.get_logger().info('Loading YOLOv8 pose model...')
        try:
            self.model = YOLO('yolov8n-pose.pt')  # nano model for speed
            self.get_logger().info('YOLOv8 pose model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            self.get_logger().info('Model will be downloaded on first run...')
            self.model = YOLO('yolov8n-pose.pt')
        
        # COCO keypoint names (17 keypoints)
        self.keypoint_names = [
            'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
            'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
            'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
            'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
        ]
        
        # Skeleton connections for visualization
        self.skeleton = [
            [0, 1], [0, 2],  # nose to eyes
            [1, 3], [2, 4],  # eyes to ears
            [0, 5], [0, 6],  # nose to shoulders
            [5, 7], [7, 9],  # left arm
            [6, 8], [8, 10], # right arm
            [5, 6],          # shoulders
            [5, 11], [6, 12], # shoulders to hips
            [11, 12],        # hips
            [11, 13], [13, 15], # left leg
            [12, 14], [14, 16]  # right leg
        ]
        
        self.frame_count = 0
        self.get_logger().info('YOLO Pose Estimation Subscriber initialized')
        self.get_logger().info('Subscribing to topic_camera_image')
        
    def calculate_direction(self, keypoints):
        """
        Calculate the direction a person is facing based on body keypoints
        Returns: direction vector, angle, and body center
        """
        # Get key points with confidence check
        nose = keypoints[0] if len(keypoints) > 0 and keypoints[0][2] > 0.5 else None
        left_shoulder = keypoints[5] if len(keypoints) > 5 and keypoints[5][2] > 0.5 else None
        right_shoulder = keypoints[6] if len(keypoints) > 6 and keypoints[6][2] > 0.5 else None
        left_hip = keypoints[11] if len(keypoints) > 11 and keypoints[11][2] > 0.5 else None
        right_hip = keypoints[12] if len(keypoints) > 12 and keypoints[12][2] > 0.5 else None
        
        # Check if we have enough keypoints
        if nose is None or left_shoulder is None or right_shoulder is None:
            return None, None, None
        
        # Calculate body center
        center_x = (left_shoulder[0] + right_shoulder[0]) / 2
        center_y = (left_shoulder[1] + right_shoulder[1]) / 2
        
        if left_hip is not None and right_hip is not None:
            hip_center_x = (left_hip[0] + right_hip[0]) / 2
            hip_center_y = (left_hip[1] + right_hip[1]) / 2
            center_x = (center_x + hip_center_x) / 2
            center_y = (center_y + hip_center_y) / 2
        
        # Vector from body center to nose (facing direction)
        direction_x = nose[0] - center_x
        direction_y = nose[1] - center_y
        
        # Normalize the direction vector
        magnitude = np.sqrt(direction_x**2 + direction_y**2)
        if magnitude > 0:
            direction_x /= magnitude
            direction_y /= magnitude
        else:
            return None, None, None
        
        # Calculate angle (0 degrees is up/north, clockwise)
        angle = np.degrees(np.arctan2(direction_x, -direction_y))
        
        # Create arrow points
        arrow_length = 120
        start_point = (int(center_x), int(center_y))
        end_point = (int(center_x + direction_x * arrow_length), 
                    int(center_y + direction_y * arrow_length))
        
        return start_point, end_point, angle
    
    def get_direction_label(self, angle):
        """Convert angle to cardinal direction and predicted movement"""
        if angle is None:
            return "Unknown", (128, 128, 128)
        
        angle = angle % 360
        if -22.5 <= angle < 22.5:
            return "North (Moving Forward)", (0, 255, 0)
        elif 22.5 <= angle < 67.5:
            return "North-East (Forward-Right)", (0, 255, 128)
        elif 67.5 <= angle < 112.5:
            return "East (Moving Right)", (0, 255, 255)
        elif 112.5 <= angle < 157.5:
            return "South-East (Back-Right)", (0, 128, 255)
        elif 157.5 <= angle < 180 or -180 <= angle < -157.5:
            return "South (Moving Backward)", (0, 0, 255)
        elif -157.5 <= angle < -112.5:
            return "South-West (Back-Left)", (128, 0, 255)
        elif -112.5 <= angle < -67.5:
            return "West (Moving Left)", (255, 0, 255)
        else:
            return "North-West (Forward-Left)", (255, 128, 0)
    
    def draw_skeleton(self, frame, keypoints):
        """Draw skeleton connections and keypoints"""
        # Draw skeleton connections
        for connection in self.skeleton:
            start_idx, end_idx = connection
            if (start_idx < len(keypoints) and end_idx < len(keypoints) and
                keypoints[start_idx][2] > 0.5 and keypoints[end_idx][2] > 0.5):
                
                start_point = (int(keypoints[start_idx][0]), int(keypoints[start_idx][1]))
                end_point = (int(keypoints[end_idx][0]), int(keypoints[end_idx][1]))
                
                cv2.line(frame, start_point, end_point, (0, 255, 0), 2)
        
        # Draw keypoints
        for i, kpt in enumerate(keypoints):
            if kpt[2] > 0.5:  # confidence threshold
                x, y = int(kpt[0]), int(kpt[1])
                cv2.circle(frame, (x, y), 4, (0, 255, 255), -1)
                cv2.circle(frame, (x, y), 6, (255, 255, 255), 1)
    
    def listener_callbackFunction(self, imageMessage):
        self.frame_count += 1
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Processing YOLO pose frame {self.frame_count}')
        
        # Convert ROS Image to OpenCV
        frame = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        display_frame = frame.copy()
        h, w = frame.shape[:2]
        
        # Run YOLO pose detection
        results = self.model(frame, verbose=False)
        
        # Process detections
        person_detected = False
        for result in results:
            if result.keypoints is not None and len(result.keypoints) > 0:
                for person_keypoints in result.keypoints.data:
                    person_detected = True
                    
                    # Convert to numpy array
                    kpts = person_keypoints.cpu().numpy()
                    
                    # Draw bounding box if available
                    if result.boxes is not None and len(result.boxes) > 0:
                        box = result.boxes[0].xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = map(int, box)
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        
                        # Draw confidence
                        conf = result.boxes[0].conf[0].cpu().numpy()
                        cv2.putText(display_frame, f'Person: {conf:.2f}', 
                                   (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.5, (255, 0, 0), 2)
                    
                    # Draw skeleton
                    self.draw_skeleton(display_frame, kpts)
                    
                    # Calculate direction
                    start_point, end_point, angle = self.calculate_direction(kpts)
                    
                    if start_point is not None:
                        direction_label, color = self.get_direction_label(angle)
                        
                        # Draw direction arrow
                        cv2.arrowedLine(display_frame, start_point, end_point, 
                                      color, 4, tipLength=0.3)
                        
                        # Draw center point
                        cv2.circle(display_frame, start_point, 10, color, -1)
                        cv2.circle(display_frame, start_point, 12, (255, 255, 255), 2)
                        
                        # Add text overlay with semi-transparent background
                        overlay = display_frame.copy()
                        cv2.rectangle(overlay, (5, 5), (w-5, 130), (0, 0, 0), -1)
                        cv2.addWeighted(overlay, 0.4, display_frame, 0.6, 0, display_frame)
                        
                        # Add direction text
                        cv2.putText(display_frame, f"Facing: {direction_label}", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.7, color, 2)
                        cv2.putText(display_frame, f"Angle: {angle:.1f} degrees", 
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.7, (255, 255, 255), 2)
                        
                        # Count detected keypoints
                        detected_kpts = sum(1 for kpt in kpts if kpt[2] > 0.5)
                        cv2.putText(display_frame, f"Keypoints: {detected_kpts}/17 detected", 
                                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.7, (255, 255, 0), 2)
                        
                        cv2.putText(display_frame, "Predicted Movement Direction -->", 
                                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.5, (0, 255, 255), 1)
                    
                    # Only process first person for now
                    break
        
        if not person_detected:
            cv2.putText(display_frame, "No person detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.9, (0, 0, 255), 2)
            cv2.putText(display_frame, "YOLO Pose Detection Active", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 0), 1)
        
        # Display
        cv2.imshow("YOLO Pose Estimation - Skeleton & Movement Prediction", display_frame)
        cv2.waitKey(1)
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    subscriberNode = YOLOPoseEstimationSubscriber()
    
    try:
        rclpy.spin(subscriberNode)
    except KeyboardInterrupt:
        pass
    finally:
        subscriberNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
