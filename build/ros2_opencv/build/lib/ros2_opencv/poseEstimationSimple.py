import cv2
import numpy as np
import os

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge


class SimplePoseEstimationSubscriber(Node):
    """
    Simple pose estimation using OpenCV's cascade classifiers and contour analysis
    This doesn't require external model downloads
    """

    def __init__(self):
        super().__init__('simple_pose_estimation_subscriber')
        
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
        
        # Find haarcascades directory
        cascade_paths = [
            '/home/minh/anaconda3/share/opencv4/haarcascades/',
            '/home/minh/anaconda3/envs/opencv-env/share/opencv4/haarcascades/',
            '/usr/share/opencv4/haarcascades/',
            '/usr/local/share/opencv4/haarcascades/',
        ]
        
        cascade_dir = None
        for path in cascade_paths:
            if os.path.exists(path):
                cascade_dir = path
                break
        
        if cascade_dir is None:
            self.get_logger().error('Could not find haarcascades directory')
            raise FileNotFoundError('Haarcascades not found')
        
        self.get_logger().info(f'Using cascade directory: {cascade_dir}')
        
        # Load Haar cascade for upper body detection
        upper_body_path = os.path.join(cascade_dir, 'haarcascade_upperbody.xml')
        face_path = os.path.join(cascade_dir, 'haarcascade_frontalface_default.xml')
        profile_path = os.path.join(cascade_dir, 'haarcascade_profileface.xml')
        fullbody_path = os.path.join(cascade_dir, 'haarcascade_fullbody.xml')
        
        # Load cascades
        if os.path.exists(upper_body_path):
            self.upper_body_cascade = cv2.CascadeClassifier(upper_body_path)
        elif os.path.exists(fullbody_path):
            self.upper_body_cascade = cv2.CascadeClassifier(fullbody_path)
            self.get_logger().info('Using fullbody cascade instead of upperbody')
        else:
            self.get_logger().error('No body cascade found')
            raise FileNotFoundError('Body cascade not found')
        
        if os.path.exists(face_path):
            self.face_cascade = cv2.CascadeClassifier(face_path)
        else:
            self.get_logger().error('Face cascade not found')
            raise FileNotFoundError('Face cascade not found')
        
        if os.path.exists(profile_path):
            self.profile_cascade = cv2.CascadeClassifier(profile_path)
        else:
            self.get_logger().warn('Profile cascade not found, will use frontal only')
            self.profile_cascade = None
        
        # Background subtractor for motion detection
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=16, detectShadows=True
        )
        
        self.frame_count = 0
        self.prev_center = None
        self.movement_history = []
        
        self.get_logger().info('Simple Pose Estimation Subscriber initialized')
        self.get_logger().info('Subscribing to topic_camera_image')
        
    def detect_person_and_direction(self, frame):
        """Detect person, estimate pose joints, and predict movement direction"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape[:2]
        
        # Detect upper body
        upper_bodies = self.upper_body_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50)
        )
        
        # Detect faces (frontal and profile)
        faces_frontal = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )
        
        # Profile detection only if cascade is available
        faces_profile_left = []
        faces_profile_right = []
        if self.profile_cascade is not None:
            faces_profile_left = self.profile_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
            )
            faces_profile_right = self.profile_cascade.detectMultiScale(
                cv2.flip(gray, 1), scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
            )
        
        # Background subtraction for motion
        fg_mask = self.bg_subtractor.apply(frame)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        
        results = {
            'person_detected': len(upper_bodies) > 0 or len(faces_frontal) > 0,
            'upper_bodies': upper_bodies,
            'faces_frontal': faces_frontal,
            'faces_profile_left': faces_profile_left,
            'faces_profile_right': faces_profile_right,
            'motion_mask': fg_mask
        }
        
        return results
    
    def estimate_joints(self, frame, upper_body_rect, face_rect=None):
        """Estimate approximate joint locations based on body detection"""
        x, y, w_body, h_body = upper_body_rect
        
        joints = {}
        
        # Estimate joint positions based on body proportions
        if face_rect is not None:
            fx, fy, fw, fh = face_rect
            joints['nose'] = (fx + fw//2, fy + fh//2)
            joints['neck'] = (fx + fw//2, fy + fh)
        else:
            # Estimate head/neck from upper body
            joints['nose'] = (x + w_body//2, y + int(h_body * 0.15))
            joints['neck'] = (x + w_body//2, y + int(h_body * 0.25))
        
        # Shoulders
        shoulder_y = y + int(h_body * 0.3)
        joints['left_shoulder'] = (x + int(w_body * 0.25), shoulder_y)
        joints['right_shoulder'] = (x + int(w_body * 0.75), shoulder_y)
        
        # Elbows
        elbow_y = y + int(h_body * 0.55)
        joints['left_elbow'] = (x + int(w_body * 0.15), elbow_y)
        joints['right_elbow'] = (x + int(w_body * 0.85), elbow_y)
        
        # Hips
        hip_y = y + int(h_body * 0.85)
        joints['left_hip'] = (x + int(w_body * 0.35), hip_y)
        joints['right_hip'] = (x + int(w_body * 0.65), hip_y)
        
        # Body center
        joints['center'] = (x + w_body//2, y + h_body//2)
        
        return joints
    
    def calculate_direction(self, joints, face_type='frontal'):
        """Calculate facing direction based on joints and face detection"""
        if 'nose' not in joints or 'center' not in joints:
            return None, None, None
        
        nose = joints['nose']
        center = joints['center']
        
        # Vector from center to nose
        dx = nose[0] - center[0]
        dy = nose[1] - center[1]
        
        # Adjust based on face type detected
        if face_type == 'profile_left':
            dx = -abs(dx)  # Facing left
        elif face_type == 'profile_right':
            dx = abs(dx)  # Facing right
        
        # Normalize
        magnitude = np.sqrt(dx**2 + dy**2)
        if magnitude > 0:
            dx /= magnitude
            dy /= magnitude
        
        # Calculate angle
        angle = np.degrees(np.arctan2(dx, -dy))
        
        # Direction arrow
        arrow_length = 100
        start_point = center
        end_point = (int(center[0] + dx * arrow_length), int(center[1] + dy * arrow_length))
        
        return start_point, end_point, angle
    
    def get_direction_label(self, angle):
        """Convert angle to direction and color"""
        if angle is None:
            return "Unknown", (128, 128, 128)
        
        angle = angle % 360
        if -22.5 <= angle < 22.5:
            return "Forward", (0, 255, 0)
        elif 22.5 <= angle < 67.5:
            return "Forward-Right", (0, 255, 128)
        elif 67.5 <= angle < 112.5:
            return "Right", (0, 255, 255)
        elif 112.5 <= angle < 157.5:
            return "Back-Right", (0, 128, 255)
        elif 157.5 <= angle < 180 or -180 <= angle < -157.5:
            return "Backward", (0, 0, 255)
        elif -157.5 <= angle < -112.5:
            return "Back-Left", (128, 0, 255)
        elif -112.5 <= angle < -67.5:
            return "Left", (255, 0, 255)
        else:
            return "Forward-Left", (255, 128, 0)
    
    def draw_skeleton(self, frame, joints):
        """Draw skeleton connections"""
        # Define skeleton connections
        connections = [
            ('nose', 'neck'),
            ('neck', 'left_shoulder'),
            ('neck', 'right_shoulder'),
            ('left_shoulder', 'left_elbow'),
            ('right_shoulder', 'right_elbow'),
            ('neck', 'center'),
            ('center', 'left_hip'),
            ('center', 'right_hip'),
        ]
        
        # Draw connections
        for joint1, joint2 in connections:
            if joint1 in joints and joint2 in joints:
                cv2.line(frame, joints[joint1], joints[joint2], (0, 255, 0), 2)
        
        # Draw joints
        for joint_name, pos in joints.items():
            cv2.circle(frame, pos, 5, (0, 255, 255), -1)
            cv2.circle(frame, pos, 7, (255, 255, 255), 1)
    
    def listener_callbackFunction(self, imageMessage):
        self.frame_count += 1
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Processing frame {self.frame_count}')
        
        # Convert ROS Image to OpenCV
        frame = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        display_frame = frame.copy()
        h, w = frame.shape[:2]
        
        # Detect person and estimate pose
        results = self.detect_person_and_direction(frame)
        
        if results['person_detected']:
            # Get the largest upper body detection
            if len(results['upper_bodies']) > 0:
                # Sort by size and take the largest
                upper_bodies = sorted(results['upper_bodies'], key=lambda x: x[2]*x[3], reverse=True)
                main_body = upper_bodies[0]
                x, y, w_body, h_body = main_body
                
                # Draw body bounding box
                cv2.rectangle(display_frame, (x, y), (x + w_body, y + h_body), (255, 0, 0), 2)
                
                # Find corresponding face
                face_rect = None
                face_type = 'frontal'
                
                if len(results['faces_frontal']) > 0:
                    face_rect = results['faces_frontal'][0]
                    face_type = 'frontal'
                elif len(results['faces_profile_left']) > 0:
                    face_rect = results['faces_profile_left'][0]
                    face_type = 'profile_left'
                elif len(results['faces_profile_right']) > 0:
                    fx, fy, fw, fh = results['faces_profile_right'][0]
                    face_rect = (w - fx - fw, fy, fw, fh)  # Flip back coordinates
                    face_type = 'profile_right'
                
                # Draw face
                if face_rect is not None:
                    fx, fy, fw, fh = face_rect
                    cv2.rectangle(display_frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
                
                # Estimate joints
                joints = self.estimate_joints(display_frame, main_body, face_rect)
                
                # Draw skeleton
                self.draw_skeleton(display_frame, joints)
                
                # Calculate direction
                start_point, end_point, angle = self.calculate_direction(joints, face_type)
                
                if start_point is not None:
                    direction_label, color = self.get_direction_label(angle)
                    
                    # Draw direction arrow
                    cv2.arrowedLine(display_frame, start_point, end_point, color, 4, tipLength=0.3)
                    cv2.circle(display_frame, start_point, 10, color, -1)
                    cv2.circle(display_frame, start_point, 12, (255, 255, 255), 2)
                    
                    # Add overlay
                    overlay = display_frame.copy()
                    cv2.rectangle(overlay, (5, 5), (w-5, 120), (0, 0, 0), -1)
                    cv2.addWeighted(overlay, 0.4, display_frame, 0.6, 0, display_frame)
                    
                    # Add text
                    cv2.putText(display_frame, f"Facing: {direction_label}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    cv2.putText(display_frame, f"Angle: {angle:.1f} degrees", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(display_frame, f"Joints: {len(joints)} detected", 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(display_frame, "Movement Direction -->", 
                               (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            cv2.putText(display_frame, "No person detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        
        # Display
        cv2.imshow("Simple Pose Estimation - Movement Prediction", display_frame)
        cv2.waitKey(1)
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    subscriberNode = SimplePoseEstimationSubscriber()
    
    try:
        rclpy.spin(subscriberNode)
    except KeyboardInterrupt:
        pass
    finally:
        subscriberNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
