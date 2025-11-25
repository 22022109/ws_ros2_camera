import cv2
import numpy as np
import urllib.request
import os

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge


class PoseEstimationSubscriber(Node):

    def __init__(self):
        super().__init__('pose_estimation_subscriber')
        
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
        
        # OpenPose model setup
        self.model_path = os.path.expanduser("~/.ros2_pose_models")
        os.makedirs(self.model_path, exist_ok=True)
        
        self.proto_file = os.path.join(self.model_path, "pose_deploy_linevec.prototxt")
        self.weights_file = os.path.join(self.model_path, "pose_iter_440000.caffemodel")
        
        # Download model files if they don't exist
        self.download_models()
        
        # Load the network
        self.net = cv2.dnn.readNetFromCaffe(self.proto_file, self.weights_file)
        
        # COCO body parts (18 keypoints)
        self.BODY_PARTS = {
            "Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
            "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
            "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
            "LEye": 15, "REar": 16, "LEar": 17
        }
        
        # Pairs for skeleton connections
        self.POSE_PAIRS = [
            ["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
            ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
            ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
            ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
            ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]
        ]
        
        self.threshold = 0.1
        self.frame_count = 0
        self.get_logger().info('Pose Estimation Subscriber initialized - subscribing to topic_camera_image')
    
    def download_models(self):
        """Download OpenPose models if not present"""
        try:
            if not os.path.exists(self.proto_file):
                self.get_logger().info('Downloading pose model prototxt...')
                url = "https://raw.githubusercontent.com/CMU-Perceptual-Computing-Lab/openpose/master/models/pose/coco/pose_deploy_linevec.prototxt"
                urllib.request.urlretrieve(url, self.proto_file)
                self.get_logger().info('Prototxt downloaded successfully')
            
            if not os.path.exists(self.weights_file):
                self.get_logger().info('Downloading pose model weights (this may take a while)...')
                self.get_logger().info('Download URL: http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/coco/pose_iter_440000.caffemodel')
                url = "http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/coco/pose_iter_440000.caffemodel"
                urllib.request.urlretrieve(url, self.weights_file)
                self.get_logger().info('Model weights downloaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to download models: {str(e)}')
            self.get_logger().error(f'Please manually download the files:')
            self.get_logger().error(f'1. Prototxt: https://raw.githubusercontent.com/CMU-Perceptual-Computing-Lab/openpose/master/models/pose/coco/pose_deploy_linevec.prototxt')
            self.get_logger().error(f'   Save to: {self.proto_file}')
            self.get_logger().error(f'2. Model weights: http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/coco/pose_iter_440000.caffemodel')
            self.get_logger().error(f'   Save to: {self.weights_file}')
            raise
        
    def calculate_direction(self, points):
        """
        Calculate the direction a person is facing based on shoulder and hip orientation
        Returns: direction vector (x, y) and angle in degrees
        """
        # Check if required points exist
        required_parts = ["Nose", "Neck", "LShoulder", "RShoulder", "LHip", "RHip"]
        if not all(part in points and points[part] is not None for part in required_parts):
            return None, None, None
        
        nose = points["Nose"]
        neck = points["Neck"]
        left_shoulder = points["LShoulder"]
        right_shoulder = points["RShoulder"]
        left_hip = points["LHip"]
        right_hip = points["RHip"]
        
        # Calculate shoulder midpoint
        shoulder_mid_x = (left_shoulder[0] + right_shoulder[0]) / 2
        shoulder_mid_y = (left_shoulder[1] + right_shoulder[1]) / 2
        
        # Calculate hip midpoint
        hip_mid_x = (left_hip[0] + right_hip[0]) / 2
        hip_mid_y = (left_hip[1] + right_hip[1]) / 2
        
        # Calculate body center
        body_center_x = (shoulder_mid_x + hip_mid_x) / 2
        body_center_y = (shoulder_mid_y + hip_mid_y) / 2
        
        # Vector from body center to nose (facing direction)
        direction_x = nose[0] - body_center_x
        direction_y = nose[1] - body_center_y
        
        # Normalize the direction vector
        magnitude = np.sqrt(direction_x**2 + direction_y**2)
        if magnitude > 0:
            direction_x /= magnitude
            direction_y /= magnitude
        
        # Calculate angle (0 degrees is up, clockwise)
        angle = np.degrees(np.arctan2(direction_x, -direction_y))
        
        # Convert to pixel coordinates for visualization
        center_pixel_x = int(body_center_x)
        center_pixel_y = int(body_center_y)
        
        arrow_length = 150
        end_x = int(center_pixel_x + direction_x * arrow_length)
        end_y = int(center_pixel_y + direction_y * arrow_length)
        
        return (center_pixel_x, center_pixel_y), (end_x, end_y), angle
    
    def get_direction_label(self, angle):
        """Convert angle to cardinal direction and predict movement"""
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
        
    def listener_callbackFunction(self, imageMessage):
        self.frame_count += 1
        
        if self.frame_count % 10 == 0:  # Log every 10 frames
            self.get_logger().info(f'Processing pose estimation frame {self.frame_count}')
        
        # Convert ROS Image message to OpenCV image
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        frame = openCVImage.copy()
        frameHeight, frameWidth = frame.shape[:2]
        
        # Prepare input blob for the network
        inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (368, 368), (0, 0, 0), swapRB=False, crop=False)
        self.net.setInput(inpBlob)
        
        # Forward pass to get output
        output = self.net.forward()
        H = output.shape[2]
        W = output.shape[3]
        
        # Store detected points
        points = {}
        
        # Extract keypoints
        for part_name, part_id in self.BODY_PARTS.items():
            # Slice heatmap of corresponding body part
            probMap = output[0, part_id, :, :]
            
            # Find global maxima of the probMap
            minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
            
            # Scale the point to fit on the original image
            x = int((frameWidth * point[0]) / W)
            y = int((frameHeight * point[1]) / H)
            
            if prob > self.threshold:
                points[part_name] = (x, y)
                # Draw circle at keypoint
                cv2.circle(frame, (x, y), 8, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                cv2.putText(frame, str(part_id), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
            else:
                points[part_name] = None
        
        # Draw skeleton
        for pair in self.POSE_PAIRS:
            partFrom = pair[0]
            partTo = pair[1]
            
            if partFrom in points and partTo in points:
                if points[partFrom] is not None and points[partTo] is not None:
                    cv2.line(frame, points[partFrom], points[partTo], (0, 255, 0), 3)
        
        # Calculate and draw direction
        start_point, end_point, angle = self.calculate_direction(points)
        
        if start_point is not None:
            # Get direction label and color
            direction_label, color = self.get_direction_label(angle)
            
            # Draw direction arrow
            cv2.arrowedLine(frame, start_point, end_point, color, 4, tipLength=0.3)
            
            # Draw direction circle at center
            cv2.circle(frame, start_point, 10, color, -1)
            cv2.circle(frame, start_point, 12, (255, 255, 255), 2)
            
            # Add text overlay with semi-transparent background
            overlay = frame.copy()
            cv2.rectangle(overlay, (5, 5), (frameWidth-5, 120), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)
            
            # Add direction text
            cv2.putText(frame, f"Facing: {direction_label}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, color, 2)
            cv2.putText(frame, f"Angle: {angle:.1f} degrees", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 255), 2)
            
            # Count detected joints
            detected_joints = sum(1 for p in points.values() if p is not None)
            cv2.putText(frame, f"Joints detected: {detected_joints}/18", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)
            
            # Add predicted movement indicator
            cv2.putText(frame, "Predicted Movement Direction -->", 
                       (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (0, 255, 255), 1)
        else:
            cv2.putText(frame, "No person detected or incomplete pose", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.9, (0, 0, 255), 2)
        
        # Display the annotated frame
        cv2.imshow("Pose Estimation - Skeleton & Movement Prediction", frame)
        cv2.waitKey(1)
        
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()
        
        
def main(args=None):
    rclpy.init(args=args)
    
    subscriberNode = PoseEstimationSubscriber()
    
    try:
        rclpy.spin(subscriberNode)
    except KeyboardInterrupt:
        pass
    finally:
        subscriberNode.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
