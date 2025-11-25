import cv2
from ultralytics import YOLO
import numpy as np

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge


class PoseEstimation2NodeClass(Node):

	def __init__(self):
		super().__init__('pose_estimation2_node')
		
		self.bridgeObject = CvBridge()
		
		self.topicNameFrames = 'topic_camera_image'
		
		self.queueSize = 20
		
		self.subscription = self.create_subscription(Image, self.topicNameFrames, self.listener_callbackFunction, self.queueSize)
		self.subscription
		
		# Load YOLO model (YOLOv11-pose - latest version)
		self.model = YOLO('yolo11n-pose.pt')
		
		# Define skeleton connections
		self.connections = [
			(3,1), (1,0), (0,2), (2,4), (1,2), (4,6), (3,5),
			(5,6), (5,7), (7,9), 
			(6,8), (8,10),
			(11,12), (11,13), (13,15),
			(12,14), (14,16),
			(5,11), (6,12)
		]
		
		# Movement tracking for trajectory prediction
		self.position_history = {}  # Dictionary to store position history for each person
		self.history_length = 5  # Number of frames to track
		self.prediction_frames = 10  # Predict 10 frames ahead
		
		self.get_logger().info('Pose Estimation 2 Node initialized')
	
	def get_body_center(self, keypoints):
		"""Calculate the center point of the body"""
		# Get key points with confidence check
		left_shoulder = keypoints[5] if len(keypoints) > 5 and keypoints[5][2] > 0.5 else None
		right_shoulder = keypoints[6] if len(keypoints) > 6 and keypoints[6][2] > 0.5 else None
		left_hip = keypoints[11] if len(keypoints) > 11 and keypoints[11][2] > 0.5 else None
		right_hip = keypoints[12] if len(keypoints) > 12 and keypoints[12][2] > 0.5 else None
		
		# Check if we have enough keypoints
		if left_shoulder is None or right_shoulder is None:
			return None
		
		# Calculate body center
		center_x = (left_shoulder[0] + right_shoulder[0]) / 2
		center_y = (left_shoulder[1] + right_shoulder[1]) / 2
		
		if left_hip is not None and right_hip is not None:
			hip_center_x = (left_hip[0] + right_hip[0]) / 2
			hip_center_y = (left_hip[1] + right_hip[1]) / 2
			center_x = (center_x + hip_center_x) / 2
			center_y = (center_y + hip_center_y) / 2
		
		return (int(center_x), int(center_y))
	
	def predict_movement(self, person_id, current_position):
		"""
		Predict future movement based on position history
		Returns: predicted position, velocity, direction label
		"""
		# Initialize history for new person
		if person_id not in self.position_history:
			self.position_history[person_id] = []
		
		# Add current position to history
		self.position_history[person_id].append(current_position)
		
		# Keep only recent history
		if len(self.position_history[person_id]) > self.history_length:
			self.position_history[person_id].pop(0)
		
		# Need at least 2 positions to calculate movement
		if len(self.position_history[person_id]) < 2:
			return current_position, 0, 0, "Standing Still", (128, 128, 128)
		
		positions = self.position_history[person_id]
		
		# Calculate average velocity over the tracked frames
		velocities_x = []
		velocities_y = []
		
		for i in range(1, len(positions)):
			vx = positions[i][0] - positions[i-1][0]
			vy = positions[i][1] - positions[i-1][1]
			velocities_x.append(vx)
			velocities_y.append(vy)
		
		# Average velocity
		avg_vx = np.mean(velocities_x)
		avg_vy = np.mean(velocities_y)
		
		# Calculate speed
		speed = np.sqrt(avg_vx**2 + avg_vy**2)
		
		# If person is not moving much, return standing still
		if speed < 2:  # Threshold for movement
			return current_position, avg_vx, avg_vy, "Standing Still", (128, 128, 128)
		
		# Predict future position
		predicted_x = current_position[0] + avg_vx * self.prediction_frames
		predicted_y = current_position[1] + avg_vy * self.prediction_frames
		predicted_pos = (int(predicted_x), int(predicted_y))
		
		# Calculate direction angle
		angle = np.degrees(np.arctan2(avg_vx, -avg_vy))
		
		# Get direction label based on movement angle
		direction_label, color = self.get_movement_direction_label(angle, speed)
		
		return predicted_pos, avg_vx, avg_vy, direction_label, color
	
	def get_movement_direction_label(self, angle, speed):
		"""Convert movement angle to direction label"""
		angle = angle % 360
		
		# Add speed indicator
		speed_label = "Fast" if speed > 5 else "Slow"
		
		if -22.5 <= angle < 22.5:
			return f"{speed_label} - Moving Up", (0, 255, 0)
		elif 22.5 <= angle < 67.5:
			return f"{speed_label} - Moving Up-Right", (0, 255, 128)
		elif 67.5 <= angle < 112.5:
			return f"{speed_label} - Moving Right", (0, 255, 255)
		elif 112.5 <= angle < 157.5:
			return f"{speed_label} - Moving Down-Right", (0, 128, 255)
		elif 157.5 <= angle < 180 or -180 <= angle < -157.5:
			return f"{speed_label} - Moving Down", (0, 0, 255)
		elif -157.5 <= angle < -112.5:
			return f"{speed_label} - Moving Down-Left", (128, 0, 255)
		elif -112.5 <= angle < -67.5:
			return f"{speed_label} - Moving Left", (255, 0, 255)
		else:
			return f"{speed_label} - Moving Up-Left", (255, 128, 0)
	
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
		
	def listener_callbackFunction(self, imageMessage):
		self.get_logger().info('The image frame is received')
		
		# Convert ROS message to OpenCV image
		frame = self.bridgeObject.imgmsg_to_cv2(imageMessage)
		
		# Resize frame
		frame = cv2.resize(frame, (640, 720))
		height, width = frame.shape[:2]
		blank_image = np.zeros((height, width, 3), dtype=np.uint8)
		
		# Run YOLO pose detection
		results = self.model(frame, verbose=False)
		frame = results[0].plot()
		
		# Track person index for text positioning
		person_index = 0
		
		# Process keypoints
		for keypoints in results[0].keypoints.data:
			keypoints = keypoints.cpu().numpy() 
			
			# Draw keypoints with labels
			for i, keypoint in enumerate(keypoints):
				x, y, confidence = keypoint
				
				if confidence > 0.7:
					cv2.circle(blank_image, (int(x), int(y)), radius=5, color=(255,0,0), thickness=1)
					cv2.putText(blank_image, f'{i}', (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
			
			# Draw skeleton connections
			for part_a, part_b in self.connections:
				x1, y1, conf1 = keypoints[part_a]
				x2, y2, conf2 = keypoints[part_b]
				
				if conf1 > 0.5 and conf2 > 0.5:
					cv2.line(blank_image, (int(x1), int(y1)), (int(x2), int(y2)), (255,0,255), thickness=2)
			
			# Get body center for movement tracking
			body_center = self.get_body_center(keypoints)
			
			if body_center is not None:
				# Predict movement based on position history
				predicted_pos, vx, vy, movement_label, color = self.predict_movement(person_index, body_center)
				
				# Draw current position
				cv2.circle(frame, body_center, 8, (0, 255, 0), -1)
				cv2.circle(blank_image, body_center, 8, (0, 255, 0), -1)
				
				# Draw trajectory (path from current to predicted)
				cv2.arrowedLine(frame, body_center, predicted_pos, color, 3, tipLength=0.2)
				cv2.arrowedLine(blank_image, body_center, predicted_pos, color, 3, tipLength=0.2)
				
				# Draw predicted position
				cv2.circle(frame, predicted_pos, 10, color, 2)
				cv2.circle(blank_image, predicted_pos, 10, color, 2)
				
				# Draw velocity vector (shorter arrow showing immediate direction)
				immediate_end = (int(body_center[0] + vx * 5), int(body_center[1] + vy * 5))
				cv2.arrowedLine(frame, body_center, immediate_end, (255, 255, 0), 2, tipLength=0.3)
				cv2.arrowedLine(blank_image, body_center, immediate_end, (255, 255, 0), 2, tipLength=0.3)
				
				# Calculate text position based on person index (stacked vertically with spacing)
				text_y_offset = 30 + (person_index * 90)  # 90 pixels spacing between people
				
				# Add person label
				cv2.putText(frame, f"Person {person_index + 1}:", 
						   (10, text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
				cv2.putText(blank_image, f"Person {person_index + 1}:", 
						   (10, text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
				
				# Add text overlay on frame
				cv2.putText(frame, f"Movement: {movement_label}", 
						   (10, text_y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
				cv2.putText(frame, f"Speed: {np.sqrt(vx**2 + vy**2):.1f} px/frame", 
						   (10, text_y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
				
				# Add text overlay on skeleton
				cv2.putText(blank_image, f"Movement: {movement_label}", 
						   (10, text_y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
				cv2.putText(blank_image, f"Speed: {np.sqrt(vx**2 + vy**2):.1f} px/frame", 
						   (10, text_y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			
			person_index += 1
		
		# Display windows
		cv2.imshow('YOLO Detection', frame)
		cv2.imshow('Skeleton', blank_image)
		cv2.waitKey(1)
			
def main(args=None):
	rclpy.init(args=args)
	
	poseNode = PoseEstimation2NodeClass()
	
	rclpy.spin(poseNode)
	
	poseNode.destroy_node()
	
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
