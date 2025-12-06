import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PoseDataSubscriberNode(Node):
	"""
	Simple subscriber to receive and parse pose data.
	
	Data format per person (40 numbers):
	[person_id, 17 keypoints (x,y), vx, vy, direction_code, confidence, timestamp]
	
	Direction codes: 0=Up, 1=Up-Right, 2=Right, 3=Down-Right, 
	                 4=Down, 5=Down-Left, 6=Left, 7=Up-Left
	"""
	
	def __init__(self):
		super().__init__('pose_data_subscriber_node')
		
		self.subscription = self.create_subscription(
			Float32MultiArray,
			'topic_pose_data',
			self.listener_callback,
			10
		)
		
		# Direction labels for display
		self.direction_names = [
			"Up", "Up-Right", "Right", "Down-Right",
			"Down", "Down-Left", "Left", "Up-Left"
		]
		
		self.get_logger().info('Pose Data Subscriber initialized')
	
	def parse_person_data(self, data, start_idx):
		"""
		Parse data for one person starting at start_idx.
		Returns: dict with person info
		"""
		person_id = int(data[start_idx])
		
		# Extract 17 keypoints (x, y)
		keypoints = []
		for i in range(17):
			x = data[start_idx + 1 + i*2]
			y = data[start_idx + 2 + i*2]
			keypoints.append((x, y))
		
		# Extract movement data (after 17 keypoints)
		idx = start_idx + 1 + 34  # 1 (id) + 34 (keypoints)
		vx = data[idx]
		vy = data[idx + 1]
		direction_code = int(data[idx + 2])
		confidence = data[idx + 3]
		timestamp = data[idx + 4]
		
		return {
			'person_id': person_id,
			'keypoints': keypoints,
			'velocity': (vx, vy),
			'direction_code': direction_code,
			'direction_name': self.direction_names[direction_code],
			'confidence': confidence,
			'timestamp': timestamp
		}
	
	def listener_callback(self, msg):
		"""Process received pose data"""
		data = msg.data
		
		# Each person has 40 numbers
		person_data_size = 40
		num_people = len(data) // person_data_size
		
		if num_people == 0:
			return
		
		self.get_logger().info(f'\n=== Received data for {num_people} person(s) ===')
		
		# Parse each person's data
		for i in range(num_people):
			start_idx = i * person_data_size
			person = self.parse_person_data(data, start_idx)
			
			# Display key information
			self.get_logger().info(f'\nPerson {person["person_id"] + 1}:')
			self.get_logger().info(f'  Velocity: ({person["velocity"][0]:.2f}, {person["velocity"][1]:.2f}) px/frame')
			self.get_logger().info(f'  Direction: {person["direction_name"]} (code: {person["direction_code"]})')
			self.get_logger().info(f'  Confidence: {person["confidence"]:.3f}')
			
			# Example: Access specific keypoints
			nose = person['keypoints'][0]  # Keypoint 0 = nose
			left_shoulder = person['keypoints'][5]  # Keypoint 5 = left shoulder
			right_shoulder = person['keypoints'][6]  # Keypoint 6 = right shoulder
			
			self.get_logger().info(f'  Nose position: ({nose[0]:.1f}, {nose[1]:.1f})')
			self.get_logger().info(f'  Shoulder width: {abs(left_shoulder[0] - right_shoulder[0]):.1f} px')


def main(args=None):
	rclpy.init(args=args)
	
	subscriber_node = PoseDataSubscriberNode()
	
	try:
		rclpy.spin(subscriber_node)
	except KeyboardInterrupt:
		pass
	
	subscriber_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
