from setuptools import find_packages, setup

package_name = 'ros2_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minh',
    maintainer_email='22022109@vnu.edu.vn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher_node=ros2_opencv.cameraPublisher:main',
            'subscriber_node=ros2_opencv.subscriberImage:main',
            'pose_estimation_node=ros2_opencv.poseEstimationSubscriber:main',
            'pose_simple_node=ros2_opencv.poseEstimationSimple:main',
            'pose_yolo_node=ros2_opencv.poseEstimationYOLO:main',
            'pose_estimation2_node=ros2_opencv.poseEstimation2:main',
        ],
    },
)
