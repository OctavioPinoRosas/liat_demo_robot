import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class DepthCameraNode(Node):
    def __init__(self):
        super().__init__('depth_camera_node')

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

        # Create CvBridge
        self.bridge = CvBridge()

        # Create publisher
        self.publisher = self.create_publisher(Image, 'color_image', 10)

    def capture_and_publish_image(self):
        ret, _, color_frame = self.get_frame()

        # Convert color_frame to ROS Image msg
        ros_image = self.bridge.cv2_to_imgmsg(color_frame, 'bgr8')

        # Publish the image
        self.publisher.publish(ros_image)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

def main(args=None):
    rclpy.init(args=args)

    depth_camera_node = DepthCameraNode()

    while rclpy.ok():
        depth_camera_node.capture_and_publish_image()
        rclpy.spin_once(depth_camera_node)

    depth_camera_node.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
