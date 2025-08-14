#!/usr/bin/env python3
import sys
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from image_action_msgs.action import CompareImages


class CompareImagesClient(Node):
    def __init__(self, img1_path, img2_path, description):
        super().__init__('compare_images_client')
        self.cli = ActionClient(self, CompareImages, 'compare_images')
        self.bridge = CvBridge()
        self.img1_path = img1_path
        self.img2_path = img2_path
        self.description = description

    def send_goal(self):
        if not self.cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Server not available')
            return

        img1_cv = cv2.imread(self.img1_path, cv2.IMREAD_COLOR)
        img2_cv = cv2.imread(self.img2_path, cv2.IMREAD_COLOR)
        if img1_cv is None or img2_cv is None:
            self.get_logger().error('Could not read one or both images.')
            return

        goal = CompareImages.Goal()
        goal.image1 = self.bridge.cv2_to_imgmsg(img1_cv, encoding='bgr8')
        goal.image2 = self.bridge.cv2_to_imgmsg(img2_cv, encoding='bgr8')
        goal.description = self.description

        self.get_logger().info(f"Sending goal: {self.description}")
        send_future = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Result: {result.result}")

def main():
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <image1> <image2> <description>")
        return
    rclpy.init()
    node = CompareImagesClient(sys.argv[1], sys.argv[2], sys.argv[3])
    node.send_goal()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
