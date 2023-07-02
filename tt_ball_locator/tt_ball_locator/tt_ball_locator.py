import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TTBallLocator(LifecycleNode):
    def __init__(self):
        super().__init__('tt_ball_locator')
        self.declare_parameter('rgb_topic')
        self.declare_parameter('depth_topic')
        self.declare_parameter('tf_frame_id')
        self.tf_frame_id = None
        self.rgb_subscriber = None
        self.depth_subscriber = None
        self.tf_broadcaster = None
        self.cv_bridge = CvBridge()  # to convert ROS Image message to OpenCV image
        self.depth_image = None

    def on_configure(self, state):
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.tf_frame_id = self.get_parameter('tf_frame_id').get_parameter_value().string_value
        self.rgb_subscriber = self.create_subscription(Image, rgb_topic, self.rgb_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        return LifecycleTransition.SUCCESS

    def on_activate(self, state):
        return LifecycleTransition.SUCCESS

    def on_deactivate(self, state):
        return LifecycleTransition.SUCCESS

    def on_cleanup(self, state):
        return LifecycleTransition.SUCCESS

    def on_shutdown(self, state):
        return LifecycleTransition.SUCCESS

    def rgb_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to the HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the color range for the green ball
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Perform a series of dilations and erosions to remove any small blobs left in the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize the ball center
        center = None

        # Only proceed if at least one contour was found
        if len(contours) > 0:
            # Find the largest contour in the mask, then use it to compute the minimum enclosing circle
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            center = (int(x), int(y))

            # Now that we have the center of the ball in the color image, we can find the depth
            depth = self.depth_image[center[1], center[0]]

            # calculate the 3D position and broadcast the TF
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "camera_frame"
            tf.child_frame_id = self.tf_frame_id
            tf.transform.translation.x = x * depth
            tf.transform.translation.y = y * depth
            tf.transform.translation.z = depth
            tf.transform.rotation.x = 0
            tf.transform.rotation.y = 0
            tf.transform.rotation.z = 0
            tf.transform.rotation.w = 1
            self.tf_broadcaster.sendTransform(tf)

    def depth_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def main(args=None):
    rclpy.init(args=args)
    tt_ball_locator = TTBallLocator()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tt_ball_locator)
    try:
        executor.spin()
    finally:
        executor.remove_node(tt_ball_locator)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
