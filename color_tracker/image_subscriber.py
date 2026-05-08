import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2
import numpy as np


class ImageSubscriber(Node):

    def __init__(self):

        super().__init__('image_subscriber')

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.bridge = CvBridge()

        # Proportional gain
        self.kp = 0.002

        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

        self.get_logger().info(
            "Color Tracking Navigation Started"
        )


    def image_callback(self, msg):

        # Convert ROS image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Get image size
        height, width, _ = frame.shape

        # Image center
        image_center_x = width // 2

        # Convert image to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green color range
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Velocity message
        twist = Twist()

        if contours:

            # Largest contour
            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            # Ignore tiny objects
            if cv2.contourArea(largest_contour) > 500:

                # Moments
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:

                    # Centroid
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Horizontal error
                    error_x = cx - image_center_x

                    # P Controller
                    angular_z = -self.kp * error_x

                    # Alignment threshold
                    threshold = 30

                    # If aligned -> move forward
                    if abs(error_x) < threshold:

                        twist.linear.x = 0.15
                        twist.angular.z = 0.0

                        motion = "MOVING FORWARD"

                    # Otherwise rotate
                    else:

                        twist.linear.x = 0.0
                        twist.angular.z = angular_z

                        motion = "ROTATING"

                    # Publish velocity
                    self.cmd_pub.publish(twist)

                    # Draw contour
                    cv2.drawContours(
                        frame,
                        [largest_contour],
                        -1,
                        (0, 255, 0),
                        2
                    )

                    # Draw centroid
                    cv2.circle(
                        frame,
                        (cx, cy),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    # Draw center line
                    cv2.line(
                        frame,
                        (image_center_x, 0),
                        (image_center_x, height),
                        (255, 0, 0),
                        2
                    )

                    # Show error
                    cv2.putText(
                        frame,
                        f'Error: {error_x}',
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2
                    )

                    # Show robot state
                    cv2.putText(
                        frame,
                        f'State: {motion}',
                        (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 0, 0),
                        2
                    )

                    # Terminal log
                    self.get_logger().info(
                        f'Error: {error_x}, Angular Z: {angular_z:.2f}, State: {motion}'
                    )

        else:

            # Stop robot if no object
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.cmd_pub.publish(twist)

        # Show windows
        cv2.imshow("Camera Feed", frame)
        cv2.imshow("Mask", mask)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = ImageSubscriber()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
