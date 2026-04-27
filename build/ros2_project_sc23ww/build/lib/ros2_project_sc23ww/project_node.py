import math
import random
import threading
import signal

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ROSInterruptException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge, CvBridgeError

class ProjectNode(Node):
    def __init__(self):
        #Setting up everything and initialising the detection + nav variables
        super().__init__('project_node')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.map_data = None

        #Havent seen anything yet

        self.red_seen = False
        self.green_seen = False
        self.blue_seen = False

        self.blue_centre_x = None
        self.blue_area = 0
        self.image_width = 640

        self.goal_active = False
        self.goal_handle = None
        self.finished = False

        self.get_logger().info('Project node started.')

    #Seeing where is safe for the roboy
    def map_callback(self, msg):
        self.map_data = msg

    def cell_is_safe(self, cell_x, cell_y): #Whether the cell is safe for the robot to drive to
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = self.map_data.data

        safety_radius = 8

        for dx in range(-safety_radius, safety_radius + 1):
            for dy in range(-safety_radius, safety_radius + 1):
                nx = cell_x + dx
                ny = cell_y + dy

                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    return False

                value = data[ny * width + nx]

                if value != 0:
                    return False

        return True

    def random_safe_pose(self): # chooses a random sage nav goal from the map
        if self.map_data is None:
            self.get_logger().warn('No map received yet.')
            return None

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        data = self.map_data.data

        for _ in range(3000):
            cell_x = random.randint(0, width - 1)
            cell_y = random.randint(0, height - 1)

            if data[cell_y * width + cell_x] != 0:
                continue

            if not self.cell_is_safe(cell_x, cell_y):
                continue

            x = origin_x + (cell_x + 0.5) * resolution
            y = origin_y + (cell_y + 0.5) * resolution
            yaw = random.uniform(-math.pi, math.pi)

            return x, y, yaw

        self.get_logger().warn('Could not find a safe random pose.')
        return None

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        self.image_width = image.shape[1]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_mask = self.make_red_mask(hsv)
        green_mask = self.make_mask(hsv, 60, 15)
        blue_mask = self.make_mask(hsv, 120, 15)

        self.red_seen = self.process_colour(red_mask, image, 'red', (0, 0, 255))
        self.green_seen = self.process_colour(green_mask, image, 'green', (0, 255, 0))
        self.blue_seen = self.process_colour(blue_mask, image, 'blue', (255, 0, 0))

        cv2.namedWindow('processed_camera_feed', cv2.WINDOW_NORMAL)
        cv2.imshow('processed_camera_feed', image)
        cv2.resizeWindow('processed_camera_feed', 640, 480)
        cv2.waitKey(3)

    def make_mask(self, hsv, hue, sensitivity):
        lower = np.array([hue - sensitivity, 100, 100])
        upper = np.array([hue + sensitivity, 255, 255])
        return cv2.inRange(hsv, lower, upper)

    def make_red_mask(self, hsv): #For detecting red 
        sensitivity = 15

        lower_1 = np.array([0, 100, 100])
        upper_1 = np.array([sensitivity, 255, 255])

        lower_2 = np.array([180 - sensitivity, 100, 100])
        upper_2 = np.array([180, 255, 255])

        mask_1 = cv2.inRange(hsv, lower_1, upper_1)
        mask_2 = cv2.inRange(hsv, lower_2, upper_2)

        return cv2.bitwise_or(mask_1, mask_2)

    def process_colour(self, mask, image, name, draw_colour): #Whether the colour detected actually exists
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            if name == 'blue':
                self.blue_centre_x = None
                self.blue_area = 0
            return False

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)

        if area < 300:
            if name == 'blue':
                self.blue_centre_x = None
                self.blue_area = 0
            return False

        x, y, w, h = cv2.boundingRect(contour)

        cv2.rectangle(image, (x, y), (x + w, y + h), draw_colour, 2)
        cv2.putText(
            image,
            name,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            draw_colour,
            2
        )

        if name == 'blue':
            self.blue_centre_x = x + (w / 2)
            self.blue_area = area

        return True

    #Nav2 Exploration
    def send_nav_goal(self, x, y, yaw):
        if self.goal_active:
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 server not ready.')
            return

        self.get_logger().info(
            f'Sending random goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        self.goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            self.goal_active = False
            self.goal_handle = None
            return

        self.get_logger().info('Goal accepted.')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info('Goal finished.')
        self.goal_active = False
        self.goal_handle = None

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling goal because blue was found.')
            self.goal_handle.cancel_goal_async()

        self.goal_active = False
        self.goal_handle = None

    def explore(self):
        if self.finished or self.blue_seen or self.goal_active:
            return

        pose = self.random_safe_pose()

        if pose is None:
            return

        x, y, yaw = pose
        self.send_nav_goal(x, y, yaw)
    def approach_blue(self): #If the blue box is spotted then the robot ahs succeeded and therefore stops
        if self.finished:
            self.stop_robot()
            return

        if not self.blue_seen or self.blue_centre_x is None:
            self.stop_robot()
            return

        if self.goal_active:
            self.cancel_goal()

        image_centre = self.image_width / 2
        error = self.blue_centre_x - image_centre

        close_enough_area = 25000

        if self.blue_area > close_enough_area:
            self.get_logger().info('Blue box reached. Stopping.')
            self.finished = True
            self.stop_robot()
            return

        twist = Twist()

        if abs(error) > 50:
            twist.linear.x = 0.0
            twist.angular.z = -0.003 * error
        else:
            twist.linear.x = 0.15
            twist.angular.z = -0.002 * error

        twist.linear.x = max(min(twist.linear.x, 0.2), -0.2)
        twist.angular.z = max(min(twist.angular.z, 0.6), -0.6)

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def step(self):
        if self.finished:
            self.stop_robot()
        elif self.blue_seen:
            self.get_logger().info(
                f'Blue seen. area={self.blue_area:.1f}. Approaching.'
            )
            self.approach_blue()
        else:
            self.get_logger().info('Blue not visible. Exploring map.')
            self.explore()

def main(args=None):
    rclpy.init(args=args)

    node = ProjectNode()

    def signal_handler(sig, frame):
        node.stop_robot()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    thread.start()

    rate = node.create_rate(2)

    try:
        while rclpy.ok():
            node.step()
            rate.sleep()
    except ROSInterruptException:
        pass
    finally:
        node.stop_robot()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()