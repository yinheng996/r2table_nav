import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from random import random
from math import sqrt, atan2, pow, pi

class RRTConnect(Node):
    def init(self):
        super().init('rrt_connect')
        self.start_point = Point()
        self.goal_point = Point()
        self.current_point = Point()
        self.map_width = 10.0
        self.map_height = 10.0
        self.tree1 = []
        self.tree2 = []
        self.max_iterations = 5000
        self.distance_threshold = 0.5
        self.step_size = 0.1
        self.laser_scan_data = []
        self.min_distance = 0.5
        self.speed = 0.2
        self.angle = 0.0
        self.pub_speed = self.create_publisher(Float32, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('RRT Connect node has been started')

    def run(self):
        self.start_point.x = -self.map_width/2
        self.start_point.y = -self.map_height/2
        self.goal_point.x = self.map_width/2
        self.goal_point.y = self.map_height/2
        self.tree1.append(self.start_point)
        self.tree2.append(self.goal_point)
        for i in range(self.max_iterations):
            if i % 2 == 0:
                q = self.generate_random_point()
                nearest_node = self.nearest_neighbor(self.tree1, q)
                new_node = self.steer(nearest_node, q)
                if self.check_collision(nearest_node, new_node):
                    self.tree1.append(new_node)
                    self.connect_trees(new_node, self.tree2)
                    if self.distance(new_node, self.goal_point) < self.distance_threshold:
                        path = self.get_path(new_node, self.tree2)
                        self.publish_speed(0.0)
                        return path
            else:
                q = self.generate_random_point()
                nearest_node = self.nearest_neighbor(self.tree2, q)
                new_node = self.steer(nearest_node, q)
                if self.check_collision(nearest_node, new_node):
                    self.tree2.append(new_node)
                    self.connect_trees(new_node, self.tree1)
                    if self.distance(new_node, self.start_point) < self.distance_threshold:
                        path = self.get_path(new_node, self.tree1)
                        self.publish_speed(0.0)
                        return path
        self.publish_speed(0.0)
        return None

    def generate_random_point(self):
        x = random() * self.map_width - self.map_width/2
        y = random() * self.map_height - self.map_height/2
        return Point(x=x, y=y)

    def nearest_neighbor(self, tree, q):
        min_dist = float('inf')
        nearest_node = None
        for node in tree:
            dist = self.distance(node, q)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def steer(self, from_node, to_node):
        dist = self.distance(from_node, to_node)
        if dist < self.step_size:
            return to_node
        else:
            ratio = self.step_size / dist
            x = from_node.x + (to_node.x - from_node.x) * ratio
            y = from_node.y + (to_node.y - from_node.y) * ratio
            return Point(x=x, y=y)
    
    def check_collision(self, from_node, to_node):
        collision = False
        for i in range(len(self.laser_scan_data)):
            angle = self.angle + self.laser_scan_data[i][1]
            distance = self.laser_scan_data[i][2]
            if distance < self.min_distance:
                collision = True
                break
        return not collision

    def connect_trees(self, new_node, tree):
        nearest_node = self.nearest_neighbor(tree, new_node)
        while True:
            next_node = self.steer(nearest_node, new_node)
            if self.check_collision(nearest_node, next_node):
                tree.append(next_node)
                nearest_node = next_node
                if self.distance(next_node, new_node) < self.distance_threshold:
                    break
            else:
                break

    def get_path(self, new_node, tree):
        path = []
        path.append(new_node)
        current_node = new_node
        while current_node != self.start_point:
            for node in tree:
                if node == current_node.parent:
                    path.append(node)
                    current_node = node
                    break
        path.reverse()
        return path

    def publish_speed(self, speed):
        msg = Float32()
        msg.data = speed
        self.pub_speed.publish(msg)

    def scan_callback(self, msg):
        self.laser_scan_data = []
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        for i in range(len(ranges)):
            if ranges[i] != float('inf') and not float('nan'):
                angle = angle_min + angle_increment * i
                distance = ranges[i]
                self.laser_scan_data.append((i, angle, distance))
        if len(self.laser_scan_data) > 0:
            self.angle = self.laser_scan_data[0][1]

    def distance(self, p1, p2):
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2))
    
    def run(self):
        while not self.rclpy.is_shutdown():
            if self.goal_point is not None:
                rand_point = self.get_random_point()
                if self.check_collision(self.start_point, rand_point):
                    new_node = self.Node(len(self.tree), rand_point)
                    nearest_node = self.nearest_neighbor(self.tree, new_node)
                    if self.distance(nearest_node, new_node) > self.step_size:
                        new_node = self.steer(nearest_node, new_node)
                    if self.check_collision(nearest_node, new_node):
                        self.connect_trees(new_node, self.tree)
                        if self.distance(new_node, self.goal_point) < self.distance_threshold:
                            goal_node = self.Node(len(self.tree), self.goal_point)
                            self.connect_trees(goal_node, self.tree)
                            path = self.get_path(goal_node, self.tree)
                            for node in path:
                                self.publish_speed(self.speed)
                                self.pub_point.publish(node.point)
                                self.rclpy.spin_once(self.node, timeout_sec=0.1)
                            self.goal_point = None
            self.rclpy.spin_once(self.node, timeout_sec=0.1)