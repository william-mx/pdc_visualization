#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_geometry_msgs
from ros2_numpy import np_to_pointcloud, np_to_point

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import PointCloud2

from pdc_visualization.utils import TFHandler

class PDCVisualizer(Node):
    def __init__(self):
        super().__init__('ultrasonic_pointcloud')

        self.max_distance = 1.5  # meters
        self.min_distance = 0.02  # meters
        self.fov = 60  # degrees field of view
        self.num_points = 20  # points per wave
        self.num_waves = 20  # waves per sensor
        self.base = "base_link"  # Target frame for transformations

        self.intervals = np.linspace(self.min_distance, self.max_distance, self.num_waves)
        self.angle_start = np.deg2rad(-self.fov / 2)  # Start angle in radians
        self.angle_end = np.deg2rad(self.fov / 2)  # End angle in radians
        self.thetas = np.linspace(self.angle_start, self.angle_end, self.num_points)

        # init to read transofmration ebtween frames
        self.tf_handler = TFHandler(self)

        # Mapping sensor indices to tf frames
        self.load_params()

        self.frames = list(self.index2frame.values())

        # Fetch transformations
        self.get_logger().info("Waiting for static transforms. Make sure the 'tf_static' topic is being published.")
        self.transformations = {frame: self.tf_handler.get_transform(frame, self.base) for frame in self.frames}

        # Create a QoS profile
        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1

        # Single Publisher for PointCloud2 message
        self.publisher = self.create_publisher(PointCloud2, "/pdc", qos_profile)

        # Subscriber for ultrasonic distances
        self.create_subscription(Int16MultiArray, '/uss_sensors', self.callback, qos_profile)

    def callback(self, data):
        all_points = []
        distances = data.data

        for i, distance in enumerate(distances):
            if distance <= 0:
                continue

            distance_meters = float(distance) / 100  # Convert cm to meters
            points = self.generate_points_at_waves(distance_meters)
            transformed_points = self.transform_points(points, self.index2frame[i])
            all_points.extend(transformed_points)


        pointcloud_msg = np_to_pointcloud(all_points, self.base)
        self.publisher.publish(pointcloud_msg)
        
    def generate_points_at_waves(self, measurement):
        distances = self.intervals[self.intervals < measurement]

        if len(distances) > 0:
            distances[-1] = measurement
        else:
            distances = [measurement]

        all_points = []
        for d in distances:
            x = d * np.cos(self.thetas)
            y = d * np.sin(self.thetas)
            z = np.full(self.num_points, d)
            points = np.column_stack((x, y, z))
            all_points.append(points)

        return np.vstack(all_points)

    def transform_points(self, points, from_frame):
        transformed_points = []
        for point in points:
            point_stamped = np_to_point(point)
            
            transformation = self.transformations.get(from_frame)
            if transformation is None:
                self.get_logger().error(f"No transformation available for frame {from_frame}")
                continue  # Skip this point
            
            # Transform point from ultrasonic sensor frame to base_link frame
            pt = tf2_geometry_msgs.do_transform_point(point_stamped, transformation) 
            transformed_points.append((pt.point.x, pt.point.y, pt.point.z))

        return transformed_points

    def load_params(self):
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('index2frame.0', 'USS_SRB'),
                ('index2frame.1', 'USS_SRF'),
                ('index2frame.2', 'USS_FR'),
                ('index2frame.3', 'USS_FC'),
                ('index2frame.4', 'USS_FL'),
                ('index2frame.5', 'USS_SLF'),
                ('index2frame.6', 'USS_SLB'),
                ('index2frame.7', 'USS_BL'),
                ('index2frame.8', 'USS_BC'),
                ('index2frame.9', 'USS_BR')
            ]
        )
        
        self.frame2index = {self.get_parameter(f'index2frame.{i}').get_parameter_value().string_value: int(i) for i in range(10)}
        self.index2frame = {v: k for k, v in self.frame2index.items()}

def main():
    rclpy.init()
    node = PDCVisualizer()

    try:
        node.get_logger().info("Node started. Press Ctrl+C to exit.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("KeyboardInterrupt received. Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


