#!/usr/bin/env python3
# -*-coding:utf8-*-
# LEGACY. This node subscribes to /graspnet/grasps and publishes vendor-
# specific piper_msgs/PosCmd on /pos_cmd. The current piper_ctl driver
# no longer subscribes to /pos_cmd — it subscribes to /pos_command
# (geometry_msgs/Pose) and /joint_command (sensor_msgs/JointState).
# Running this node in the current pipeline has no effect on the arm.
# Kept only for historical reference; do not add to any launch.

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time
from graspnet_msgs.msg import GraspPose
from piper_msgs.msg import PosCmd
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import math


class GraspnetToPoscmdNode(Node):
    """ROS2 node that converts GraspNet grasp poses to robot position commands"""

    def __init__(self) -> None:
        super().__init__('graspnet_to_poscmd_node')
        
        # Declare parameters for time tolerance
        self.declare_parameter('time_tolerance', 0.1)  # 100ms tolerance
        self.declare_parameter('use_latest_transform', True)  # Use latest available transform
        
        self.time_tolerance = self.get_parameter('time_tolerance').get_parameter_value().double_value
        self.use_latest_transform = self.get_parameter('use_latest_transform').get_parameter_value().bool_value
        self.finished_grasp = False
        # Initialize TF2 buffer and listener for coordinate transformation
        # Set cache time to 10 seconds to have more transform history
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for position commands
        self.pos_cmd_pub = self.create_publisher(PosCmd, 'pos_cmd', 1)
        
        # Subscriber for grasp poses
        self.grasp_sub = self.create_subscription(
            GraspPose,
            '/graspnet/grasps',
            self.grasp_callback,
            1
        )
        
        self.get_logger().info('GraspNet to PosCmd node initialized')
        self.get_logger().info(f'Time tolerance: {self.time_tolerance} seconds')
        self.get_logger().info(f'Use latest transform: {self.use_latest_transform}')
        self.get_logger().info('Subscribing to: /graspnet/grasps')
        self.get_logger().info('Publishing to: /pos_cmd')

    def grasp_callback(self, msg: GraspPose):
        """Callback function for grasp pose messages
        
        Args:
            msg: GraspPose message containing target_pose and gripper_width
        """
        try:
            # Transform the pose to base_link frame
            if self.use_latest_transform:
                # Use the latest available transform (time=0) to avoid extrapolation errors
                # This is more robust when there are small time synchronization issues
                pose_stamped = msg.target_pose
                pose_stamped.header.stamp = rclpy.time.Time().to_msg()  # Use latest transform
                pose_in_base_link = self.tf_buffer.transform(
                    pose_stamped,
                    'base_link',
                    timeout=Duration(seconds=1.0)
                )
            else:
                # Use the original timestamp with tolerance
                try:
                    pose_in_base_link = self.tf_buffer.transform(
                        msg.target_pose,
                        'base_link',
                        timeout=Duration(seconds=1.0)
                    )
                except ExtrapolationException:
                    # If extrapolation fails, try with latest transform
                    self.get_logger().warn('Extrapolation error, using latest transform')
                    pose_stamped = msg.target_pose
                    pose_stamped.header.stamp = rclpy.time.Time().to_msg()
                    pose_in_base_link = self.tf_buffer.transform(
                        pose_stamped,
                        'base_link',
                        timeout=Duration(seconds=1.0)
                    )
            
            # Extract position (in meters)
            x = pose_in_base_link.pose.position.x
            y = pose_in_base_link.pose.position.y
            z = pose_in_base_link.pose.position.z
            
            # if z < 0.1:
            #     z += 0.1
            self.get_logger().info('not fix z height')
            # Extract orientation quaternion
            qx = pose_in_base_link.pose.orientation.x
            qy = pose_in_base_link.pose.orientation.y
            qz = pose_in_base_link.pose.orientation.z
            qw = pose_in_base_link.pose.orientation.w
            
            # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
            # Using the same conversion as in moveit_control_node.cpp
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            
            # Create PosCmd message
            pos_cmd = PosCmd() 
            pos_cmd.x = x
            pos_cmd.y = y
            pos_cmd.z = z
            pos_cmd.roll = roll
            pos_cmd.pitch = pitch
            pos_cmd.yaw = yaw
            
            # Convert gripper width (0-0.07m) to gripper angle
            # Based on piper_ctrl_single_node.py, gripper value is in radians or similar units
            # The gripper_width from GraspNet is the full width, so we use it directly
            pos_cmd.gripper = msg.gripper_width
            
            # Set default modes
            pos_cmd.mode1 = 0
            pos_cmd.mode2 = 0
            
            # Publish the command
            self.pos_cmd_pub.publish(pos_cmd)
            
            # Log the transformation
            self.get_logger().info('Received grasp pose and transformed to base_link:')
            self.get_logger().info(f'  Position: [{x:.4f}, {y:.4f}, {z:.4f}] m')
            self.get_logger().info(f'  Orientation (RPY): [{math.degrees(roll):.2f}, '
                                 f'{math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}] deg')
            self.get_logger().info(f'  Gripper width: {msg.gripper_width:.4f} m')
            self.get_logger().info('Published to /pos_cmd')
            
        except TransformException as ex:
            self.get_logger().error(f'Could not transform pose to base_link: {ex}')
        except Exception as e:
            self.get_logger().error(f'Error in grasp callback: {e}')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)
        
        Args:
            x, y, z, w: Quaternion components
            
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = GraspnetToPoscmdNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

