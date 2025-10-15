#!/usr/bin/env python3
"""
Standalone ROS2 node for publishing XRoboToolkit data.

This node publishes:
- Headset pose
- Left and right controller poses and states
- Motion tracker data (if available)
- Body tracking data (if available)
- Hand tracking states

Each component is modular and can be enabled/disabled independently.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, String, Bool
import xrobotoolkit_sdk as xrt
import sys


class XRoboPublisher(Node):
    """Main ROS2 node for publishing XRoboToolkit data."""

    def __init__(self):
        super().__init__('xrobo_publisher')

        # Declare parameters for enabling/disabling publishers
        self.declare_parameter('publish_headset', True)
        self.declare_parameter('publish_controllers', True)
        self.declare_parameter('publish_motion_trackers', True)
        self.declare_parameter('publish_body_tracking', True)
        self.declare_parameter('publish_hand_tracking', True)
        self.declare_parameter('publish_rate', 50.0)  # Hz

        # Get parameters
        self.publish_headset = self.get_parameter('publish_headset').value
        self.publish_controllers = self.get_parameter('publish_controllers').value
        self.publish_motion_trackers = self.get_parameter('publish_motion_trackers').value
        self.publish_body_tracking = self.get_parameter('publish_body_tracking').value
        self.publish_hand_tracking = self.get_parameter('publish_hand_tracking').value
        publish_rate = self.get_parameter('publish_rate').value

        # Initialize SDK
        try:
            self.get_logger().info('Initializing XRoboToolkit SDK...')
            xrt.init()
            self.get_logger().info('SDK initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize SDK: {e}')
            raise

        # Initialize publishers
        self._init_publishers()

        # Create timer for publishing at specified rate
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_data)

        self.get_logger().info(f'XRobo ROS2 Node started at {publish_rate} Hz')
        self.get_logger().info(f'  Headset: {self.publish_headset}')
        self.get_logger().info(f'  Controllers: {self.publish_controllers}')
        self.get_logger().info(f'  Motion Trackers: {self.publish_motion_trackers}')
        self.get_logger().info(f'  Body Tracking: {self.publish_body_tracking}')
        self.get_logger().info(f'  Hand Tracking: {self.publish_hand_tracking}')

    def _init_publishers(self):
        """Initialize all ROS2 publishers."""
        # Headset publishers
        if self.publish_headset:
            self.headset_pose_pub = self.create_publisher(PoseStamped, 'xrobo/headset/pose', 10)

        # Controller publishers
        if self.publish_controllers:
            self.left_controller_pose_pub = self.create_publisher(PoseStamped, 'xrobo/controller/left/pose', 10)
            self.right_controller_pose_pub = self.create_publisher(PoseStamped, 'xrobo/controller/right/pose', 10)
            self.left_controller_joy_pub = self.create_publisher(Joy, 'xrobo/controller/left/joy', 10)
            self.right_controller_joy_pub = self.create_publisher(Joy, 'xrobo/controller/right/joy', 10)

        # Motion tracker publishers
        if self.publish_motion_trackers:
            # Create dynamic publishers for each motion tracker
            # Will be created on first detection of trackers
            self.motion_tracker_pose_pubs = {}
            self.motion_tracker_twist_pubs = {}
            self.motion_tracker_accel_pubs = {}
            self.motion_tracker_serials_pub = self.create_publisher(String, 'xrobo/motion_trackers/serial_numbers', 10)

        # Body tracking publishers
        if self.publish_body_tracking:
            self.body_tracking_available_pub = self.create_publisher(Bool, 'xrobo/body/available', 10)
            self.body_poses_pub = self.create_publisher(String, 'xrobo/body/poses', 10)
            self.body_velocities_pub = self.create_publisher(String, 'xrobo/body/velocities', 10)
            self.body_accelerations_pub = self.create_publisher(String, 'xrobo/body/accelerations', 10)

        # Hand tracking publishers
        if self.publish_hand_tracking:
            self.left_hand_state_pub = self.create_publisher(Bool, 'xrobo/hand/left/active', 10)
            self.right_hand_state_pub = self.create_publisher(Bool, 'xrobo/hand/right/active', 10)

    def publish_data(self):
        """Main callback to publish all data."""
        try:
            timestamp = self.get_clock().now().to_msg()

            if self.publish_headset:
                self._publish_headset(timestamp)

            if self.publish_controllers:
                self._publish_controllers(timestamp)

            if self.publish_motion_trackers:
                self._publish_motion_trackers(timestamp)

            if self.publish_body_tracking:
                self._publish_body_tracking(timestamp)

            if self.publish_hand_tracking:
                self._publish_hand_tracking(timestamp)

        except Exception as e:
            self.get_logger().error(f'Error publishing data: {e}')

    def _publish_headset(self, timestamp):
        """Publish headset pose."""
        try:
            pose = xrt.get_headset_pose()
            if pose and len(pose) == 7:
                msg = PoseStamped()
                msg.header.stamp = timestamp
                msg.header.frame_id = 'xrobo_world'
                msg.pose.position.x = pose[0]
                msg.pose.position.y = pose[1]
                msg.pose.position.z = pose[2]
                msg.pose.orientation.x = pose[3]
                msg.pose.orientation.y = pose[4]
                msg.pose.orientation.z = pose[5]
                msg.pose.orientation.w = pose[6]
                self.headset_pose_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f'Error publishing headset: {e}')

    def _publish_controllers(self, timestamp):
        """Publish controller poses and button states."""
        try:
            # Left controller
            left_pose = xrt.get_left_controller_pose()
            if left_pose and len(left_pose) == 7:
                msg = PoseStamped()
                msg.header.stamp = timestamp
                msg.header.frame_id = 'xrobo_world'
                msg.pose.position.x = left_pose[0]
                msg.pose.position.y = left_pose[1]
                msg.pose.position.z = left_pose[2]
                msg.pose.orientation.x = left_pose[3]
                msg.pose.orientation.y = left_pose[4]
                msg.pose.orientation.z = left_pose[5]
                msg.pose.orientation.w = left_pose[6]
                self.left_controller_pose_pub.publish(msg)

            # Right controller
            right_pose = xrt.get_right_controller_pose()
            if right_pose and len(right_pose) == 7:
                msg = PoseStamped()
                msg.header.stamp = timestamp
                msg.header.frame_id = 'xrobo_world'
                msg.pose.position.x = right_pose[0]
                msg.pose.position.y = right_pose[1]
                msg.pose.position.z = right_pose[2]
                msg.pose.orientation.x = right_pose[3]
                msg.pose.orientation.y = right_pose[4]
                msg.pose.orientation.z = right_pose[5]
                msg.pose.orientation.w = right_pose[6]
                self.right_controller_pose_pub.publish(msg)

            # Left controller buttons and axes
            left_joy = Joy()
            left_joy.header.stamp = timestamp
            left_joy.header.frame_id = 'left_controller'
            left_axis = xrt.get_left_axis()
            left_joy.axes = [
                left_axis[0] if left_axis else 0.0,
                left_axis[1] if left_axis else 0.0,
                xrt.get_left_trigger(),
                xrt.get_left_grip()
            ]
            left_joy.buttons = [
                int(xrt.get_left_menu_button()),
                int(xrt.get_X_button()),
                int(xrt.get_Y_button()),
                int(xrt.get_left_axis_click())
            ]
            self.left_controller_joy_pub.publish(left_joy)

            # Right controller buttons and axes
            right_joy = Joy()
            right_joy.header.stamp = timestamp
            right_joy.header.frame_id = 'right_controller'
            right_axis = xrt.get_right_axis()
            right_joy.axes = [
                right_axis[0] if right_axis else 0.0,
                right_axis[1] if right_axis else 0.0,
                xrt.get_right_trigger(),
                xrt.get_right_grip()
            ]
            right_joy.buttons = [
                int(xrt.get_right_menu_button()),
                int(xrt.get_A_button()),
                int(xrt.get_B_button()),
                int(xrt.get_right_axis_click())
            ]
            self.right_controller_joy_pub.publish(right_joy)

        except Exception as e:
            self.get_logger().debug(f'Error publishing controllers: {e}')

    def _publish_motion_trackers(self, timestamp):
        """Publish motion tracker data as individual PoseStamped and TwistStamped messages."""
        try:
            num_trackers = xrt.num_motion_data_available()
            if num_trackers > 0:
                # Get all motion tracker data
                poses = xrt.get_motion_tracker_pose()
                velocities = xrt.get_motion_tracker_velocity()
                accelerations = xrt.get_motion_tracker_acceleration()
                serials = xrt.get_motion_tracker_serial_numbers()

                # Publish serial numbers
                msg = String()
                msg.data = str(serials)
                self.motion_tracker_serials_pub.publish(msg)

                # Publish individual tracker data
                for i in range(num_trackers):
                    # Create publishers dynamically if needed
                    tracker_id = f"tracker_{i}"
                    if serials and i < len(serials):
                        tracker_id = f"tracker_{serials[i]}"

                    # Create publishers for this tracker if they don't exist
                    if tracker_id not in self.motion_tracker_pose_pubs:
                        self.motion_tracker_pose_pubs[tracker_id] = self.create_publisher(
                            PoseStamped, f'xrobo/motion_trackers/{tracker_id}/pose', 10)
                        self.motion_tracker_twist_pubs[tracker_id] = self.create_publisher(
                            TwistStamped, f'xrobo/motion_trackers/{tracker_id}/twist', 10)
                        self.motion_tracker_accel_pubs[tracker_id] = self.create_publisher(
                            AccelStamped, f'xrobo/motion_trackers/{tracker_id}/accel', 10)
                        self.get_logger().info(f'Created publishers for motion tracker: {tracker_id}')

                    # Publish pose (x, y, z, qx, qy, qz, qw)
                    if poses and i < len(poses) and len(poses[i]) == 7:
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = timestamp
                        pose_msg.header.frame_id = 'xrobo_world'
                        pose_msg.pose.position.x = poses[i][0]
                        pose_msg.pose.position.y = poses[i][1]
                        pose_msg.pose.position.z = poses[i][2]
                        pose_msg.pose.orientation.x = poses[i][3]
                        pose_msg.pose.orientation.y = poses[i][4]
                        pose_msg.pose.orientation.z = poses[i][5]
                        pose_msg.pose.orientation.w = poses[i][6]
                        self.motion_tracker_pose_pubs[tracker_id].publish(pose_msg)

                    # Publish velocity (vx, vy, vz, wx, wy, wz)
                    if velocities and i < len(velocities) and len(velocities[i]) == 6:
                        twist_msg = TwistStamped()
                        twist_msg.header.stamp = timestamp
                        twist_msg.header.frame_id = 'xrobo_world'
                        twist_msg.twist.linear.x = velocities[i][0]
                        twist_msg.twist.linear.y = velocities[i][1]
                        twist_msg.twist.linear.z = velocities[i][2]
                        twist_msg.twist.angular.x = velocities[i][3]
                        twist_msg.twist.angular.y = velocities[i][4]
                        twist_msg.twist.angular.z = velocities[i][5]
                        self.motion_tracker_twist_pubs[tracker_id].publish(twist_msg)

                    # Publish acceleration (ax, ay, az, wax, way, waz)
                    if accelerations and i < len(accelerations) and len(accelerations[i]) == 6:
                        accel_msg = AccelStamped()
                        accel_msg.header.stamp = timestamp
                        accel_msg.header.frame_id = 'xrobo_world'
                        accel_msg.accel.linear.x = accelerations[i][0]
                        accel_msg.accel.linear.y = accelerations[i][1]
                        accel_msg.accel.linear.z = accelerations[i][2]
                        accel_msg.accel.angular.x = accelerations[i][3]
                        accel_msg.accel.angular.y = accelerations[i][4]
                        accel_msg.accel.angular.z = accelerations[i][5]
                        self.motion_tracker_accel_pubs[tracker_id].publish(accel_msg)

        except Exception as e:
            self.get_logger().debug(f'Error publishing motion trackers: {e}')

    def _publish_body_tracking(self, timestamp):
        """Publish body tracking data."""
        try:
            is_available = xrt.is_body_data_available()

            # Publish availability
            msg = Bool()
            msg.data = is_available
            self.body_tracking_available_pub.publish(msg)

            if is_available:
                # Publish body poses
                body_poses = xrt.get_body_joints_pose()
                msg = String()
                msg.data = str(body_poses)
                self.body_poses_pub.publish(msg)

                # Publish body velocities
                body_velocities = xrt.get_body_joints_velocity()
                msg = String()
                msg.data = str(body_velocities)
                self.body_velocities_pub.publish(msg)

                # Publish body accelerations
                body_accelerations = xrt.get_body_joints_acceleration()
                msg = String()
                msg.data = str(body_accelerations)
                self.body_accelerations_pub.publish(msg)

        except Exception as e:
            self.get_logger().debug(f'Error publishing body tracking: {e}')

    def _publish_hand_tracking(self, timestamp):
        """Publish hand tracking states."""
        try:
            # Left hand state
            left_hand_state = xrt.get_left_hand_tracking_state()
            msg = Bool()
            msg.data = bool(left_hand_state)
            self.left_hand_state_pub.publish(msg)

            # Right hand state
            right_hand_state = xrt.get_right_hand_tracking_state()
            msg = Bool()
            msg.data = bool(right_hand_state)
            self.right_hand_state_pub.publish(msg)

        except Exception as e:
            self.get_logger().debug(f'Error publishing hand tracking: {e}')

    def destroy_node(self):
        """Clean up when node is destroyed."""
        try:
            self.get_logger().info('Closing XRoboToolkit SDK...')
            xrt.close()
            self.get_logger().info('SDK closed successfully')
        except Exception as e:
            self.get_logger().error(f'Error closing SDK: {e}')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = XRoboPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
