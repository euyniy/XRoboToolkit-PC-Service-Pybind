#!/usr/bin/env python3
"""
Comprehensive XR Tracking Script for XRoboToolkit SDK

This script provides unified tracking capabilities for:
- Body tracking (24 joints with pose, velocity, acceleration)
- Head/HMD tracking (pose)
- Controller tracking (poses, buttons, triggers, axes)
- Motion tracker support (with configurable modes)

Usage:
    python comprehensive_tracker.py --prefix mydata --hz 60
    python comprehensive_tracker.py --no-body --no-head --controller --motion-tracker body
    python comprehensive_tracker.py --motion-tracker object --hz 30 --prefix experiment
"""

import xrobotoolkit_sdk as xrt
import argparse
import time
import os
import json
import csv
from datetime import datetime
from typing import Dict, List, Any, Optional


class XRTracker:
    def __init__(self, prefix: str = "xr_tracking", hz: float = 50.0,
                 body: bool = True, head: bool = True, controller: bool = False,
                 motion_tracker: Optional[str] = None):
        self.prefix = prefix
        self.hz = hz
        self.sample_interval = 1.0 / hz
        self.enable_body = body
        self.enable_head = head
        self.enable_controller = controller
        self.enable_motion_tracker = motion_tracker is not None
        self.motion_tracker_mode = motion_tracker

        # Data storage
        self.data_buffer = []
        self.start_time = None

        # Joint names for body tracking
        self.joint_names = [
            "Pelvis", "Left_Hip", "Right_Hip", "Spine1", "Left_Knee", "Right_Knee",
            "Spine2", "Left_Ankle", "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
            "Neck", "Left_Collar", "Right_Collar", "Head", "Left_Shoulder", "Right_Shoulder",
            "Left_Elbow", "Right_Elbow", "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand"
        ]

        # Initialize SDK
        xrt.init()
        self._wait_for_data()

    def _wait_for_data(self):
        """Wait for required data streams to become available"""
        print("Waiting for data streams to become available...")

        if self.enable_body:
            print("Waiting for body tracking data...")
            while not xrt.is_body_data_available():
                time.sleep(0.01)
            print("Body tracking data available!")

        if self.enable_motion_tracker:
            print("Waiting for motion tracker data...")
            while xrt.num_motion_data_available() == 0:
                time.sleep(0.01)
            print(f"Motion tracker data available! {xrt.num_motion_data_available()} trackers detected.")

        print("All required data streams are ready!")

    def _get_timestamp_filename(self, extension: str = "txt") -> str:
        """Generate filename with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{self.prefix}_{timestamp}.{extension}"

    def _collect_frame_data(self) -> Dict[str, Any]:
        """Collect data for a single frame"""
        frame_data = {
            "timestamp_ns": xrt.get_time_stamp_ns(),
            "frame_time": time.time() - self.start_time if self.start_time else 0.0
        }

        # Body tracking data
        if self.enable_body and xrt.is_body_data_available():
            body_poses = xrt.get_body_joints_pose()
            body_velocities = xrt.get_body_joints_velocity()
            body_accelerations = xrt.get_body_joints_acceleration()
            imu_timestamps = xrt.get_body_joints_timestamp()
            body_timestamp = xrt.get_body_timestamp_ns()

            frame_data["body"] = {
                "timestamp_ns": body_timestamp,
                "joints": {}
            }

            for i, joint_name in enumerate(self.joint_names):
                if i < len(body_poses):
                    frame_data["body"]["joints"][joint_name] = {
                        "pose": {
                            "position": [body_poses[i][0], body_poses[i][1], body_poses[i][2]],
                            "rotation": [body_poses[i][6], body_poses[i][3], body_poses[i][4], body_poses[i][5]]  # w,x,y,z
                        },
                        "velocity": body_velocities[i] if i < len(body_velocities) else [0]*6,
                        "acceleration": body_accelerations[i] if i < len(body_accelerations) else [0]*6,
                        "imu_timestamp": imu_timestamps[i] if i < len(imu_timestamps) else 0
                    }

        # Head/HMD tracking data
        if self.enable_head:
            headset_pose = xrt.get_headset_pose()
            frame_data["head"] = {
                "pose": {
                    "position": headset_pose[:3],
                    "rotation": headset_pose[3:]  # qx,qy,qz,qw
                }
            }

        # Controller tracking data
        if self.enable_controller:
            frame_data["controllers"] = {
                "left": {
                    "pose": {
                        "position": xrt.get_left_controller_pose()[:3],
                        "rotation": xrt.get_left_controller_pose()[3:]
                    },
                    "buttons": {
                        "menu": xrt.get_left_menu_button(),
                        "primary": xrt.get_X_button(),
                        "secondary": xrt.get_Y_button(),
                        "axis_click": xrt.get_left_axis_click()
                    },
                    "analog": {
                        "trigger": xrt.get_left_trigger(),
                        "grip": xrt.get_left_grip(),
                        "axis": xrt.get_left_axis()
                    }
                },
                "right": {
                    "pose": {
                        "position": xrt.get_right_controller_pose()[:3],
                        "rotation": xrt.get_right_controller_pose()[3:]
                    },
                    "buttons": {
                        "menu": xrt.get_right_menu_button(),
                        "primary": xrt.get_A_button(),
                        "secondary": xrt.get_B_button(),
                        "axis_click": xrt.get_right_axis_click()
                    },
                    "analog": {
                        "trigger": xrt.get_right_trigger(),
                        "grip": xrt.get_right_grip(),
                        "axis": xrt.get_right_axis()
                    }
                }
            }

        # Motion tracker data
        if self.enable_motion_tracker and xrt.num_motion_data_available() > 0:
            motion_poses = xrt.get_motion_tracker_pose()
            motion_velocities = xrt.get_motion_tracker_velocity()
            motion_accelerations = xrt.get_motion_tracker_acceleration()
            motion_serials = xrt.get_motion_tracker_serial_numbers()
            motion_timestamp = xrt.get_motion_timestamp_ns()

            frame_data["motion_trackers"] = {
                "mode": self.motion_tracker_mode,
                "timestamp_ns": motion_timestamp,
                "trackers": []
            }

            num_trackers = len(motion_poses) if motion_poses else 0
            for i in range(num_trackers):
                tracker_data = {
                    "id": i,
                    "serial": motion_serials[i] if i < len(motion_serials) else f"unknown_{i}",
                    "pose": {
                        "position": motion_poses[i][:3] if len(motion_poses[i]) >= 3 else [0,0,0],
                        "rotation": motion_poses[i][3:] if len(motion_poses[i]) >= 7 else [0,0,0,1]
                    },
                    "velocity": motion_velocities[i] if i < len(motion_velocities) else [0]*6,
                    "acceleration": motion_accelerations[i] if i < len(motion_accelerations) else [0]*6
                }
                frame_data["motion_trackers"]["trackers"].append(tracker_data)

        # Hand tracking data (always available when body tracking is enabled)
        if self.enable_body:
            frame_data["hands"] = {
                "left": {
                    "tracking_state": xrt.get_left_hand_tracking_state(),
                    "is_active": xrt.get_left_hand_isActive()
                },
                "right": {
                    "tracking_state": xrt.get_right_hand_tracking_state(),
                    "is_active": xrt.get_right_hand_isActive()
                }
            }

        return frame_data

    def _save_data_txt(self, filename: str):
        """Save collected data to text file"""
        with open(filename, 'w') as f:
            f.write(f"# XR Tracking Data - {datetime.now().isoformat()}\n")
            f.write(f"# Configuration: body={self.enable_body}, head={self.enable_head}, ")
            f.write(f"controller={self.enable_controller}, motion_tracker={self.motion_tracker_mode}\n")
            f.write(f"# Sample Rate: {self.hz} Hz\n")
            f.write(f"# Total Frames: {len(self.data_buffer)}\n\n")

            for i, frame in enumerate(self.data_buffer):
                f.write(f"Frame {i:06d}:\n")
                f.write(json.dumps(frame, indent=2))
                f.write("\n\n")

    def _save_data_json(self, filename: str):
        """Save collected data to JSON file"""
        output_data = {
            "metadata": {
                "timestamp": datetime.now().isoformat(),
                "configuration": {
                    "body": self.enable_body,
                    "head": self.enable_head,
                    "controller": self.enable_controller,
                    "motion_tracker": self.motion_tracker_mode
                },
                "sample_rate_hz": self.hz,
                "total_frames": len(self.data_buffer)
            },
            "frames": self.data_buffer
        }

        with open(filename, 'w') as f:
            json.dump(output_data, f, indent=2)

    def run_continuous(self):
        """Run continuous tracking and save data"""
        print(f"Starting continuous tracking at {self.hz} Hz...")
        print(f"Enabled streams: body={self.enable_body}, head={self.enable_head}, controller={self.enable_controller}")
        if self.enable_motion_tracker:
            print(f"Motion tracker mode: {self.motion_tracker_mode}")

        self.start_time = time.time()
        frame_count = 0

        try:
            while True:
                loop_start = time.time()

                # Collect frame data
                frame_data = self._collect_frame_data()
                self.data_buffer.append(frame_data)
                frame_count += 1

                # Print status every 100 frames
                if frame_count % 100 == 0:
                    elapsed = time.time() - self.start_time
                    actual_hz = frame_count / elapsed
                    print(f"Frame {frame_count:06d} | Elapsed: {elapsed:.1f}s | Rate: {actual_hz:.1f} Hz")

                # Sleep to maintain target frame rate
                loop_time = time.time() - loop_start
                sleep_time = max(0, self.sample_interval - loop_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print(f"\nStopping tracking... Collected {frame_count} frames")

            # Save data
            txt_filename = self._get_timestamp_filename("txt")
            json_filename = self._get_timestamp_filename("json")

            print(f"Saving data to {txt_filename} and {json_filename}...")
            self._save_data_txt(txt_filename)
            self._save_data_json(json_filename)
            print("Data saved successfully!")


def main():
    parser = argparse.ArgumentParser(description="Comprehensive XR Tracking Script")

    # Data streams
    parser.add_argument("--no-body", action="store_true", help="Disable body tracking (default: enabled)")
    parser.add_argument("--no-head", action="store_true", help="Disable head/HMD tracking (default: enabled)")
    parser.add_argument("--controller", action="store_true", help="Enable controller tracking (default: disabled)")
    parser.add_argument("--motion-tracker", choices=["body", "object", "generic"],
                       help="Enable motion tracker with specified mode")

    # Recording parameters
    parser.add_argument("--prefix", default="xr_tracking", help="Output file prefix (default: xr_tracking)")
    parser.add_argument("--hz", type=float, default=50.0, help="Sample rate in Hz (default: 50.0)")

    args = parser.parse_args()

    # Configure tracking based on arguments
    body_enabled = not args.no_body
    head_enabled = not args.no_head
    controller_enabled = args.controller
    motion_tracker_mode = args.motion_tracker

    print("=== XR Comprehensive Tracking Script ===")
    print(f"Configuration:")
    print(f"  Body tracking: {body_enabled}")
    print(f"  Head tracking: {head_enabled}")
    print(f"  Controller tracking: {controller_enabled}")
    print(f"  Motion tracker: {motion_tracker_mode or 'disabled'}")
    print(f"  Sample rate: {args.hz} Hz")
    print(f"  Output prefix: {args.prefix}")
    print()

    # Create and run tracker
    tracker = XRTracker(
        prefix=args.prefix,
        hz=args.hz,
        body=body_enabled,
        head=head_enabled,
        controller=controller_enabled,
        motion_tracker=motion_tracker_mode
    )

    tracker.run_continuous()


if __name__ == "__main__":
    main()