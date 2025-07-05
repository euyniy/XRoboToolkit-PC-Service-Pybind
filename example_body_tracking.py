#!/usr/bin/env python3
"""
Example script demonstrating whole body motion tracking with XRoboToolkit SDK.

This script shows how to:
1. Check if body tracking data is available
2. Get body joint poses (position and rotation)
3. Get body joint velocities and accelerations
4. Get IMU timestamps for each joint
5. Get body data timestamp
6. Save data to CSV file (optional)

Body Joint Indices (24 joints total):
0: Pelvis, 1: Left Hip, 2: Right Hip, 3: Spine1, 4: Left Knee, 5: Right Knee,
6: Spine2, 7: Left Ankle, 8: Right Ankle, 9: Spine3, 10: Left Foot, 11: Right Foot,
12: Neck, 13: Left Collar, 14: Right Collar, 15: Head, 16: Left Shoulder, 17: Right Shoulder,
18: Left Elbow, 19: Right Elbow, 20: Left Wrist, 21: Right Wrist, 22: Left Hand, 23: Right Hand
"""

import xrobotoolkit_sdk as xrt
import time
import argparse
import csv
import os
from datetime import datetime

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='XRoboToolkit Body Tracking Demo')
    parser.add_argument('--save', action='store_true', 
                       help='Save tracking data to CSV file')
    parser.add_argument('--episode-length', type=float, default=10.0,
                       help='Episode length in seconds (default: 10.0)')
    parser.add_argument('--output-dir', type=str, default='./data',
                       help='Output directory for saved data (default: ./data)')
    parser.add_argument('--sample-rate', type=float, default=10.0,
                       help='Data sampling rate in Hz (default: 10.0)')
    
    args = parser.parse_args()
    
    print("Initializing XRoboToolkit SDK...")
    xrt.init()
    
    # Create output directory if saving data
    if args.save:
        os.makedirs(args.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(args.output_dir, f'body_tracking_{timestamp}.csv')
        print(f"Data will be saved to: {csv_filename}")
    
    try:
        print("Checking for body tracking data...")
        
        # Wait for body tracking data to be available
        timeout = 10  # 10 seconds timeout
        start_time = time.time()
        
        while not xrt.is_body_data_available():
            if time.time() - start_time > timeout:
                print("Warning: No body tracking data available after 10 seconds.")
                print("Make sure:")
                print("1. PICO headset is connected")
                print("2. Body tracking is enabled in the control panel")
                print("3. At least two Pico Swift devices are connected and calibrated")
                break
            time.sleep(0.1)
        
        if xrt.is_body_data_available():
            print("Body tracking data is available!")
            
            # Joint names for reference
            joint_names = [
                "Pelvis", "Left Hip", "Right Hip", "Spine1", "Left Knee", "Right Knee",
                "Spine2", "Left Ankle", "Right Ankle", "Spine3", "Left Foot", "Right Foot",
                "Neck", "Left Collar", "Right Collar", "Head", "Left Shoulder", "Right Shoulder",
                "Left Elbow", "Right Elbow", "Left Wrist", "Right Wrist", "Left Hand", "Right Hand"
            ]
            
            # Get initial body joint poses for display
            body_poses = xrt.get_body_joints_pose()
            print(f"Body joints pose data (24 joints):")
            
            for i, pose in enumerate(body_poses):
                x, y, z, qx, qy, qz, qw = pose
                print(f"  Joint {i:2d} ({joint_names[i]:<13}): "
                      f"Pos({x:6.3f}, {y:6.3f}, {z:6.3f}) "
                      f"Rot({qx:6.3f}, {qy:6.3f}, {qz:6.3f}, {qw:6.3f})")
            
            # Get body joint velocities
            body_velocities = xrt.get_body_joints_velocity()
            print(f"\nBody joints velocity data (first 3 joints):")
            for i in range(3):
                vx, vy, vz, wx, wy, wz = body_velocities[i]
                print(f"  Joint {i:2d} ({joint_names[i]:<13}): "
                      f"Vel({vx:6.3f}, {vy:6.3f}, {vz:6.3f}) "
                      f"AngVel({wx:6.3f}, {wy:6.3f}, {wz:6.3f})")
            
            # Get body joint accelerations
            body_accelerations = xrt.get_body_joints_acceleration()
            print(f"\nBody joints acceleration data (first 3 joints):")
            for i in range(3):
                ax, ay, az, wax, way, waz = body_accelerations[i]
                print(f"  Joint {i:2d} ({joint_names[i]:<13}): "
                      f"Acc({ax:6.3f}, {ay:6.3f}, {az:6.3f}) "
                      f"AngAcc({wax:6.3f}, {way:6.3f}, {waz:6.3f})")
            
            # Get IMU timestamps
            imu_timestamps = xrt.get_body_joints_timestamp()
            print(f"\nIMU timestamps (first 3 joints):")
            for i in range(3):
                print(f"  Joint {i:2d} ({joint_names[i]:<13}): {imu_timestamps[i]}")
            
            # Get body data timestamp
            body_timestamp = xrt.get_body_timestamp_ns()
            print(f"\nBody data timestamp: {body_timestamp} ns")
            
            # Prepare CSV file if saving
            csv_file = None
            csv_writer = None
            if args.save:
                csv_file = open(csv_filename, 'w', newline='')
                fieldnames = ['timestamp', 'body_timestamp_ns']
                
                # Add fieldnames for each joint (pose, velocity, acceleration, imu_timestamp)
                for i, joint_name in enumerate(joint_names):
                    fieldnames.extend([
                        f'joint_{i:02d}_{joint_name}_pos_x',
                        f'joint_{i:02d}_{joint_name}_pos_y', 
                        f'joint_{i:02d}_{joint_name}_pos_z',
                        f'joint_{i:02d}_{joint_name}_rot_qx',
                        f'joint_{i:02d}_{joint_name}_rot_qy',
                        f'joint_{i:02d}_{joint_name}_rot_qz',
                        f'joint_{i:02d}_{joint_name}_rot_qw',
                        f'joint_{i:02d}_{joint_name}_vel_x',
                        f'joint_{i:02d}_{joint_name}_vel_y',
                        f'joint_{i:02d}_{joint_name}_vel_z',
                        f'joint_{i:02d}_{joint_name}_ang_vel_x',
                        f'joint_{i:02d}_{joint_name}_ang_vel_y',
                        f'joint_{i:02d}_{joint_name}_ang_vel_z',
                        f'joint_{i:02d}_{joint_name}_acc_x',
                        f'joint_{i:02d}_{joint_name}_acc_y',
                        f'joint_{i:02d}_{joint_name}_acc_z',
                        f'joint_{i:02d}_{joint_name}_ang_acc_x',
                        f'joint_{i:02d}_{joint_name}_ang_acc_y',
                        f'joint_{i:02d}_{joint_name}_ang_acc_z',
                        f'joint_{i:02d}_{joint_name}_imu_timestamp'
                    ])
                
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                csv_writer.writeheader()
            
            print(f"\nStarting episode data collection...")
            print(f"Episode length: {args.episode_length} seconds")
            print(f"Sample rate: {args.sample_rate} Hz")
            if args.save:
                print(f"Saving data: {csv_filename}")
            print("Press Ctrl+C to stop early\n")
            
            try:
                episode_start_time = time.time()
                sample_interval = 1.0 / args.sample_rate
                next_sample_time = episode_start_time
                sample_count = 0
                
                while True:
                    current_time = time.time()
                    elapsed_time = current_time - episode_start_time
                    
                    # Check if episode is complete
                    if elapsed_time >= args.episode_length:
                        print(f"\nEpisode complete! Duration: {elapsed_time:.2f} seconds")
                        print(f"Total samples collected: {sample_count}")
                        break
                    
                    # Sample data at specified rate
                    if current_time >= next_sample_time and xrt.is_body_data_available():
                        # Get all body tracking data
                        body_poses = xrt.get_body_joints_pose()
                        body_velocities = xrt.get_body_joints_velocity()
                        body_accelerations = xrt.get_body_joints_acceleration()
                        imu_timestamps = xrt.get_body_joints_timestamp()
                        body_timestamp = xrt.get_body_timestamp_ns()
                        
                        # Display head and pelvis positions
                        head_joint_idx = 15  # Head joint
                        pelvis_joint_idx = 0  # Pelvis joint
                        
                        head_pose = body_poses[head_joint_idx]
                        x, y, z, qx, qy, qz, qw = head_pose
                        
                        pelvis_pose = body_poses[pelvis_joint_idx]
                        px, py, pz = pelvis_pose[0], pelvis_pose[1], pelvis_pose[2]
                        
                        print(f"\rSample {sample_count:4d} | "
                              f"Time: {elapsed_time:6.2f}s | "
                              f"Head: ({x:6.3f}, {y:6.3f}, {z:6.3f}) | "
                              f"Pelvis: ({px:6.3f}, {py:6.3f}, {pz:6.3f})", end="")
                        
                        # Save data to CSV if requested
                        if args.save and csv_writer:
                            row = {
                                'timestamp': current_time,
                                'body_timestamp_ns': body_timestamp
                            }
                            
                            for i, joint_name in enumerate(joint_names):
                                # Pose data
                                pose = body_poses[i]
                                row[f'joint_{i:02d}_{joint_name}_pos_x'] = pose[0]
                                row[f'joint_{i:02d}_{joint_name}_pos_y'] = pose[1]
                                row[f'joint_{i:02d}_{joint_name}_pos_z'] = pose[2]
                                row[f'joint_{i:02d}_{joint_name}_rot_qx'] = pose[3]
                                row[f'joint_{i:02d}_{joint_name}_rot_qy'] = pose[4]
                                row[f'joint_{i:02d}_{joint_name}_rot_qz'] = pose[5]
                                row[f'joint_{i:02d}_{joint_name}_rot_qw'] = pose[6]
                                
                                # Velocity data
                                vel = body_velocities[i]
                                row[f'joint_{i:02d}_{joint_name}_vel_x'] = vel[0]
                                row[f'joint_{i:02d}_{joint_name}_vel_y'] = vel[1]
                                row[f'joint_{i:02d}_{joint_name}_vel_z'] = vel[2]
                                row[f'joint_{i:02d}_{joint_name}_ang_vel_x'] = vel[3]
                                row[f'joint_{i:02d}_{joint_name}_ang_vel_y'] = vel[4]
                                row[f'joint_{i:02d}_{joint_name}_ang_vel_z'] = vel[5]
                                
                                # Acceleration data
                                acc = body_accelerations[i]
                                row[f'joint_{i:02d}_{joint_name}_acc_x'] = acc[0]
                                row[f'joint_{i:02d}_{joint_name}_acc_y'] = acc[1]
                                row[f'joint_{i:02d}_{joint_name}_acc_z'] = acc[2]
                                row[f'joint_{i:02d}_{joint_name}_ang_acc_x'] = acc[3]
                                row[f'joint_{i:02d}_{joint_name}_ang_acc_y'] = acc[4]
                                row[f'joint_{i:02d}_{joint_name}_ang_acc_z'] = acc[5]
                                
                                # IMU timestamp
                                row[f'joint_{i:02d}_{joint_name}_imu_timestamp'] = imu_timestamps[i]
                            
                            csv_writer.writerow(row)
                        
                        sample_count += 1
                        next_sample_time += sample_interval
                    
                    time.sleep(0.001)  # Small sleep to prevent busy waiting
                    
            except KeyboardInterrupt:
                elapsed_time = time.time() - episode_start_time
                print(f"\nEpisode stopped early! Duration: {elapsed_time:.2f} seconds")
                print(f"Total samples collected: {sample_count}")
            
            finally:
                if csv_file:
                    csv_file.close()
                    print(f"Data saved to: {csv_filename}")
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        print("\nClosing XRoboToolkit SDK...")
        xrt.close()

if __name__ == "__main__":
    main() 