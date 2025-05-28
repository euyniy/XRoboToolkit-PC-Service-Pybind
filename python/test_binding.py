import pyroboticsservice as pico
import time
import sys


def run_tests():
    print("Starting Python binding test...")

    try:
        print("Initializing SDK...")
        pico.init()
        print("SDK Initialized successfully.")

        print("\n--- Testing all functions for 10 iterations ---")
        for i in range(10):
            print(f"\n--- Iteration {i+1} ---")

            # Poses
            left_pose = pico.get_left_controller_pose()
            right_pose = pico.get_right_controller_pose()
            headset_pose = pico.get_headset_pose()
            print(f"Left Controller Pose: {left_pose}")
            print(f"Right Controller Pose: {right_pose}")
            print(f"Headset Pose: {headset_pose}")

            # Triggers
            left_trigger = pico.get_left_trigger()
            right_trigger = pico.get_right_trigger()
            print(f"Left Trigger: {left_trigger}")
            print(f"Right Trigger: {right_trigger}")

            # Grips
            left_grip = pico.get_left_grip()
            right_grip = pico.get_right_grip()
            print(f"Left Grip: {left_grip}")
            print(f"Right Grip: {right_grip}")

            # Menu Buttons
            left_menu = pico.get_left_menu_button()
            right_menu = pico.get_right_menu_button()
            print(f"Left Menu Button: {left_menu}")
            print(f"Right Menu Button: {right_menu}")

            # Axis Clicks
            left_axis_click = pico.get_left_axis_click()
            right_axis_click = pico.get_right_axis_click()
            print(f"Left Axis Click: {left_axis_click}")
            print(f"Right Axis Click: {right_axis_click}")

            # Axes
            left_axis = pico.get_left_axis()
            right_axis = pico.get_right_axis()
            print(f"Left Axis (X, Y): {left_axis}")
            print(f"Right Axis (X, Y): {right_axis}")

            # Primary Buttons (X, A)
            x_button = pico.get_X_button()  # Left Primary
            a_button = pico.get_A_button()  # Right Primary
            print(f"X Button (Left Primary): {x_button}")
            print(f"A Button (Right Primary): {a_button}")

            # Secondary Buttons (Y, B)
            y_button = pico.get_Y_button()  # Left Secondary
            b_button = pico.get_B_button()  # Right Secondary
            print(f"Y Button (Left Secondary): {y_button}")
            print(f"B Button (Right Secondary): {b_button}")

            # Timestamp
            timestamp = pico.get_time_stamp_ns()
            print(f"Timestamp (ns): {timestamp}")

            time.sleep(1)  # Wait for 1 second before the next iteration

        print("\nAll iterations complete.")

    except RuntimeError as e:
        print(f"Runtime Error: {e}", file=sys.stderr)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
    finally:
        print("\nDeinitializing SDK...")
        pico.deinit()
        print("SDK Deinitialized.")
        print("Test finished.")


if __name__ == "__main__":
    run_tests()
