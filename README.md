# 

This project provides a python interface to extract XR state using Pico robot sdk.

## Requirements

- CMake
- python3
- pybind11
- nlohmann/json
- XRobotoolkit PC Service

## Building the Project

```
python setup.py install
```

## Using the Python Bindings

## Examples

**1. Get Controller and Headset Poses**

```python
import xrobotoolkit_sdk as xr

xr.init()

left_pose = xr.get_left_controller_pose()
right_pose = xr.get_right_controller_pose()
headset_pose = xr.get_headset_pose()

print(f"Left Controller Pose: {left_pose}")
print(f"Right Controller Pose: {right_pose}")
print(f"Headset Pose: {headset_pose}")

xr.close()
```

**2. Get Controller Inputs (Triggers, Grips, Buttons, Axes)**

```python
import xrobotoolkit_sdk as xr

xr.init()

# Triggers and Grips
left_trigger = xr.get_left_trigger()
right_grip = xr.get_right_grip()
print(f"Left Trigger: {left_trigger}, Right Grip: {right_grip}")

# Buttons
a_button_pressed = xr.get_A_button()
x_button_pressed = xr.get_X_button()
print(f"A Button Pressed: {a_button_pressed}, X Button Pressed: {x_button_pressed}")

# Axes
left_axis = xr.get_left_axis()
right_axis_click = xr.get_right_axis_click()
print(f"Left Axis: {left_axis}, Right Axis Clicked: {right_axis_click}")

# Timestamp
timestamp = xr.get_time_stamp_ns()
print(f"Current Timestamp (ns): {timestamp}")

xr.close()
```