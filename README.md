# 

This project provides a python interface to extract XR state using Pico robot sdk.

## Requirements

- CMake
- python3
- pybind11
- nlohmann/json
- pico roboticsservice

## Building the Project

```
python setup.py install
```

## Using the Python Bindings

## Examples

**1. Get Controller and Headset Poses**

```python
import pyroboticsservice as pico

pico.init()

left_pose = pico.get_left_controller_pose()
right_pose = pico.get_right_controller_pose()
headset_pose = pico.get_headset_pose()

print(f"Left Controller Pose: {left_pose}")
print(f"Right Controller Pose: {right_pose}")
print(f"Headset Pose: {headset_pose}")

pico.deinit()
```

**2. Get Controller Inputs (Triggers, Grips, Buttons, Axes)**

```python
import pyroboticsservice as pico

pico.init()

# Triggers and Grips
left_trigger = pico.get_left_trigger()
right_grip = pico.get_right_grip()
print(f"Left Trigger: {left_trigger}, Right Grip: {right_grip}")

# Buttons
a_button_pressed = pico.get_A_button()
x_button_pressed = pico.get_X_button()
print(f"A Button Pressed: {a_button_pressed}, X Button Pressed: {x_button_pressed}")

# Axes
left_axis = pico.get_left_axis()
right_axis_click = pico.get_right_axis_click()
print(f"Left Axis: {left_axis}, Right Axis Clicked: {right_axis_click}")

# Timestamp
timestamp = pico.get_time_stamp_ns()
print(f"Current Timestamp (ns): {timestamp}")

pico.deinit()
```