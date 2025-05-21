import pyroboticsservice
import time

pyroboticsservice.init()

while True:
    time.sleep(1)
    print(pyroboticsservice.get_right_controller_pose())
