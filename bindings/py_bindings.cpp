#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <thread>
#include <iostream>
#include <mutex>
#include <sstream>
#include <array>
#include <nlohmann/json.hpp>
#include "PXREARobotSDK.h"

using json = nlohmann::json;

std::array<double, 7> LeftControllerPose;
std::array<double, 7> RightControllerPose;
std::array<double, 7> HeadsetPose;

std::array<std::array<double, 7>, 26> LeftHandTrackingState;
double LeftHandScale = 1.0;
std::array<std::array<double, 7>, 26> RightHandTrackingState;
double RightHandScale = 1.0;

bool LeftMenuButton;
double LeftTrigger;
double LeftGrip;
std::array<double, 2> LeftAxis{0.0, 0.0};
bool LeftAxisClick;
bool LeftPrimaryButton;
bool LeftSecondaryButton;

bool RightMenuButton;
double RightTrigger;
double RightGrip;
std::array<double, 2> RightAxis{0.0, 0.0};
bool RightAxisClick;
bool RightPrimaryButton;
bool RightSecondaryButton;

int64_t TimeStampNs;

std::mutex leftMutex;
std::mutex rightMutex;
std::mutex headsetPoseMutex;
std::mutex timestampMutex;
std::mutex leftHandMutex;
std::mutex rightHandMutex;

std::array<double, 7> stringToPoseArray(const std::string& poseStr) {
    std::array<double, 7> result{0};
    std::stringstream ss(poseStr);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 7) {
        result[i++] = std::stod(value);
    }
    return result;
}

void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData)
{
    switch (type)
    {
    case PXREAServerConnect:
        std::cout << "server connect\n" << std::endl;
        break;
    case PXREAServerDisconnect:
        std::cout << "server disconnect\n" << std::endl;
        break;
    case PXREADeviceFind:
        std::cout << "device found\n" << (const char*)userData << std::endl;
        break;
    case PXREADeviceMissing:
        std::cout << "device missing\n" << (const char*)userData << std::endl;
        break;
    case PXREADeviceConnect:
        std::cout << "device connect\n" << (const char*)userData << status << std::endl;
        break;
    case PXREADeviceStateJson:
        auto& dsj = *((PXREADevStateJson*)userData);
        try {
            json data = json::parse(dsj.stateJson);
            if (data.contains("value")) {
                auto value = json::parse(data["value"].get<std::string>());
                if (value["Controller"].contains("left")) {
                    auto& left = value["Controller"]["left"];
                    {
                        std::lock_guard<std::mutex> lock(leftMutex);
                        LeftControllerPose = stringToPoseArray(left["pose"].get<std::string>());
                        LeftTrigger = left["trigger"].get<double>();
                        LeftGrip = left["grip"].get<double>();
                        LeftMenuButton = left["menuButton"].get<bool>();
                        LeftAxis[0] = left["axisX"].get<double>();
                        LeftAxis[1] = left["axisY"].get<double>();
                        LeftAxisClick = left["axisClick"].get<bool>();
                        LeftPrimaryButton = left["primaryButton"].get<bool>();
                        LeftSecondaryButton = left["secondaryButton"].get<bool>();
                    }
                }
                if (value["Controller"].contains("right")) {
                    auto& right = value["Controller"]["right"];
                    {
                        std::lock_guard<std::mutex> lock(rightMutex);
                        RightControllerPose = stringToPoseArray(right["pose"].get<std::string>());
                        RightTrigger = right["trigger"].get<double>();
                        RightGrip = right["grip"].get<double>();
                        RightMenuButton = right["menuButton"].get<bool>();
                        RightAxis[0] = right["axisX"].get<double>();
                        RightAxis[1] = right["axisY"].get<double>();
                        RightAxisClick = right["axisClick"].get<bool>();
                        RightPrimaryButton = right["primaryButton"].get<bool>();
                        RightSecondaryButton = right["secondaryButton"].get<bool>();
                    }
                }
                if (value.contains("Head")) {
                    auto& headset = value["Head"];
                    {
                        std::lock_guard<std::mutex> lock(headsetPoseMutex);
                        HeadsetPose = stringToPoseArray(headset["pose"].get<std::string>());
                    }
                }
                if (value.contains("timeStampNs")) {
                    std::lock_guard<std::mutex> lock(timestampMutex);
                    TimeStampNs = value["timeStampNs"].get<int64_t>();
                }
                if (value["Hand"].contains("leftHand")) {
                    auto& leftHand = value["Hand"]["leftHand"];
                    {
                        std::lock_guard<std::mutex> lock(leftHandMutex);
                        
                        LeftHandScale = leftHand["scale"].get<double>();
                        for (int i = 0; i < 26; i++) {
                            LeftHandTrackingState[i] = stringToPoseArray(leftHand["HandJointLocations"][i]["p"].get<std::string>());
                        }
                    }
                }
                if (value["Hand"].contains("rightHand")) {
                    auto& rightHand = value["Hand"]["rightHand"];
                    {
                        std::lock_guard<std::mutex> lock(rightHandMutex);
                        RightHandScale = rightHand["scale"].get<double>();
                        for (int i = 0; i < 26; i++) {
                            RightHandTrackingState[i] = stringToPoseArray(rightHand["HandJointLocations"][i]["p"].get<std::string>());
                        }
                    }
                }
            }
        } catch (const json::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
        }
            break;
    }
}

void init() {
    if (PXREAInit(NULL, OnPXREAClientCallback, PXREAFullMask) != 0) {
        throw std::runtime_error("PXREAInit failed");
    }
}

void deinit() {
    PXREADeinit();
}

std::array<double, 7> getLeftControllerPose() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftControllerPose;
}

std::array<double, 7> getRightControllerPose() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightControllerPose;
}

std::array<double, 7> getHeadsetPose() {
    std::lock_guard<std::mutex> lock(headsetPoseMutex);
    return HeadsetPose;
}

double getLeftTrigger() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftTrigger;
}

double getLeftGrip() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftGrip;
}

double getRightTrigger() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightTrigger;
}

double getRightGrip() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightGrip;
}

bool getLeftMenuButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftMenuButton;
}

bool getRightMenuButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightMenuButton;
}

bool getLeftAxisClick() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftAxisClick;
}

bool getRightAxisClick() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightAxisClick;
}

std::array<double, 2> getLeftAxis() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftAxis;
}


std::array<double, 2> getRightAxis() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightAxis;
}

bool getLeftPrimaryButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftPrimaryButton;
}

bool getRightPrimaryButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightPrimaryButton;
}

bool getLeftSecondaryButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftSecondaryButton;
}

bool getRightSecondaryButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightSecondaryButton;
}

int64_t getTimeStampNs() {
    std::lock_guard<std::mutex> lock(timestampMutex);
    return TimeStampNs;
}

std::array<std::array<double, 7>, 26> getLeftHandTrackingState() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandTrackingState;
}

int getLeftHandScale() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandScale;
}

std::array<std::array<double, 7>, 26> getRightHandTrackingState() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandTrackingState;
}

int getRightHandScale() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandScale;
}

PYBIND11_MODULE(xrobotoolkit_sdk, m) {
    m.def("init", &init, "Initialize the PXREARobot SDK.");
    m.def("close", &deinit, "Deinitialize the PXREARobot SDK.");
    m.def("get_left_controller_pose", &getLeftControllerPose, "Get the left controller pose.");
    m.def("get_right_controller_pose", &getRightControllerPose, "Get the right controller pose.");
    m.def("get_headset_pose", &getHeadsetPose, "Get the headset pose.");
    m.def("get_left_trigger", &getLeftTrigger, "Get the left trigger value.");
    m.def("get_left_grip", &getLeftGrip, "Get the left grip value.");
    m.def("get_right_trigger", &getRightTrigger, "Get the right trigger value.");
    m.def("get_right_grip", &getRightGrip, "Get the right grip value.");
    m.def("get_left_menu_button", &getLeftMenuButton, "Get the left menu button state.");
    m.def("get_right_menu_button", &getRightMenuButton, "Get the right menu button state.");
    m.def("get_left_axis_click", &getLeftAxisClick, "Get the left axis click state.");
    m.def("get_right_axis_click", &getRightAxisClick, "Get the right axis click state.");
    m.def("get_left_axis", &getLeftAxis, "Get the left axis values (x, y).");
    m.def("get_right_axis", &getRightAxis, "Get the right axis values (x, y).");
    m.def("get_X_button", &getLeftPrimaryButton, "Get the left primary button state.");
    m.def("get_A_button", &getRightPrimaryButton, "Get the right primary button state.");
    m.def("get_Y_button", &getLeftSecondaryButton, "Get the left secondary button state.");
    m.def("get_B_button", &getRightSecondaryButton, "Get the right secondary button state.");
    m.def("get_time_stamp_ns", &getTimeStampNs, "Get the timestamp in nanoseconds.");
    m.def("get_left_hand_tracking_state", &getLeftHandTrackingState, "Get the left hand state.");
    m.def("get_right_hand_tracking_state", &getRightHandTrackingState, "Get the right hand state.");
    m.doc() = "Python bindings for PXREARobot SDK using pybind11.";
}