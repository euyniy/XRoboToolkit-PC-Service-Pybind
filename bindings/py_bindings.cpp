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
double LeftTrigger;
double LeftGrip;
double RightTrigger;
double RightGrip;

std::mutex leftPoseMutex;
std::mutex rightPoseMutex;
std::mutex headsetPoseMutex;
std::mutex coutMutex;

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
        std::cout << "device find " << (const char*)userData << std::endl;
        break;
    case PXREADeviceMissing:
        std::cout << "device missing " << (const char*)userData << std::endl;
        break;
    case PXREADeviceConnect:
        std::cout << "device connect " << (const char*)userData << status << std::endl;
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
                        std::lock_guard<std::mutex> lock(leftPoseMutex);
                        LeftControllerPose = stringToPoseArray(left["pose"].get<std::string>());
                        LeftTrigger = left["trigger"].get<double>();
                        LeftGrip = left["grip"].get<double>();
                    }
                }
                if (value["Controller"].contains("right")) {
                    auto& right = value["Controller"]["right"];
                    {
                        std::lock_guard<std::mutex> lock(rightPoseMutex);
                        RightControllerPose = stringToPoseArray(right["pose"].get<std::string>());
                        RightTrigger = right["trigger"].get<double>();
                        RightGrip = right["grip"].get<double>();
                    }
                }
                if (value.contains("Head")) {
                    auto& headset = value["Head"];
                    {
                        std::lock_guard<std::mutex> lock(headsetPoseMutex);
                        HeadsetPose = stringToPoseArray(headset["pose"].get<std::string>());
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

void run_main_loop() {
    if (PXREAInit(NULL, OnPXREAClientCallback, PXREAFullMask) != 0) {
        throw std::runtime_error("PXREAInit failed");
    }

    try {
        pybind11::gil_scoped_release release_gil;

        while (true) {
            {
                pybind11::gil_scoped_acquire acquire_gil;
                if (PyErr_CheckSignals() != 0) {
                    throw pybind11::error_already_set();
                }
            } // GIL is released here by acquire_gil's destructor

            std::cout << " main loop alive." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const pybind11::error_already_set &e) {
        std::cout << "Interrupt received or Python error, deinitializing SDK..." << std::endl;
        PXREADeinit(); // Ensure resources are cleaned up
        throw; 
    } catch (const std::exception &e) {
        // Catch any other C++ exceptions
        std::cerr << "C++ exception in main loop: " << e.what() << std::endl;
        PXREADeinit(); // Ensure resources are cleaned up
        throw;
    }
}

std::array<double, 7> getLeftControllerPose() {
    std::lock_guard<std::mutex> lock(leftPoseMutex);
    return LeftControllerPose;
}
std::array<double, 7> getRightControllerPose() {
    std::lock_guard<std::mutex> lock(rightPoseMutex);
    return RightControllerPose;
}
std::array<double, 7> getHeadsetPose() {
    std::lock_guard<std::mutex> lock(headsetPoseMutex);
    return HeadsetPose;
}
double getLeftTrigger() {
    std::lock_guard<std::mutex> lock(leftPoseMutex);
    return LeftTrigger;
}
double getLeftGrip() {
    std::lock_guard<std::mutex> lock(leftPoseMutex);
    return LeftGrip;
}
double getRightTrigger() {
    std::lock_guard<std::mutex> lock(rightPoseMutex);
    return RightTrigger;
}
double getRightGrip() {
    std::lock_guard<std::mutex> lock(rightPoseMutex);
    return RightGrip;
}

std::array<double, 7> getPoseByName(const std::string& poseName) {
    if (poseName == "left_controller") {
        return getLeftControllerPose();
    } else if (poseName == "right_controller") {
        return getRightControllerPose();
    } else if (poseName == "headset") {
        return getHeadsetPose();
    } else {
        throw std::invalid_argument("Invalid pose name");
    }
}

double getKeyValueByName(const std::string& keyName) {
    if (keyName == "left_trigger") {
        return getLeftTrigger();
    } else if (keyName == "left_grip") {
        return getLeftGrip();
    } else if (keyName == "right_trigger") {
        return getRightTrigger();
    } else if (keyName == "right_grip") {
        return getRightGrip();
    } else {
        std::cout << "Invalid key name: " << keyName << std::endl;
        throw std::invalid_argument("Invalid key name");
    }
}

PYBIND11_MODULE(pyroboticsservice, m) {
    m.def("run_main_loop", &run_main_loop, "A function that runs the main loop of the application.");
    m.def("init", &init, "Initialize the PXREARobot SDK.");
    m.def("deinit", &deinit, "Deinitialize the PXREARobot SDK.");
    m.def("get_left_controller_pose", &getLeftControllerPose, "Get the left controller pose.");
    m.def("get_right_controller_pose", &getRightControllerPose, "Get the right controller pose.");
    m.def("get_headset_pose", &getHeadsetPose, "Get the headset pose.");
    m.def("get_left_trigger", &getLeftTrigger, "Get the left trigger value.");
    m.def("get_left_grip", &getLeftGrip, "Get the left grip value.");
    m.def("get_right_trigger", &getRightTrigger, "Get the right trigger value.");
    m.def("get_right_grip", &getRightGrip, "Get the right grip value.");
    m.def("get_pose_by_name", &getPoseByName, "Get the pose by name (left_controller, right_controller, headset).");
    m.def("get_key_value_by_name", &getKeyValueByName, "Get the key value by name (left_trigger, left_grip, right_trigger, right_grip).");
}