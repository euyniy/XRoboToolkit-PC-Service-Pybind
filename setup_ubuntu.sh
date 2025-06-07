export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
mkdir -p tmp
cd tmp
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK 
bash build.sh
cd ../../../..
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
rm -rf tmp

# Build the project
python setup.py install