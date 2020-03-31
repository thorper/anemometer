# Anemometer
This node will listen for an IMU message streaming from the Pixhawk. This message is the mavros/imu topic and it's streamed at 50 hz. Once this message is heard by ROS a trigger is sent to the sonic anemometer for a reading. This reading is sent back to the RPi. If the user wanted to save the sonic anemometer data alongside the Pixhawk data a bag file must be started to log all the data. First check connectivity of the sensors.

Anemometer must be installed and connected to RaspberryPi3. While logged into RPi check with the following commands:
```
cd /dev
sudo chmod 777 /dev/ttyUSB0
```
Pixhawk must be installed and connected to RaspberryPi3. While logged into RPi check with the following commands:
```
cd /dev
sudo chmod 777 /dev/ttyUSB1
```  
Run launch file from command line:
```
roslaunch anemometer sas.launch
```
Now start a bag file to log all data. While logged into RPi type the following commands:
```
cd bag_files
mkdir <test name>
cd <test name>
rosbag record -a
```
When testing is complete type:
```
ctrl + C
```
