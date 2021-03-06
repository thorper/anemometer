# Anemometer
This node will listen for an IMU message streaming from the Pixhawk. This message is the mavros/imu topic and it's streamed at 50 hz. Once this message is heard by ROS a trigger is sent to the sonic anemometer for a reading. This reading is sent back to the RPi. If the user wanted to save the sonic anemometer data alongside the Pixhawk data a bag file must be started to log all the data.

## If needed, check connectivity of the sensors.

Anemometer must be installed and connected to RaspberryPi3. While logged into RPi check with the following commands:
```
cd /dev
sudo chmod 777 /dev/ttyUSB0
```
Pixhawk must be installed and connected to RaspberryPi3. While logged into RPi check with the following commands:
```
cd /dev
sudo chmod 777 /dev/ttyS0
```  
## Start reading wind data
While logged into RPi run launch file from command line:
```
roslaunch anemometer sas.launch
```
## Now start a bag file to log pixhawk and wind data. 
While logged into RPi type the following commands:
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
Please read the sonic anemometer user manual for further settings
```
SATI_User_Manual.pdf
```
The 2018 Aviation conference paper this ROS-node is referenced can be found [here]

[here]: https://doi.org/10.2514/6.2018-4218

