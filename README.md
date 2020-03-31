# Anemometer
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
