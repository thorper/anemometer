# anemometer
i. Anemometer must be installed and connected to RaspberryPi3
	cd /dev
	sudo chmod 777 /dev/ttyUSB0
ii. Pixhawk must be installed and connected to RaspberryPi3
	cd /dev
	sudo chmod 777 /dev/ttyUSB1
  
Run launch file from command line:
	roslaunch anemometer sas.launch
