# KABOAT2020
source code for KABOAT 2020 </br>
ros DISTRO: melodic </br>
OS: ubuntu 18.04LTS

Following ROS packages are recommended to be installed previously.

1. ydlidar_ros_driver: https://github.com/YDLIDAR/ydlidar_ros.git
2. hector_mapping from hector_slam: https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
3. nmea_navsat_driver: https://github.com/ros-drivers/nmea_navsat_driver.git
4. navigation: https://github.com/ros-planning/navigation.git 
5. joy: https://github.com/ros-drivers/joystick_drivers.git
 - for package dependency
6. tf2_sensor_msgs: https://github.com/ros/geometry2.git
7. move_base_msgs: https://github.com/ros-planning/navigation_msgs.git

Following linux packages are recommended to be installed.
1. chrony:
- 3.3 isolated networks in https://chrony.tuxfamily.org/manual.html
- https://docs.fedoraproject.org/en-US/Fedora/25/html/System_Administrators_Guide/sect-Setting_up_chrony_for_a_system_in_an_isolated_network.html
```sh
sudo apt install chrony
sudo systemctl start chronyd
sudo systemctl enable chronyd
sudo systemctl status chronyd
chronyc tracking
```
if you have error writing to /etc/chrony/chrony.conf
```sh
sudo chmod 777 /etc/chrony/chrony.conf
``2

2. 
sudo apt-get install libusb-dev

3.
sudo apt-get install libspnav-dev

4.
sudo apt-get install libbluetooth-dev

5.
sudo apt-get install libcwiid1 libcwiid-dev

Following python modules are recommended to be installed.
1. pigpio
```sh
$ wget https://github.com/joan2937/pigpio/archive/master.zip
$ unzip master.zip
$ cd pigpio-master
$ make
$ sudo make install
```
don't forget to start pigpio using $ sudo pigpiod

2. pyserial
```sh
pip install pyserial
```
try using pip3 instead if you get errors "pip command not found"

3. geopy
```sh
pip install geopy
```


