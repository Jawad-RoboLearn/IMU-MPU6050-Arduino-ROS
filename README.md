# IMU-MPU6050-Arduino-ROS
MPU6050 Inertial Measurement Unit working with Arduino and ROS: Solving Drifting in Euler Angles


You may read my small but useful blog on this repository at 
https://jawadrobotics.com/mpu6050-inertial-measurement-unit-working-with-arduino-and-ros-solving-drifting-in-euler-angles/

The code file at Arduino side can be found as IMU_ROS_ARDUINO.ino

Once you upload this file to your Arduino. Please run 
>> Terminal 1: roscore
>> Terminal 2: rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1 _baud:=57600
>> Terminal 3: rostopic echo /mpu6050 

The euler angles provided by DMP library are fine but I have observed that after running the code for sometime the random values are being added to the euler angles and the data is not useful. You will convert the quaternions coming on the rostopic to the euler angles by the code provided in 
>> "quat2euler" 
