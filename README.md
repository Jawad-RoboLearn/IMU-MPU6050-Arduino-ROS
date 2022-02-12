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

'''
// I have used the Eigen Matrix library for this. The input to this function is a 4x1 quaternion vector in the order [q.w, q.x, q.y, q.z]

Eigen::Vector3d quat2euler(const Eigen::Vector4d &q)
{
  Eigen::Vector3d eulers;
  double x,y,z,w;
  x = q(1); y = q(2); z = q(3); w = q(0);
  double t0 = 2.0 * (w*x + y*z);
  double t1 = 1.0 - 2.0 * (x*x + y*y);
  double roll = atan2(t0, t1);
  eulers(0) = roll * (180/3.14159);

  double t2 = 2.0 * (w * y - z * x); 
  if (t2 > 1.0)
  {
    t2 = 1.0;
  }
  if (t2 < -1)
  {
    t2 = -1;
  }
  double pitch = asin(t2);
  eulers(1) = pitch * (180/3.14159);

  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (y * y + z * z);
  double yaw = atan2(t3, t4);
  eulers(2) = -(yaw * (180/3.14159) + 36.82); //here 36.82 is my offset from front and negative sign shows positive on clockwise

  return eulers;
}
'''
