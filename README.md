# Attitude-Estimation-of-an-IMU-using-Complimentary-filter
The project aims to estimate the three dimensional orientation of the IMU by reading the acceleration and the gyroscope values provided by it. A complementary filter is implemented to fuse the values from the accelerometer and the gyroscope to obtain the orientation. The orientation is then compared with the ground truth from the vicon motion capture system.
The data from a six degree IMU is provided, 3-axis gyro- scope and 3-axis accelerometer. The orientation of the IMU recorded from the VICON motion capture is also provided. The data for each correspondence is provided in a .mat file.
The IMU data follows the following structure:
[a x a y a z w z w x w y ]

The VICON data meanwhile stores the timestamps ts and the orientation in a 3x3 Rotation matrix denoting Z-Y-X euler angles for N time instances. The parameter values for the IMU are also provided in a 2x3 vector containing the Scale and Bias values.
