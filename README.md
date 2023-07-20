## Microstrain_3dmgx1_imu
A driver for IMUs compatible the microstrain 3DM-GX1

In this version, the node has been updated to have the possibility of reading all de data (or the data that the user chooses)
from the IMU.
It is possible to utilize the 3DM-GX1 which utilizes USB-RS232 converter for communication.

# Topics
The read sensor measures are published on different topic:
imu/raw_data: publishes the raw sensor measures of the acceleration and angular velocity of the IMU.
- imu/raw_data: publishes the raw sensor measures of the acceleration and angular velocity of the IMU.
- raw_mag: publishes the raw sensor measures of the magnetic field.
- imu/data: publishes the instantaneous sensor measures of acceleration, angular velocity, and orientation.
- mag: publishes the instantaneous sensor measures of the magnetic field
- imu/data_no_grab: publishes the instantaneous sensor measures of the acceleration without gravity component, angular velocity, and orientation.
- imu/stab_data: publishes the acceleration and angular velocity stabilised with the internal complementary filter of the IMU.
- stab_mag: publishes the magnetic field stabilised with the internal complementary filter of the IMU.
- orientation: publishes the instantaneous quaternion that represents the orientation of the ENU reference frame.
- stab_orientation: publishes the stab quaternion that represents the orientation of the ENU reference frame.
- euler: publishes the instantaneous euler angles (roll, pitch and yaw) in degrees
- stab_euler: publishes the stabilised euler angles (roll, pitch, and yaw) in degrees
    
    IMU DATA follows the REP 103 ROS Covection (at rest gravity is +g when +z is upwards). Angles are right handled.
    Linear acceleration (without gravity) follows ROS convection REP 105, positive when when direcction is positive.

# Services
There are seven services:
- add_offset: to update time offset with the IMU time.
- captureBias: to capture gyro and linear acceleration (with no gravity) bias. When the node starts, it does a calibration process.
- publish_tf: [publishTF: (true/false),stab_publisTF: (true/false)] activate or deactivate the publisher of the IMU TF WRT an ENU/NED reference [base_frame].
- poll: activate the poll mode and polls the imu
- continuous: activate continuous mode
- start_calibration: It starts the magnetometer calibration. The user has to rotate the IMU (or the robot) to allow the algorithm to get the maximum and minimum in each axis.
- stop_calibration [typeId:(0/1), typeName: ("3D"/"2D")]: it stops the magnetometer calibration. The user has to select the calibration type. 3D is used when the possibility of rotating the IMU in the three axes exists. 2D is used when it is impossible to rotate the IMU in X and Y axes. In this case, it is necessary to send the magnitude of the magnetic field in Z in the working zone (intermagnet.org). See magnitudeZ param. 

# Params
The node can be configured with different parameters that allow users to use the sensor as they need.
- time_offset: defines the time offset with respect to the sensor clock
- port: serial port to read IMU data. 
- frame_id: name of the local reference frame of the IMU for instantaneous measures.
- stab_frame_id: name of the local reference frame of the IMU for stabilized measures.
- base_frame: name of the frame WRT the IMU frames will be orientated.
- auto_capture_bias: if true, the imu captures the bias when the node starts.
- publish_tf: if true, the node publishes the IMU instantaneous TF with respect to the ENU/NED frame called [base_frame].
- publish_stab_tf: if true, the node publishes the IMU stabilized TF with respect to the ENU/NED frame called [base_frame].
- use_enu_frame: if true, the IMU data is published with respect ENU reference frame. The IMU printed frame does not correspond in this case. If false, the NED reference frame is used and corresponds with the IMU printed frame.
- magnitudeZ: magnitude Z of the magnetic field of the IMU working zone. This parameter only is used if the magnetometer calibration used is 2D type. (Can use intermagnet.org)  
- poll_mode: if true, starts the imu in poll mode. If false, starts the node in continuous mode.
- raw_sensor: if true, the node publishes in the topic .../imu/data_raw the accelerometer and angular velocity raw data, and in the topic .../raw_mag the raw magnetometer data. 
- stab_mag_accel_angrate: if true, the node publishes in the topic .../imu/stab_data the stabilized acceleration and angular velocity, and in the topic .../stab_mag the stabilized magnetometer data.
- instant_mag_accel_angrate: if true, the node publishes in the topic .../imu/data the instantaneous acceleration and angular velocity, and in the topic .../mag the instantaneous magnetometer data.
- stab_quat_orient: if true the node publishes in the topic .../stab_orientation the stabilized quaternion that represents the IMU orientation in the ENU/NED reference system
- instant_quat_orient: if true the node publishes in the topic .../stab_orientation the instantaneous quaternion that represents the IMU orientation in the ENU/NED reference system
- stab_orient_mag_accel_angrate: if true the node publishes in the topic .../imu/stab_data and .../stab_mag the IMU acceleration, angular velocity and magnetic field established with its internal complementary filter
- instant_orient_mag_accel_angrate: if true the node publishes in the topic .../imu/data and .../mag the instantaneous IMU acceleration, angular velocity, and magnetic field. This param works at the frecuency of 30 Hz. 
- stab_euler_angles: if true the node publish in the topic .../stab_euler the stabilized roll, pitch and yaw in degrees. 
- euler_angles: if true, the node publishes in the topic .../euler the instantaneous roll, pitch, and yaw in degrees.

*If the user activates params that generate the same data, the params that generate redundant data are deactivated.
*If only one of the data parameters is activated, the work rate is 100Hz (The node writes the EEPROM, needed to restart the IMU power the first time). If there are two or more parameters activated, the frequency is 50Hz. When the parameter instant_orient_mag_accel_angrate, alone or with other parameters, the work frequency is near 30Hz.

The file params.yaml contains different interesting matrices. The covariance matrices are calculated by obtaining data with the IMU immobile.
- tf_translation: translation vector of the IMU instantaneous reference frame with respect to the ENU/NED frame called [base_frame]
- stab_tf_translation: translation vector of the IMU stabilized reference frame with respect to the ENU/NED frame called [base_frame]
- orientation_cov: covariance matrix of the covariance of the instantaneous Euler angles obtained from de IMU
- stab_orientation_cov: covariance matrix of the covariance of the stabilized Euler angles obtained from de IMU
- ang_vel_cov: covariance matrix of the instantaneous angular velocity
- stab_ang_vel_cov: covariance matrix of the stabilized angular velocity
- lin_acc_cov: covariance matrix of the instantaneous linear acceleration
- stab_lin_acc_cov: covariance matrix of the stabilized linear acceleration
- lin_acc_cov_no_grab: covariance matrix of the instantaneous linear acceleration without the gravity component
- stab_lin_acc_cov_no_grab: covariance matrix of the instantaneous linear acceleration without the gravity component
- mag_cov: covariance matrix of the scaled magnetic field vector
- stab_mag_cov: covariance matrix of the stabilized magnetic field vector

To check the working rate of the IMU it is possible to use the ../diagnostics topic. 

To make a node test use: 

`rosrun self_test run_selftest /self_test`

or 

`rosservice call /self_test`

with the node runnig. 
