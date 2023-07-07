/*
 * Software License Agreement (BSD License)
 *
 *  Microstrain 3DM-Gx1 node
 *  Copyright (c) 2008-2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <math.h>
#include <iostream>
#include <signal.h>

#include <boost/format.hpp>

#include "microstrain_3dmgx1_imu/3dmgx1.h"

#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/Imu.h"
#include <sensor_msgs/MagneticField.h>
#include "std_srvs/Empty.h"

#include "microstrain_3dmgx1_imu/AddOffset.h"

#include "std_msgs/Bool.h"

#include "microstrain_3dmgx1_imu/EulerStamped.h"
#include "microstrain_3dmgx1_imu/PublishTF.h"
#include "microstrain_3dmgx1_imu/StopCalibrate.h"

using namespace std;

#define X 0 //! Index of the first element of array that corresponds to x component of a point
#define Y 1 //! Index of the first element of array that corresponds to y component of a point
#define Z 2 //! Index of the first element of array that corresponds to z component of a point

bool sigint_emited=false;

class ImuNode 
{
public:
  microstrain_3dmgx1_imu::IMU imu; //! Object that allows the comunication with the IMUs

  string port; //! Serial port in which the IMU is connected

  //  microstrain_3dmgx1_imu::IMU::cmd cmd;
  std::vector<microstrain_3dmgx1_imu::IMU::cmd> cmd_v; //! List of commands that are being reading from de IMU
  std::vector<microstrain_3dmgx1_imu::IMU::cmd> cmd_capture_bias; //! List of commands read to calibrate (calculate bias).


  self_test::TestRunner self_test_; //! Object to test the IMU
  diagnostic_updater::Updater diagnostic_; //! Object that allows the IMU diagnostic

  ros::NodeHandle private_node_handle_; //! NodeHandle

  ros::Publisher imu_data_pub_; //! Publisher of IMU stabilized data
  ros::Publisher imu_data_no_grab_pub_; //! Publisher of IMU stabilized data without the grabity component
  ros::Publisher imu_stab_data_no_grab_pub_; //! Publisher of IMU stabilized data without the grabity component
  ros::Publisher imu_stab_data_pub_; //! Publisher of IMU stabilized data with the sensor complementary filter
  ros::Publisher imu_data_raw_pub_; //! Publisher of IMU raw data
  ros::ServiceServer add_offset_serv_; //! Service to update the time offset
  ros::ServiceServer capture_bias_serv_; //! Service to capture bias of the IMU
  ros::ServiceServer start_calibrate_serv; //! Service to start magnetometer calibration
  ros::ServiceServer stop_calibrate_serv; //! Service to stop magnetometer calibration
  ros::ServiceServer publishTF_serv_; //! Service to activate or deactivate the publisher of TF
  ros::ServiceServer poll_serv; //! Service to change the node to poll mode
  ros::ServiceServer continuous_serv; //!Service to change the node to continuous mode
  ros::Publisher mag_pub; //! Publisher of magnetometer data
  ros::Publisher raw_mag_pub; //! Publisher of magnetometer data
  ros::Publisher orientation_pub; //! Publisher of the orientation data
  ros::Publisher stab_orientation_pub; //! Publisher of the orientation data
  ros::Publisher stab_mag_pub; //! Publisher of the magnetometer data stabilized with the IMU complementary filter
  ros::Publisher stab_euler_pub; //! //! Publisher of the stab euler angles
  ros::Publisher euler_pub; //! //! Publisher of the euler angles

  bool running; //! Flag that indicates if the IMU is running

  bool auto_capture_bias; //! Flag that indicates if the IMU auto capture bias mode is activated
  bool capture_bias_requested_; //! Flag that indicates that it is necessary to capture the bias of the IMU
  bool biasCaptured_; //! Flag that indicates that the IMU is bias captured or not
  bool capturingBias=false; //! Flag that indicated that the IMU is capturing the bias at this moment
  bool publishTF; //! Flag that indicated if the TF publisher is activated or not
  bool stab_publishTF; //! Flag that indicated if the TF publisher is activated or not

  tf2_ros::TransformBroadcaster br; //! Reference system broadcaster
  tf2_ros::TransformBroadcaster stab_br; //! Reference system broadcaster

  int error_count_; //! Counter of IMU errors
  std::string error_status_; //! Message of the error status

  string frameid_; //! Id of the local frame of the IMU
  string stab_frameid_; //! Id of the local frame of the IMU

  string base_frame; //! Id of the base frame of the IMU
  
  double offset_; //! Time offset

  double max_drift_rate_; //! Reference Drift Rate to check the quality of the bias captured

  double desired_freq_; //! Working frecuency of the IMU to pass to self_test
  std::unique_ptr<diagnostic_updater::FrequencyStatus> freq_diag_; //! Self_test object to monitoring the IMU diagnosis

  std::vector<double> orientation_cov; //! Covariance of the orientation
  std::vector<double> stab_orientation_cov; //! Covariance of the orientation
  std::vector<double> lin_acc_cov; //! Covariance of the linear acceleration
  std::vector<double> ang_vel_cov; //! Covariance of the angular velocity
  std::vector<double> lin_acc_cov_no_grab; //! Covariance of the linear acceleration without gravity acceleration
  std::vector<double> stab_lin_acc_cov_no_grab; //! Covariance of the linear acceleration without gravity acceleration
  std::vector<double> mag_cov; //! Covariance of the magnetic field vector
  std::vector<double> stab_ang_vel_cov; //! Covariance of the angular velocity stabilized with the IMU complementary filter
  std::vector<double> stab_lin_acc_cov; //! Covariance of the linear acceleration stabilized with the IMU complementary filter
  std::vector<double> stab_mag_cov; //! Covariance of the magnetic field vector stabized with the IMU complementary filter
  std::vector<double> tf_translation; //! Traslation vector of the instantaneous TF
  std::vector<double> stab_tf_translation; //! Traslation vector of the stabilized TF

  bool raw_sensor; //! Flag that indicates that the raw data is read from de IMU
  bool stab_mag_accel_angrate; //! Flag that indicates that the stab magnetic field acceleration and angular velocity is read from the IMU
  bool instant_mag_accel_angrate; //! Flag that indicates that the instant magnetic field acceleration and angular velocity is read from the IMU
  bool stab_quat_orient; //! Flag that indicates that the stab quaternion is read from the IMU
  bool instant_quat_orient; //! Flag that indicates that the instant quaternion is read from the IMU
  bool stab_orient_mag_accel_angrate; //! Flag that indicates that the stab orientation magnetic field acceleration and angular velocity is read from the IMU
  bool stab_euler_angles; //! Flag that indicates that the stab euler angles is read from the IMU
  bool instant_euler_angles; //! Flag that indicates that the stab euler angles is read from the IMU
  bool instant_orient_mag_accel_angrate; //! Flag that indicates that the stab orientation and instant magnetic field acceleration and angular velocity is read from the IMU

  double linear_accel_bias[3]={0.0,0.0,0.0}; //! Biass of the components of the linear acceleration
  double angrate_bias[3]={0.0,0.0,0.0}; //! Biass of the components of the ang_rate
  double stab_linear_accel_bias[3]={0.0,0.0,0.0}; //! Biass of the components of the linear acceleration
  double stab_angrate_bias[3]={0.0,0.0,0.0}; //! Biass of the components of the ang_rate
  double magOffset[3]={0.0,0.0,0.0}; //! Offset of the bias calibration


  bool use_enu_frame; //! Flag that publicates the data in ENU frame

  tf2::Quaternion q_rot; //!Quaternion to rotate to ENU frame

  ros::WallTimer wallTimer; //! Walltimer to read the IMU contiunosly
  ros::WallTimer calibrateTimer; //! Timer to capture calibration data

  bool calibrationStarted = false;
  bool calibrated = false;

  bool poll_mode; //! Poll mode activate
  int magnitudeZ; //! Magnitude Z in miligauss in the working zone. Only used in 2D calibration mode

  bool slow=false; //! Indicates all instant values are read, so the frecuenzy is 30 hz

  /*!
   * Constructor of the IMU Node. The constructor process the parameters, initualizes the IMU, the
   * test object and the diagnostic object.
   *
   * \brief ImuNode Inializes the node and the imu reader object
   */
  ImuNode() : self_test_(), diagnostic_(),
    private_node_handle_("~"), capture_bias_requested_(false),
    error_count_(0)
  {
    microstrain_3dmgx1_imu::IMU::cmd cmd_tmp = microstrain_3dmgx1_imu::IMU::NONE;
    microstrain_3dmgx1_imu::IMU::cmd cmd_tmp2 = microstrain_3dmgx1_imu::IMU::NONE;

    if(!private_node_handle_.getParam("use_enu_frame", use_enu_frame))
      use_enu_frame = false;

    if(!private_node_handle_.getParam("auto_capture_bias", auto_capture_bias))
      auto_capture_bias = true;

    if(!private_node_handle_.getParam("port", port))
      port = "/dev/ttyUSB0";

    if(!private_node_handle_.getParam("max_drift_rate", max_drift_rate_))
      max_drift_rate_ = 0.0025;

    if(!private_node_handle_.getParam("time_offset", offset_))
      offset_ = 0.0;

    if(!private_node_handle_.getParam("frame_id", frameid_))
      frameid_ = "imu";

    if(!private_node_handle_.getParam("stab_frame_id", stab_frameid_))
      stab_frameid_ = "stab_imu";

    if(!private_node_handle_.getParam("base_frame", base_frame))
      base_frame = "base";

    if(!private_node_handle_.getParam("publish_tf",publishTF))
      publishTF = true;

    if(!private_node_handle_.getParam("publish_stab_tf",stab_publishTF))
      stab_publishTF = true;

    if(!private_node_handle_.getParam("magnitudeZ",magnitudeZ))
      magnitudeZ = 333;

    if(!private_node_handle_.getParam("poll_mode", poll_mode))
      poll_mode = false;

    if(!private_node_handle_.getParam("raw_sensor", raw_sensor))
      raw_sensor = false;

    if(raw_sensor) {
      cmd_v.push_back(microstrain_3dmgx1_imu::IMU::CMD_RAW_SENSOR);
      imu_data_raw_pub_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu/raw_data", 1);
      raw_mag_pub = private_node_handle_.advertise<sensor_msgs::MagneticField>("raw_mag", 1);
    }

    if(!private_node_handle_.getParam("stab_mag_accel_angrate", stab_mag_accel_angrate))
      stab_mag_accel_angrate = false;

    if(stab_mag_accel_angrate) {
      cmd_tmp = microstrain_3dmgx1_imu::IMU::CMD_GYRO_VECTOR;
      imu_stab_data_pub_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu/stab_data", 1);
      stab_mag_pub = private_node_handle_.advertise<sensor_msgs::MagneticField>("stab_mag", 1);
    }

    if(!private_node_handle_.getParam("instant_mag_accel_angrate", instant_mag_accel_angrate))
      instant_mag_accel_angrate = false;

    if(instant_mag_accel_angrate) {
      cmd_tmp2 = microstrain_3dmgx1_imu::IMU::CMD_INSTANT_VECTOR;
      imu_data_pub_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu/data", 1);
      mag_pub = private_node_handle_.advertise<sensor_msgs::MagneticField>("mag", 1);
    }

    if(!private_node_handle_.getParam("stab_quat_orient", stab_quat_orient))
      stab_quat_orient = false;

    if(stab_quat_orient) {
      cmd_v.push_back(microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT);
      stab_orientation_pub = private_node_handle_.advertise<geometry_msgs::QuaternionStamped>("stab_orientation", 1);
    }

    if(!private_node_handle_.getParam("instant_quat_orient", instant_quat_orient))
      instant_quat_orient = false;

    if(instant_quat_orient) {
      cmd_v.push_back(microstrain_3dmgx1_imu::IMU::CMD_INSTANT_QUAT);
      orientation_pub = private_node_handle_.advertise<geometry_msgs::QuaternionStamped>("orientation", 1);
    }

    if(!private_node_handle_.getParam("stab_orient_mag_accel_angrate", stab_orient_mag_accel_angrate))
      stab_orient_mag_accel_angrate = false;

    if(stab_orient_mag_accel_angrate) {
      cmd_tmp = microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_VECTOR;
      stab_mag_pub = private_node_handle_.advertise<sensor_msgs::MagneticField>("stab_mag", 1);
      imu_stab_data_pub_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu/stab_data", 1);
      imu_stab_data_no_grab_pub_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu/stab_data_no_grab", 1);
    }

    if(!private_node_handle_.getParam("instant_orient_mag_accel_angrate", instant_orient_mag_accel_angrate))
      instant_orient_mag_accel_angrate = true;

    if(instant_orient_mag_accel_angrate) {
      cmd_tmp2 = microstrain_3dmgx1_imu::IMU::CMD_INSTANT_GYRO_QUAT_VECTOR;
      slow=true;
      mag_pub = private_node_handle_.advertise<sensor_msgs::MagneticField>("mag", 1);
      imu_data_pub_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu/data", 1);
      imu_data_no_grab_pub_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu/data_no_grab", 1);
    }

    if(!private_node_handle_.getParam("stab_euler_angles", stab_euler_angles))
      stab_euler_angles = false;

    if(stab_euler_angles) {
      cmd_v.push_back(microstrain_3dmgx1_imu::IMU::CMD_GYRO_EULER); //
      stab_euler_pub = private_node_handle_.advertise<microstrain_3dmgx1_imu::EulerStamped>("stab_euler", 1);
    }

    if(!private_node_handle_.getParam("instant_euler_angles", instant_euler_angles))
      instant_euler_angles = false;
    if(instant_euler_angles) {
      cmd_v.push_back(microstrain_3dmgx1_imu::IMU::CMD_INSTANT_EULER); //
      euler_pub = private_node_handle_.advertise<microstrain_3dmgx1_imu::EulerStamped>("euler", 1);
    }

    if(!private_node_handle_.getParam("tf_translation",tf_translation))
      tf_translation =  {2.0, 2.0, 0.0};

    if(!private_node_handle_.getParam("stab_tf_translation",stab_tf_translation))
      tf_translation =  {2.0, 1.0, 0.0};

    if(!private_node_handle_.getParam("orientation_cov",orientation_cov))
      orientation_cov =  {0.0020,  0.0,    0.0,
                          0.0,    1.4908e-6,  0.0,
                          0.0,    0.0,    8.4248e-4};

    if(!private_node_handle_.getParam("stab_orientation_cov",stab_orientation_cov))
      stab_orientation_cov =  {0.0020,  0.0,    0.0,
                          0.0,    1.4908e-6,  0.0,
                          0.0,    0.0,    8.4248e-4};

    if(!private_node_handle_.getParam("ang_vel_cov",ang_vel_cov))
      ang_vel_cov =  {1.01850738002291e-05, 0.0,    0.0,
                      0.0,    8.98952077110984e-06, 0.0,
                      0.0,    0.0,    1.64683313442398e-05};

    if(!private_node_handle_.getParam("lin_acc_cov",lin_acc_cov))
      lin_acc_cov =  {7.25143532155076e-05, 0.0,    0.0,
                      0.0,    6.21782151391795e-05, 0.0,
                      0.0,    0.0,    3.18621874795061e-05};

    if(!private_node_handle_.getParam("lin_acc_cov_no_grab",lin_acc_cov_no_grab))
      lin_acc_cov_no_grab =  {0.000210987265734, 0.0,    0.0,
                              0.0,    0.000234988106838, 0.0,
                              0.0,    0.0,    3.1886763565377e-05};

    if(!private_node_handle_.getParam("stab_lin_acc_cov_no_grab",stab_lin_acc_cov_no_grab))
      stab_lin_acc_cov_no_grab =  {0.000210987265734, 0.0,    0.0,
                              0.0,    0.000234988106838, 0.0,
                              0.0,    0.0,    3.1886763565377e-05};

    if(!private_node_handle_.getParam("mag_cov",mag_cov))
      mag_cov =  {1.16666432794325e-05, 0.0,    0.0,
                  0.0, 5.1490058533241e-06, 0.0,
                  0.0,    0.0,    8.89686548995742e-06};

    if(!private_node_handle_.getParam("stab_ang_vel_cov",stab_ang_vel_cov))
      stab_ang_vel_cov =  {1.05986466329007e-05, 0.0,    0.0,
                           0.0,    8.99776473732564e-06, 0.0,
                           0.0,    0.0,    1.40389011593441e-05};

    if(!private_node_handle_.getParam("stab_lin_acc_cov",stab_lin_acc_cov))
      stab_lin_acc_cov =  {0.00013126810827, 0.0,    0.0,
                           0.0,    0.000167757339186, 0.0,
                           0.0,    0.0,    1.81007369002871e-07};

    if(!private_node_handle_.getParam("stab_mag_cov",stab_mag_cov))
      stab_mag_cov =  {1.2198e-5, 0.0,    0.0,
                       0.0, 4.1285e-6, 0.0,
                       0.0,    0.0,    1.1703e-5};


    if(cmd_tmp != microstrain_3dmgx1_imu::IMU::NONE)
      cmd_v.push_back(cmd_tmp);

    if(cmd_tmp2 != microstrain_3dmgx1_imu::IMU::NONE) {
      cmd_v.push_back(cmd_tmp2);
    }

    if(slow)
      desired_freq_ = 34.0;
    else if(cmd_v.size()>1)
      desired_freq_ = 50.0;
    else
      desired_freq_ = 100.0;

    freq_diag_ = std::make_unique<diagnostic_updater::FrequencyStatus>(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.15));

    add_offset_serv_ = private_node_handle_.advertiseService("add_offset", &ImuNode::addOffset, this);
    capture_bias_serv_ = private_node_handle_.advertiseService("captureBias", &ImuNode::calculateBias, this);
    publishTF_serv_ = private_node_handle_.advertiseService("publish_tf", &ImuNode::pubishTFCallback, this);
    poll_serv = private_node_handle_.advertiseService("poll",&ImuNode::poll,this);
    continuous_serv = private_node_handle_.advertiseService("continous",&ImuNode::continuous,this);
    start_calibrate_serv = private_node_handle_.advertiseService("start_calibrate", &ImuNode::start_calibrate, this);
    stop_calibrate_serv = private_node_handle_.advertiseService("stop_calibrate", &ImuNode::stop_calibrate, this);


    cmd_capture_bias.push_back(microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_VECTOR);
    cmd_capture_bias.push_back(microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_INSTANT_VECTOR);

    running = false;

    q_rot.setRPY(M_PI,0,M_PI/2); //ENU
    q_rot.normalize();

    self_test_.add("Interruption Test", this, &ImuNode::interruptionTest);
    self_test_.add("Connect Test", this, &ImuNode::connectTest);
    self_test_.add("Read ID Test", this, &ImuNode::readIDTest);
    self_test_.add("Gyro Bias Test", this, &ImuNode::biasTest);
    self_test_.add("Streamed Data Test", this, &ImuNode::streamedDataTest);
    self_test_.add("Gravity Test", this, &ImuNode::gravityTest);
    self_test_.add("Temperature Test",this,&ImuNode::temperatureTest);
    self_test_.add("Disconnect Test", this, &ImuNode::disconnectTest);
    self_test_.add("Resume Test", this, &ImuNode::resumeTest);

    diagnostic_.add( *(freq_diag_.get()));
    diagnostic_.add( "Bias Status", this, &ImuNode::biasStatus);
    diagnostic_.add( "IMU Status", this, &ImuNode::deviceStatus);
    diagnostic_.add( "Calibration Status", this, &ImuNode::calibrationStatus);

    wallTimer = private_node_handle_.createWallTimer(ros::WallDuration(0),&ImuNode::wallTimerCallback,this,false,false);
    calibrateTimer = private_node_handle_.createWallTimer(ros::WallDuration(0),&::ImuNode::calibrateTimerCallback,this,false,false);

    start();
  }

  /**
   * @brief Callback that reads the data stream from de IMU sensor
   *
   */
  void wallTimerCallback(const ros::WallTimerEvent& event) {

    uint8_t cmd[2];


    if(cmd_v.size() == 1 && !poll_mode && cmd_v[0] != microstrain_3dmgx1_imu::IMU::CMD_INSTANT_GYRO_QUAT_VECTOR) {
      getData(cmd_v[0],NULL,NULL);
    }
    else {
      for(int i=0;i<cmd_v.size();i++) {
        cmd[0] = static_cast<uint8_t>(cmd_v[i]);
        if(cmd[0] == microstrain_3dmgx1_imu::IMU::CMD_INSTANT_GYRO_QUAT_VECTOR) {
          cmd[0] = microstrain_3dmgx1_imu::IMU::CMD_INSTANT_VECTOR;
          imu.send(cmd,1);
          cmd[0] = microstrain_3dmgx1_imu::IMU::CMD_INSTANT_QUAT;
          imu.send(cmd,1);
        }
        else
          imu.send(cmd,1);
        getData(cmd_v[i],NULL,NULL);
      }
    }
    self_test_.checkTest();
    diagnostic_.update();

    if(sigint_emited) {
      stop();
      ros::shutdown();
    }
  }

  /*!
   * Service PublishTF [publisTF: true/false, stab_publisTF: true/false]: Enables or disables broadcasting of the IMU reference system.
   *
   * \brief broadcatsTf Service broadcast_tf [publisTF: true/false, stab_publisTF: true/false]: Enables or disables broadcasting of the IMU reference system for instantaneous and stabilised data.
   * \param req Server Requets object
   * \param res Server Response object
   * \return The broadcast status
   */
  bool pubishTFCallback(microstrain_3dmgx1_imu::PublishTFRequest &req, microstrain_3dmgx1_imu::PublishTFResponse &res) {
    string cad1,cad2;

    publishTF = req.publishTF;
    stab_publishTF = req.stab_publishTF;


    publishTF ? cad1="TF Active" : cad1="TF Inactive";
    stab_publishTF ? cad2="Stab TF Active" : cad2="Stab TF Inactive";

    res.response=cad1 + " " + cad2;

    // write changes to param server
    private_node_handle_.setParam("publish_tf", publishTF);
    private_node_handle_.setParam("publish_stab_tf", stab_publishTF);

    return true;
  }

  ~ImuNode()
  {
    stop();
  }

  /**
   * @brief clearErrorStatus Clear the error status string
   */
  void clearErrorStatus()
  {
    error_status_.clear();
  }

  /**
   * @brief start Configure the communication throught serial port. And configure the imu data time, do the bias capture, obtain the IMU data scaled and starts the data streams.
   * @return 0 if ok or -1 if an error ocurred
   */
  void start()
  {
    int intentos=5;
    while(intentos) {
      stop();

      try
      {
        imu.openPort(port.c_str());
      }
      catch (microstrain_3dmgx1_imu::Exception& e) {
        error_count_++;
        ROS_ERROR("%s", e.what());
        diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, e.what());
      }
      try
      {
        diagnostic_.setHardwareID(getID(true));

        ROS_INFO("Initializing IMU time with offset %f.", offset_);

        imu.initTime(offset_);
        imu.getScales();
        imu.configureCalculationCycle();

        if (auto_capture_bias || capture_bias_requested_)
        {
          captureBias(0);
          capture_bias_requested_ = false;
          auto_capture_bias = false; // No need to do this each time we reopen the device.
        }
        else
        {
          ROS_INFO("Not capturing the bias of the IMU sensor. Use the captureBias service to capture it before use.");
        }

        ROS_INFO("IMU sensor initialized.");

        if(cmd_v.size() == 1 && !poll_mode && cmd_v[0] != microstrain_3dmgx1_imu::IMU::CMD_INSTANT_GYRO_QUAT_VECTOR)
          imu.setContinuous(cmd_v[0]);

        if(!poll_mode)
          wallTimer.start();

        freq_diag_->clear();

        running = true;
        break;
      }
      catch (microstrain_3dmgx1_imu::Exception& e) {
        error_count_++;
        ROS_ERROR("Exception thrown while starting IMU. This sometimes happens if you are not connected to an IMU or if another process is trying to access the IMU port. You may try 'lsof|grep %s' to see if other processes have the port open. %s", port.c_str(), e.what());
        diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Error opening IMU.");
      }
      intentos--;
    }
    if(!intentos) {
      ROS_ERROR("Imposible to connect with the IMU");
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Imposible to connect with the IMU");
      ros::shutdown();
    }
  }

  /**
   * @brief getID Obtains the sensor information (firmware and serial number)
   * @param output_info if true prints the info
   * @return device info string
   */
  std::string getID(bool output_info = false)
  {
    int serial_num;
    std::string firmware;

    imu.getFirmwareVersion(&firmware);
    imu.getSerialNumber(&serial_num);

    if (output_info)
      ROS_INFO("Connected to IMU [%d] with firmware [%s]",serial_num,firmware.c_str());

    return (boost::format("%d_%s")%serial_num%firmware.c_str()).str();
  }
  
  /**
   * @brief stop Stops the data streams, and close the serial port
   * @return 0 if all is ok, -1 if an error occurred
   */
  void stop()
  {
    if(running)
    {
      try
      {
        if(imu.getContinuous())
          imu.stopContinuous();
        else
          wallTimer.stop();
        running = false;
        imu.closePort();
      } catch (microstrain_3dmgx1_imu::Exception& e) {
        error_count_++;
        ROS_ERROR("Exception thrown while stopping IMU. %s", e.what());
        diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,"Exception thrown while stopping IMU "+string(e.what()));
      }
    }
    ROS_INFO("Closing...");

  }

  /**
   * @brief InterruptionTest Test if the serial port has other connections
   * @param status node status
   */
  void interruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (imu_data_pub_.getNumSubscribers() == 0 )
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "No operation interrupted.");
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "There were active subscribers.  Running of self test interrupted operations.");
  }

  /**
   * @brief Temperature Test the IMU temperature
   * @param status node status
   */
  void temperatureTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    double temp=0.0;
    uint8_t cmd[1];

    try {
      cmd[0]=microstrain_3dmgx1_imu::IMU::CMD_TEMPERATURE;
      imu.send(cmd,1);
      imu.getTemperature(&temp);
      status.add("Temperature",temp);
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature test");
    }
    catch(microstrain_3dmgx1_imu::Exception& e) {
      ROS_ERROR("It is not possible to get temperature: %s",e.what());
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "It is not possible to get temperature: "+string(e.what()));
    }
  }

  /**
   * @brief ConnectTest Test if the connection throght the serial port is possible
   * @param status node status
   */
  void connectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    try {
      imu.openPort(port.c_str());
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected successfully.");
    }
    catch(microstrain_3dmgx1_imu::Exception& e) {
      ROS_ERROR("It is not possible to connect: %s",e.what());
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "It is not possible to connect: "+string(e.what()));
    }
  }

  /**
   * @brief ReadIDTest Test if it is possible to read the IMU info (firmware and serial number)
   * @param status node status
   */
  void readIDTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    try {
      self_test_.setID(getID());
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Read Successfully");
    }
    catch(microstrain_3dmgx1_imu::Exception& e) {
      ROS_ERROR("%s",e.what());
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not read de IMU ID: "+string(e.what()));
    }
  }

  /**
   * @brief biasTest Test calculate the IMU Bias
   * @param status node status
   */
  void biasTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    calculateBias();

    status.add("Bias_X", angrate_bias[X]);
    status.add("Bias_Y", angrate_bias[Y]);
    status.add("Bias_Z", angrate_bias[Z]);

    status.add("Linear Accel Bias X",linear_accel_bias[X]);
    status.add("Linear Accel Bias Y", linear_accel_bias[Y]);
    status.add("Linear Accel Bias Z", linear_accel_bias[Z]);

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Successfully calculated gyro biases.");
  }

  /**
   * @brief transformVector Transforms a vector from de NED IMU reference frame to the ENU reference frame
   * @param vIn Input vector to be transformed
   * @param vOut Output transformed vector
   */
  void transformVector(double* vIn, double* vOut){
    //Transforms has singular problems!!!
    tf2::Vector3 in,out;

    tf2::Quaternion qOut,qIn(vIn[X],vIn[Y],vIn[Z],0.0);


    qOut=q_rot*qIn*q_rot.inverse();

    vOut[X] = qOut.x();
    vOut[Y] = qOut.y();
    vOut[Z] = qOut.z();
  }

  /**
   * @brief transformQuaternion Transform a Quaternion from the IMU frame (NED) to ENU reference Frame
   * @param qIn Input quaternion
   * @return transformed quaternion
   */
  tf2::Quaternion transformQuaternion(tf2::Quaternion qIn) {
    tf2::Quaternion qOut;

    qOut=q_rot*qIn*q_rot.inverse();
    qOut.normalize();

    return qOut;
  }

  /**
   * @brief createImuData Creates the IMU data message
   * @param accel Acceleration components
   * @param angrate Angular velocity components
   * @param q2 Quaternion that defines the imu orientation in ENU convection
   * @param time Imu time stamp
   * @param ang_vel_cov Angular velocity covariance matrix
   * @param lin_acc_cov Linear acceleration convariance matrix
   * @param orientation_cov Euler angles convariance
   * @return Imu message
   */
  sensor_msgs::Imu createImuData(double *accel, double *angrate, tf2::Quaternion q2, uint64_t time, vector<double> ang_vel_cov, vector<double> lin_acc_cov,vector<double> orientation_cov,std::string frameid_,double *angrate_bias)
  {
    sensor_msgs::Imu imuData;

    //    ROS_INFO("ANT %lf %lf %lf %lf %lf %lf",accel[X],accel[Y],accel[Z],angrate[X],angrate[Y],angrate[Z]);
    if(use_enu_frame) {
      transformVector(accel,accel);
      transformVector(angrate,angrate);
    }
    //    ROS_INFO("DES %lf %lf %lf %lf %lf %lf",accel[X],accel[Y],accel[Z],angrate[X],angrate[Y],angrate[Z]);
    imuData.linear_acceleration.x = accel[X];
    imuData.linear_acceleration.y = accel[Y];
    imuData.linear_acceleration.z = accel[Z];

    imuData.angular_velocity.x = angrate[X]-angrate_bias[X];
    imuData.angular_velocity.y = angrate[Y]-angrate_bias[Y];
    imuData.angular_velocity.z = angrate[Z]-angrate_bias[Z];

    imuData.orientation = tf2::toMsg(q2);

    std::copy_n(orientation_cov.begin(),orientation_cov.size(),imuData.orientation_covariance.begin());
    std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),imuData.angular_velocity_covariance.begin());
    std::copy_n(lin_acc_cov.begin(),lin_acc_cov.size(),imuData.linear_acceleration_covariance.begin());

    imuData.header.stamp = ros::Time::now().fromNSec(time);
    imuData.header.frame_id = frameid_;

    return imuData;
  }

  /**
   * @brief createQuaternion Create the quaternion that represent the IMU orientation
   * @param quat vector of four doubles (the four components of the quaternion).
   * @return Quaternion that represents the imu orientation
   */
  tf2::Quaternion createQuaternion(double *quat)
  {
    tf2::Quaternion q(quat[1],quat[2],quat[3],quat[0]);

    q.normalize();

    if(use_enu_frame)
      q=transformQuaternion(q);

    return q;
  }

  /**
   * @brief createQuaternion Create the quaternion that represent the IMU orientation
   * @param roll orientation WRT x axis
   * @param pitch orientation WRT y axis
   * @param yaw orientation WRT z axis
   * @return Quaternion that represents the imu orientation
   */
  tf2::Quaternion createQuaternion(double roll,double pitch,double yaw)
  {
    tf2::Quaternion q;

    q.setRPY(roll*M_PI/180,pitch*M_PI/180,yaw*M_PI/180);
    q.normalize();

    if(use_enu_frame)
      q=transformQuaternion(q);

    return q;
  }

  /**
   * @brief transformRPY Transforms the euler angles (roll, pitch and yaw) to the ENU frame.
   * @param roll_in input roll in NED frame
   * @param pitch_in input pitch in NED frame
   * @param yaw_in input yaw in NED frame
   * @param roll_out output roll in ENU frame
   * @param pitch_out output pitch in ENU frame
   * @param yaw_out output yaw in ENU frame
   */
  void transformRPY(tf2Scalar roll_in,tf2Scalar pitch_in, tf2Scalar yaw_in, tf2Scalar &roll_out,tf2Scalar &pitch_out, tf2Scalar &yaw_out) {
    tf2::Quaternion q;
    tf2::Matrix3x3 m;

    q.setRPY(roll_in*M_PI/180,pitch_in*M_PI/180,yaw_in*M_PI/180);
    q.normalize();

    q=transformQuaternion(q);

    m.setRotation(q);
    m.getRPY(roll_out,pitch_out,yaw_out);

    roll_out = roll_out*180/M_PI;
    pitch_out = pitch_out*180/M_PI;
    yaw_out = yaw_out*180/M_PI;
  }
  /**
   * @brief createIMUNoGrabityData Creates the IMU data message without the gravity component
   * @param time IMU time
   * @param angrate Angular velocity components
   * @param accel Acceleration components
   * @param q Quaternion of the IMU orientation
   * @param ang_vel_cov Angular velocity covariance matrix
   * @param lin_acc_cov Linear acceleration convariance matrix
   * @param orientation_cov Euler angles convariance
   * @param frameid_ Name of the frame WRT the IMU
   * @param angrate_bias Bias of the angular rate to subtract to the measures
   * @param linear_accel_bias Bias of the linear acceleration to subtract to the measures
   * @return Imu message without the gravity component
   */
  sensor_msgs::Imu createIMUNoGrabityData(uint64_t time, double *angrate, double *accel,tf2::Quaternion q, vector<double> ang_vel_cov, vector<double> lin_acc_cov_no_grab, vector<double> orientation_cov, std::string frameid_, double *angrate_bias,double *linear_accel_bias)
  {
    tf2::Transform transform;
    tf2::Vector3 g_w,g_b,a_b;
    sensor_msgs::Imu imuData;
    tf2::Matrix3x3 m;

    if(use_enu_frame) {
      g_w[X] = 0.0;
      g_w[Y] = 0.0;
      g_w[Z] = 9.81;
    }
    else {
      g_w[X] = 0.0;
      g_w[Y] = 0.0;
      g_w[Z] = -9.81;
    }

    transform.setRotation(q.inverse());

    g_b= transform*g_w;

    //      ROS_INFO("%lf %lf %lf %lf %lf %lf",g_b[X],g_b[Y],g_b[Z],accel[X],accel[Y],accel[Z]);
    a_b[X] = accel[X] - g_b[X];
    a_b[Y] = accel[Y] - g_b[Y];
    a_b[Z] = accel[Z] - g_b[Z];

    imuData.linear_acceleration.x = a_b[X];
    imuData.linear_acceleration.y = a_b[Y];
    imuData.linear_acceleration.z = a_b[Z];

    if(!capturingBias) {
      imuData.linear_acceleration.x -= linear_accel_bias[X];
      imuData.linear_acceleration.y -= linear_accel_bias[Y];
      imuData.linear_acceleration.z -= linear_accel_bias[Z];
    }

    imuData.angular_velocity.x = angrate[X]-angrate_bias[X];
    imuData.angular_velocity.y = angrate[Y]-angrate_bias[Y];
    imuData.angular_velocity.z = angrate[Z]-angrate_bias[Z];

    imuData.orientation = tf2::toMsg(q);

    //      ROS_INFO("%lf %lf %lf %lf %lf %lf",a_b[X],a_b[Y],a_b[Z],linear_accel_bias[X],linear_accel_bias[Y],linear_accel_bias[Z]);
    std::copy_n(orientation_cov.begin(),orientation_cov.size(),imuData.orientation_covariance.begin());
    std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),imuData.angular_velocity_covariance.begin());
    std::copy_n(lin_acc_cov_no_grab.begin(),lin_acc_cov_no_grab.size(),imuData.linear_acceleration_covariance.begin());

    imuData.header.stamp = ros::Time::now().fromNSec(time);
    imuData.header.frame_id = frameid_;

    return imuData;
  }

  /**
   * @brief createMagData reates the Magnetic Field data message
   * @param mag Magnetic field components
   * @param time Imu time stamp
   * @param mag_cov Magnetic field covariance matrix
   * @param frameid_ Name of the frame WRT the IMU
   * @return Magnetic field message
   */
  sensor_msgs::MagneticField createMagData(double *mag, uint64_t time,vector<double> mag_cov,std::string frameid_)
  {
    sensor_msgs::MagneticField magData;

    if(use_enu_frame) {
      transformVector(mag,mag);
    }

    magData.header.stamp = ros::Time::now().fromNSec(time);
    magData.header.frame_id = frameid_;
    magData.magnetic_field.x = mag[0];
    magData.magnetic_field.y = mag[1];
    magData.magnetic_field.z = mag[2];

    std::copy_n(mag_cov.begin(),mag_cov.size(),magData.magnetic_field_covariance.begin());

    return magData;
  }

  /**
   * @brief getData Read de data from the IMU and publish it in the corresponding topic
   * @param actual_cmd Imu command of the data to be read
   * @param iD return Imu message to capture the bias of the gyro
   * @param iDNG return Imu message to capture de bias of the linear no gravity
   * @return
   */
  void getData(microstrain_3dmgx1_imu::IMU::cmd actual_cmd,sensor_msgs::Imu *iD,sensor_msgs::Imu *iDNG)
  {
    uint64_t time=0;
    double accel[3];
    double angrate[3];
    double mag[3];
    double quat[4];

    tf2::Quaternion q;
    tf2Scalar roll,pitch,yaw;
    tf2::Matrix3x3 m;

    sensor_msgs::Imu imuData; //! Message data type for the data IMU
    sensor_msgs::Imu imuDataNoGraviy; //! Message data type for the IMU data without the grabity component
    sensor_msgs::MagneticField magData; //! Message type for the IMU magnetic field data
    geometry_msgs::QuaternionStamped quaternion_msg; //! Message type for the orientation data (Quaternion data type)
    microstrain_3dmgx1_imu::EulerStamped eulerMsg; //! Message type for the orientation data (Euler Stamped)

    geometry_msgs::TransformStamped transformStamped;
    bool localPublishTF=false, stab_localPublishTF=false;

    vector<double> v={0.0,0.0,0.0,
                      0.0,0.0,0.0,
                      0.0,0.0,0.0};
    double b[3] = {0.0,0.0,0.0};

    try
    {
      switch (actual_cmd) {
      case microstrain_3dmgx1_imu::IMU::CMD_RAW_SENSOR:
        imu.getRawSensorOutput(&time,mag, accel, angrate);
        imuData = createImuData(accel,angrate,tf2::Quaternion(0,0,0,1),time,v,v,v,frameid_,b);
        magData = createMagData(mag, time,v,frameid_);
        imu_data_raw_pub_.publish(imuData);
        raw_mag_pub.publish(magData);

        break;
      case microstrain_3dmgx1_imu::IMU::CMD_GYRO_VECTOR:
        imu.getVectors(&time,mag,accel,angrate,true);
        imuData = createImuData(accel,angrate,tf2::Quaternion(0,0,0,1),time,stab_ang_vel_cov,stab_lin_acc_cov,v,stab_frameid_,stab_angrate_bias);
        magData = createMagData(mag, time,stab_mag_cov,stab_frameid_);
        if(!capturingBias) {
          imu_stab_data_pub_.publish(imuData);
          stab_mag_pub.publish(magData);
        }

        break;
      case microstrain_3dmgx1_imu::IMU::CMD_INSTANT_VECTOR:
        imu.getVectors(&time,mag,accel,angrate,false);
        imuData = createImuData(accel,angrate,tf2::Quaternion(0,0,0,1),time,ang_vel_cov,lin_acc_cov,v,frameid_,angrate_bias);
        magData = createMagData(mag, time,mag_cov,frameid_);
        if(!capturingBias) {
          imu_data_pub_.publish(imuData);
          mag_pub.publish(magData);
//          m.setRotation(q);
//          m.getRPY(roll,pitch,yaw);
//          ROS_INFO("Orientation 1: %lf %lf %lf",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
        }

        break;
      case microstrain_3dmgx1_imu::IMU::CMD_INSTANT_QUAT:
        imu.getQuaternions(&time,quat,false);
        q=createQuaternion(quat);

        quaternion_msg.quaternion = tf2::toMsg(q);
        quaternion_msg.header.stamp = ros::Time::now().fromNSec(time);
        quaternion_msg.header.frame_id = frameid_;
        if(!capturingBias) {
          orientation_pub.publish(quaternion_msg);
          localPublishTF = true;
//          m.setRotation(q);
//          m.getRPY(roll,pitch,yaw);
//          ROS_INFO("Orientation 2: %lf %lf %lf",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);

        }
        break;
      case microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT:
        imu.getQuaternions(&time,quat,true);
        q=createQuaternion(quat);

        quaternion_msg.quaternion = tf2::toMsg(q);
        quaternion_msg.header.stamp = ros::Time::now().fromNSec(time);
        quaternion_msg.header.frame_id = stab_frameid_;
        if(!capturingBias) {
          stab_orientation_pub.publish(quaternion_msg);
          stab_localPublishTF = true;

//          m.setRotation(q);
//          m.getRPY(roll,pitch,yaw);
//          ROS_INFO("Orientation 2: %lf %lf %lf",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
        }
        break;
      case microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_VECTOR:
        imu.getGyroStabQuatVectors(&time,quat,mag,accel,angrate,true);

        q = createQuaternion(quat);
        imuData = createImuData(accel, angrate, q, time,stab_ang_vel_cov,stab_lin_acc_cov,stab_orientation_cov,stab_frameid_,stab_angrate_bias);
        //Paso por puntero, ya está aplicada la transformacion de angrate y accel!!
        imuDataNoGraviy = createIMUNoGrabityData(time, angrate,accel,q,stab_ang_vel_cov,stab_lin_acc_cov_no_grab,stab_orientation_cov,stab_frameid_,stab_angrate_bias,stab_linear_accel_bias);
        magData = createMagData(mag, time,stab_mag_cov,stab_frameid_);
        if(!capturingBias) {
          imu_stab_data_pub_.publish(imuData);
          imu_stab_data_no_grab_pub_.publish(imuDataNoGraviy);
          stab_mag_pub.publish(magData);
          stab_localPublishTF = true;

//          m.setRotation(q);
//          m.getRPY(roll,pitch,yaw);
//          ROS_INFO("Orientation 3: %lf %lf %lf",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
        }

        break;

      case microstrain_3dmgx1_imu::IMU::CMD_INSTANT_GYRO_QUAT_VECTOR:
        imu.getVectors(&time,mag,accel,angrate,false);
        imu.getQuaternions(&time,quat,false);

        q = createQuaternion(quat);
        imuData = createImuData(accel, angrate, q, time,ang_vel_cov,lin_acc_cov,orientation_cov,frameid_,angrate_bias);
        //Paso por puntero, ya está aplicada la transformacion de angrate y accel!!
        imuDataNoGraviy = createIMUNoGrabityData(time, angrate,accel,q,ang_vel_cov,lin_acc_cov_no_grab,orientation_cov,frameid_,angrate_bias,linear_accel_bias);
        magData = createMagData(mag, time,mag_cov,frameid_);
        if(!capturingBias) {
          imu_data_pub_.publish(imuData);
          imu_data_no_grab_pub_.publish(imuDataNoGraviy);
          mag_pub.publish(magData);
          localPublishTF = true;

//          m.setRotation(q);
//          m.getRPY(roll,pitch,yaw);
//          ROS_INFO("Orientation 3: %lf %lf %lf",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);

        }

        break;

      case microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_INSTANT_VECTOR:
        imu.getGyroStabQuatVectors(&time,quat,mag,accel,angrate,false);

        q = createQuaternion(quat);
        imuData = createImuData(accel, angrate, q, time,ang_vel_cov,lin_acc_cov,orientation_cov,frameid_,angrate_bias);
        //Paso por puntero, ya está aplicada la transformacion de angrate y accel!!
        imuDataNoGraviy = createIMUNoGrabityData(time, angrate,accel,q,ang_vel_cov,lin_acc_cov_no_grab,orientation_cov,frameid_,angrate_bias,linear_accel_bias);

        break;
      case microstrain_3dmgx1_imu::IMU::CMD_INSTANT_EULER:
        imu.getEulerAngles(&time,&pitch,&roll,&yaw,false);
        q=createQuaternion(roll,pitch,yaw);

        if(use_enu_frame)
          transformRPY(roll,pitch,yaw,roll,pitch,yaw);


        eulerMsg.roll=roll;
        eulerMsg.pitch=pitch;
        eulerMsg.yaw=yaw;
        eulerMsg.header.frame_id = frameid_;
        eulerMsg.header.stamp=ros::Time::now().fromNSec(time);

        if(!capturingBias) {
          euler_pub.publish(eulerMsg);
          localPublishTF = true;
//          ROS_INFO("Orientation 4a: %lf %lf %lf",roll,pitch,yaw);

//          m.setRotation(q);
//          m.getRPY(roll,pitch,yaw);
//          ROS_INFO("Orientation 4b: %lf %lf %lf",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
        }

        break;
      case microstrain_3dmgx1_imu::IMU::CMD_GYRO_EULER:
        imu.getEulerAngles(&time,&pitch,&roll,&yaw,true);
        q=createQuaternion(roll,pitch,yaw);

        if(use_enu_frame)
          transformRPY(roll,pitch,yaw,roll,pitch,yaw);

        eulerMsg.roll=roll;
        eulerMsg.pitch=pitch;
        eulerMsg.yaw=yaw;
        eulerMsg.header.frame_id = frameid_;
        eulerMsg.header.stamp=ros::Time::now().fromNSec(time);

        if(!capturingBias) {
          stab_euler_pub.publish(eulerMsg);
          stab_localPublishTF = true;

//          ROS_INFO("Orientation 4a: %lf %lf %lf",roll,pitch,yaw);

//          m.setRotation(q);
//          m.getRPY(roll,pitch,yaw);
//          ROS_INFO("Orientation 4b: %lf %lf %lf",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
        }
        break;
      default:
        break;
      }


      if(localPublishTF && publishTF) {
        transformStamped.header.stamp = ros::Time::now().fromNSec(time);
        transformStamped.header.frame_id = base_frame;
        transformStamped.child_frame_id = frameid_;
        transformStamped.transform.translation.x = tf_translation[X];
        transformStamped.transform.translation.y = tf_translation[Y];
        transformStamped.transform.translation.z = tf_translation[Z];

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
      }

      if(stab_localPublishTF && stab_publishTF) {
        transformStamped.header.stamp = ros::Time::now().fromNSec(time);
        transformStamped.header.frame_id = base_frame;
        transformStamped.child_frame_id = stab_frameid_;
        transformStamped.transform.translation.x = stab_tf_translation[X];
        transformStamped.transform.translation.y = stab_tf_translation[Y];
        transformStamped.transform.translation.z = stab_tf_translation[Z];

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        stab_br.sendTransform(transformStamped);
      }

      clearErrorStatus(); // If we got here, then the IMU really is working. Next time an error occurs, we want to print it.
    }
    catch(microstrain_3dmgx1_imu::CorruptedDataException& ce) {
      error_count_++;
      ROS_WARN("Corrupted data, data packet discarted");
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Corrupted data, data packet discarted: " + string(ce.what()));
    }
    catch (microstrain_3dmgx1_imu::Exception& e) {
      error_count_++;
      ROS_WARN("Exception thrown while trying to get the IMU reading. This sometimes happens due to a communication glitch, or if another process is trying to access the IMU port. You may try 'lsof|grep %s' to see if other processes have the port open. %s", port.c_str(), e.what());
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Problem with IMU reading" + string(e.what()));
    }
    freq_diag_->tick();

    if(capturingBias) {
      *iD = imuData;
      *iDNG = imuDataNoGraviy;
    }

  }

  /**
   * @brief StreamedDataTest Test if the continuous mode of the IMU works correctly
   * @param status node status
   */
  void streamedDataTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    uint64_t time;
    double accel[3];
    double angrate[3];
    double quat[3];
    double mag[3];

    try {
      imu.setContinuous(microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_INSTANT_VECTOR);

      for (int i = 0; i < 100; i++)
      {
        imu.getGyroStabQuatVectors(&time,quat,mag,accel,angrate,false);
      }

      imu.stopContinuous();

      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Data streamed successfully.");

    }
    catch(microstrain_3dmgx1_imu::Exception& e) {
      ROS_ERROR("%s",e.what());
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(e.what()));
    }
  }


  /**
   * @brief GravityTest Test if the imu acceleration works correctly
   * @param status node status
   */
  void gravityTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    uint64_t time;
    double accel[3];
    double angrate[3];
    double quat[3];
    double mag[3];

    double grav = 0.0;

    double grav_x = 0.0;
    double grav_y = 0.0;
    double grav_z = 0.0;

    try {
      imu.setContinuous(microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_INSTANT_VECTOR);

      int num = 200;

      for (int i = 0; i < num; i++)
      {
        imu.getGyroStabQuatVectors(&time,quat,mag,accel,angrate,false);
        
        grav_x += accel[0];
        grav_y += accel[1];
        grav_z += accel[2];
      }
      
      imu.stopContinuous();

      grav += sqrt( pow(grav_x / (double)(num), 2.0) +
                    pow(grav_y / (double)(num), 2.0) +
                    pow(grav_z / (double)(num), 2.0));
      
      //      double err = (grav - microstrain_3dmgx1_imu::G);
      double err = (grav - 9.796);
      
      if (fabs(err) < .05)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Gravity detected correctly.");
      } else {
        status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Measured gravity deviates by %f", err);
      }

      status.add("Measured gravity", grav);
      status.add("Gravity error", err);
    }
    catch(microstrain_3dmgx1_imu::Exception& e) {
      ROS_ERROR("%s", e.what());
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not be done the gravity test: "+string(e.what()));
    }
  }

  /**
   * @brief DisconnectTest Test if the IMU is disconnected correctly
   * @param status
   */
  void disconnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    stop();

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Disconnected successfully.");
  }


  /**
   * @brief ResumeTest Test that check that, after disconnect the imu, it is possible to reconnect it again
   * @param status node status
   */
  void resumeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    start();
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Previous operation resumed successfully.");
  }

  /**
   * @brief deviceStatus Check the status of the device
   * @param status node status
   */
  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (running)
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "IMU is running");
    else if(capturingBias)
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "IMU is capturing bias");
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU is stopped");

    status.add("Device", port);
    status.add("TF frame", frameid_);
    status.add("StabilisedTF frame", stab_frameid_);
    status.add("Base frame", base_frame);
    status.add("Error count", error_count_);
    status.add("ENU Frame",use_enu_frame);
    status.add("Publish TF",publishTF);
    status.add("Publish Stabilised TF",stab_publishTF);
    status.add("Pool Mode",poll_mode);
    status.add("Raw Data",raw_sensor);
    status.add("Stabilised Acceleration, Angular Rate and Magnetic Field Data",stab_mag_accel_angrate);
    status.add("Instantaneous Acceleration, Angular Rate and Magnetic Field Data",instant_mag_accel_angrate);
    status.add("Stabilised Orientation",stab_quat_orient);
    status.add("Instantaneous Orientation",instant_quat_orient);
    status.add("Stabilised Acceleration, Angular Rate, Magnetic Field and Orientation Data",stab_orient_mag_accel_angrate);
    status.add("Instantaneous Acceleration, Angular Rate, Magnetic Field and Orientation Data",instant_orient_mag_accel_angrate);
    status.add("Stabilised Euler Angles",stab_euler_angles);
    status.add("Instantaneous Euler Angles",instant_euler_angles);
    status.add("Capturing Bias",capturingBias);
  }

  /**
   * @brief biasStatus Check the status of the imu bias
   * @param status node status
   */
  void biasStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (biasCaptured_)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Bias is captured");
      status.add("X bias", angrate_bias[X]);
      status.add("Y bias", angrate_bias[Y]);
      status.add("Z bias", angrate_bias[Z]);

      status.add("Linear Accel Bias X",linear_accel_bias[X]);
      status.add("Linear Accel Bias Y", linear_accel_bias[Y]);
      status.add("Linear Accel Bias Z", linear_accel_bias[Z]);
    }
    else if(capturingBias) {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Bias is capturing");
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Bias is captured");
  }

  /**
   * @brief calibrationStatus Check the status of the imu calibration
   * @param status node status
   */
  void calibrationStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (calibrated)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Magnetometer calibrated in the actual execution");
      status.add("X offset", magOffset[X]);
      status.add("Y offset", magOffset[Y]);
      status.add("Z offset", magOffset[Z]);

    }
    else if(calibrationStarted) {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Magnetometer is calibrating");
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Magnetometer not calibrated in the actual execution");
  }

  /**
   * @brief addOffset Service to update the imu time offset
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool addOffset(microstrain_3dmgx1_imu::AddOffset::Request &req, microstrain_3dmgx1_imu::AddOffset::Response &resp)
  {
    double offset = req.add_offset;
    offset_ += offset;

    ROS_INFO("Adding %f to existing IMU time offset.", offset);
    ROS_INFO("Total IMU time offset is now %f.", offset_);

    // send changes to imu driver
    //    imu.setFixedOffset(offset_);

    // write changes to param server
    private_node_handle_.setParam("time_offset", offset_);

    // set response
    resp.total_offset = offset_;

    return true;
  }

  /**
   * @brief start_calibrate Service to start the magnetometer calibration
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool start_calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {

    stop();
    try {
      imu.openPort(port.c_str());
      imu.initializeMagCalibration();
      ROS_INFO("Starting magnetometer calibration");
    }
    catch (microstrain_3dmgx1_imu::Exception& e) {
          error_count_++;
          ROS_ERROR("Exception thrown while calibrating IMU %s", e.what());
          diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Exception thrown while calibrating IMU" + string(e.what()));
    }
    calibrateTimer.start();
    calibrationStarted = true;
    return true;
  }

  /**
   * @brief calibrateTimerCallback callback of the timer that is activated when the magnetometer calibration starts
   * @param event
   */
  void calibrateTimerCallback(const ros::WallTimerEvent& event){
    double mag[3],min[3],max[3];
    uint64_t time;
    if(calibrationStarted) {
      try {
        imu.collectHardIronData(&time,mag,min,max);
        ROS_INFO("Collecting calibration data: MAG: %lf %lf %lf MIN: %lf %lf %lf MAX: %lf %lf %lf",mag[X],mag[Y],mag[Z],min[X],min[Y],min[Z],max[X],max[Y],max[Z]);
      }
      catch (microstrain_3dmgx1_imu::Exception& e) {
        error_count_++;
        ROS_ERROR("Exception thrown while calibrating IMU %s", e.what());
        diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Exception thrown while calibrating IMU" + string(e.what()));
      }
    }
  }

  /**
   * @brief stop_calibrate [tipeId: 0/1, typeName: 3D/2D] Service to stop the calibration of de IMU. It receve the type of calibration 3D or 2D.
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool stop_calibrate(microstrain_3dmgx1_imu::StopCalibrate::Request &req, microstrain_3dmgx1_imu::StopCalibrate::Response &res) {

    uint8_t id;
    uint64_t time;

    if(calibrationStarted) {
      calibrateTimer.stop();

      ROS_INFO("%d",req.calibrationType.typeName.compare("2D"));

      //If the label is present it takes precedence over the id
      if(req.calibrationType.typeName.size()>0) {
        if(!req.calibrationType.typeName.compare("2D") || !req.calibrationType.typeName.compare("2d")) {
          id = microstrain_3dmgx1_imu::CalibrationType::calibration2D;
        }
        else if(!req.calibrationType.typeName.compare("3D") || !req.calibrationType.typeName.compare("3d")) {
          id = microstrain_3dmgx1_imu::CalibrationType::calibration3D;
        }
        else
          return false;
      }
      else {
        if(req.calibrationType.typeId == microstrain_3dmgx1_imu::CalibrationType::calibration2D)
          id = microstrain_3dmgx1_imu::CalibrationType::calibration2D;
        else if(req.calibrationType.typeId == microstrain_3dmgx1_imu::CalibrationType::calibration3D)
          id = microstrain_3dmgx1_imu::CalibrationType::calibration3D;
        else
          return false;
      }


      try {
        imu.computeHardIronData(id,magnitudeZ,&time,magOffset);
        ROS_INFO("Stopping magnetometer calibration. Offset: %lf %lf %lf",magOffset[X],magOffset[Y],magOffset[Z]);
        calibrated=true;
        calibrationStarted=false;
      }
      catch(microstrain_3dmgx1_imu::Exception& e) {
        error_count_++;
        ROS_ERROR("Exception thrown while calibrating IMU %s", e.what());
        diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Exception thrown while calibrating IMU" + string(e.what()));
      }
      start();
      return true;
    }
    else {
      ROS_ERROR("Calibration is not started");
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Calibration is not started");
      return false;
    }
  }

  /**
   * @brief captureBias Service to capture bias of the IMU
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool calculateBias(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    calculateBias();
    return true;
  }

  /**
   * @brief Poll Service to poll data to the IMU
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool poll(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    try {
      if(imu.getContinuous())
        imu.stopContinuous();
      }
    catch(microstrain_3dmgx1_imu::Exception& e) {
      error_count_++;
      ROS_ERROR("Problem stoping continous mode %s", e.what());
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Problem stoping continous mode " + string(e.what()));
    }

    if(wallTimer.hasStarted())
      wallTimer.stop();

    poll_mode=true;
    private_node_handle_.setParam("poll_mode",poll_mode);

    wallTimerCallback(ros::WallTimerEvent());

    return true;
  }

  /**
   * @brief Continuous Service change the node to continuous mode
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool continuous(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    if(wallTimer.hasStarted())
      return true;
    else
      wallTimer.start();

    if(cmd_v.size() == 1 && !imu.getContinuous() && cmd_v[0] != microstrain_3dmgx1_imu::IMU::CMD_INSTANT_GYRO_QUAT_VECTOR)
      imu.setContinuous(cmd_v[0]);

    poll_mode=false;
    private_node_handle_.setParam("poll_mode",poll_mode);

    return true;
  }

  /**
   * @brief calculateBias Service to capture the bias of the IMU
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool calculateBias() {
    bool old_running = running;

    try
    {
      capture_bias_requested_ = true;
      if (old_running)
      {
        stop();
        start(); // Start will do the capture of the bias.
      }
      else
      {
        imu.openPort(port.c_str());
        captureBias(0);
        imu.closePort();
      }
    } catch (microstrain_3dmgx1_imu::Exception& e) {
      error_count_++;
      biasCaptured_ = false;
      ROS_ERROR("Exception thrown while caluting bias of the IMU %s", e.what());
      stop();
      if (old_running)
        start(); // Might throw, but we have nothing to lose... Needs restructuring.
      return false;
    }

    return true;
  }

  /**
   * @brief captureBias sets the imu gyro bias and calculate the linear accel (without grabity)
   * @param it Number of attemps. If the capture it is not correct after 6 attempts, the node stops.
   */
  void captureBias(int it)
  {
    double rate[3] = {0.0,0.0,0.0};
    double accel[3] = {0.0,0.0,0.0};
    double stab_rate[3] = {0.0,0.0,0.0};
    double stab_accel[3] = {0.0,0.0,0.0};
    sensor_msgs::Imu imuData; //! Message data type for the data IMU
    sensor_msgs::Imu imuDataNoGraviy; //! Message data type for the IMU data without the grabity component

    ros::Time start_time;
    size_t count = 0;
    uint8_t cmd[1];
    capturingBias= true;
    // Expects to be called with the IMU stopped.
    ROS_INFO("Obtaining biases.");
    diagnostic_.update();
//    imu.captureGyroBias();
    
    //    ROS_INFO("gyro bias: %lf %lf %lf", bias_x_,bias_y_,bias_z_);
    start_time = ros::Time::now();

    while(ros::Time::now() - start_time < ros::Duration(20.0)){
      for(int i=0;i<cmd_capture_bias.size();i++) {
        cmd[0] = static_cast<uint8_t>(cmd_capture_bias[i]);
        //        ROS_INFO("%x",cmd[0]);
        imu.send(cmd,1);
        //        ROS_INFO("GetData");
        getData(cmd_capture_bias[i],&imuData,&imuDataNoGraviy);
        switch (cmd_capture_bias[i]) {
        case microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_VECTOR:
          stab_rate[X] += imuData.angular_velocity.x;
          stab_rate[Y] += imuData.angular_velocity.y;
          stab_rate[Z] += imuData.angular_velocity.z;

          //   ROS_INFO("Del %lf %lf %lf",imuData.angular_velocity.x,imuData.angular_velocity.y,imuData.angular_velocity.z);

          stab_accel[X] += imuDataNoGraviy.linear_acceleration.x;
          stab_accel[Y] += imuDataNoGraviy.linear_acceleration.y;
          stab_accel[Z] += imuDataNoGraviy.linear_acceleration.z;
          //            ROS_INFO("Accel: %lf %lf %lf",accel[X],accel[Y],accel[Z]);
          break;
        case microstrain_3dmgx1_imu::IMU::CMD_GYRO_QUAT_INSTANT_VECTOR:
          rate[X] += imuData.angular_velocity.x;
          rate[Y] += imuData.angular_velocity.y;
          rate[Z] += imuData.angular_velocity.z;

          accel[X] += imuDataNoGraviy.linear_acceleration.x;
          accel[Y] += imuDataNoGraviy.linear_acceleration.y;
          accel[Z] += imuDataNoGraviy.linear_acceleration.z;
          //            ROS_INFO("aDel %lf %lf %lf",deltaVel_.vector.x,deltaVel_.vector.y,deltaVel_.vector.z);
          //            ROS_INFO("Del %lf %lf %lf",del_vel[X],del_vel[Y],del_vel[Z]);
          break;
        }


        //        ROS_INFO("%lf %lf %lf",imuDataNoGraviy.linear_acceleration.x, imuDataNoGraviy.linear_acceleration.y, imuDataNoGraviy.linear_acceleration.z);

      }
      count++;
      freq_diag_->tick();
      diagnostic_.update();

      if(sigint_emited) {
        stop();
        ros::shutdown();
      }
    }
    ROS_INFO("Accel: %lf %lf %lf",accel[X],accel[Y],accel[Z]);
    linear_accel_bias[X] = accel[X]/count;
    linear_accel_bias[Y] = accel[Y]/count;
    linear_accel_bias[Z] = accel[Z]/count;

    ROS_INFO("Angular velocity: %lf %lf %lf",rate[X],rate[Y],rate[Z]);

    angrate_bias[X] = rate[X]/count;
    angrate_bias[Y] = rate[Y]/count;
    angrate_bias[Z] = rate[Z]/count;

    ROS_INFO("Stab Accel: %lf %lf %lf",stab_accel[X]/ count,stab_accel[Y]/ count,stab_accel[Z]/ count);

    stab_linear_accel_bias[X] = stab_accel[X]/count;
    stab_linear_accel_bias[Y] = stab_accel[Y]/count;
    stab_linear_accel_bias[Z] = stab_accel[Z]/count;

    ROS_INFO("Stab Angular velocity: %lf %lf %lf",stab_rate[X]/count,stab_rate[Y]/count,stab_rate[Z]/count);

    stab_angrate_bias[X] = stab_rate[X]/count;
    stab_angrate_bias[Y] = stab_rate[Y]/count;
    stab_angrate_bias[Z] = stab_rate[Z]/count;


    ROS_INFO("Angular velocidty: %lf %lf %lf",stab_rate[X],stab_rate[Y],stab_rate[Z]);
    double average_rate = sqrt(rate[X]*rate[X] + rate[Y]*rate[Y] + rate[Z]*rate[Z]) / count;
    double stab_average_rate = sqrt(stab_rate[X]*stab_rate[X] + stab_rate[Y]*stab_rate[Y] + stab_rate[Z]*stab_rate[Z]) / count;

    // capture succeeded
    if (/*average_rate < (max_drift_rate_*10) &&*/ stab_average_rate < max_drift_rate_) {
      ROS_INFO("Imu: Capture check succeeded: average angular drift = %f deg/sec, average stab angular drift = %f deg/sec, max drift rate %f deg/se", average_rate*180/M_PI, stab_average_rate*180/M_PI, max_drift_rate_*180/M_PI);
      biasCaptured_ = true;
      ROS_INFO("IMU gyro capture completed.");
      freq_diag_->clear();
    }
    // capture failed
    else{
      biasCaptured_ = false;
      ROS_ERROR("Imu: capture check failed: average angular drift = %f deg/sec, average stab angular drift = %f deg/sec, max drift rate %f deg/sec", average_rate*180/M_PI,stab_average_rate*180/M_PI, max_drift_rate_*180/M_PI);
      if(it < 6) {
        captureBias(it+1);
      }
      else {
        ROS_ERROR("Imu: Imposible to capture bias");
        ros::shutdown();
      }
    }
    capturingBias= false;
  }
};

/**
 * @brief onSigInt Activate a flag to close properly the node
 * @param signal
 */
void onSigInt(int signal) {
  sigint_emited = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "microstrain_3dmgx1_node",ros::init_options::NoSigintHandler);

  signal(SIGINT,onSigInt);
  ImuNode in;
  ros::spin();
  return 0;
}
