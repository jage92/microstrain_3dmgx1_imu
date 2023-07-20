/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-20010  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <stdlib.h>

#include <sys/time.h>

//#include <ros/console.h>

#include "microstrain_3dmgx1_imu/3dmgx1.h"

#include "poll.h"


//! Macro for throwing an exception with a message
#define IMU_EXCEPT(except, msg, ...) \
  { \
    char buf[1000]; \
    snprintf(buf, 1000, msg" (in microstrain_3dmgx1_imu::IMU:%s)", ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
  }


/**
 * @brief time_helper Helper function to get system time in nanoseconds.
 * @return the nanoseconds since 01/01/1970
 */
static unsigned long long time_helper()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (unsigned long long)(curtime.tv_sec) * 1000000000 + (unsigned long long)(curtime.tv_nsec);  
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (unsigned long long)(timeofday.tv_sec) * 1000000000 + (unsigned long long)(timeofday.tv_usec) * 1000;  
#endif
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::IMU Constructor
 */
microstrain_3dmgx1_imu::IMU::IMU() : fd(-1), continuous(false), gyroGainScale(10000),magGainScale(2000),accelGainScale(8500), offset_ticks(0)
{ }


/**
 * @brief microstrain_3dmgx1_imu::IMU::~IMU Destructor
 */
microstrain_3dmgx1_imu::IMU::~IMU()
{
  closePort();
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::initTime Initialize the sensor time
 * @param fix_off
 */
void microstrain_3dmgx1_imu::IMU::initTime(double fix_off)
{
  wraps = 0;

  uint8_t cmd[1];
  uint8_t rep[7];

  cmd[0] = CMD_TEMPERATURE;

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

  offset_ticks = convert2int(&rep[3]);

  start_time = time_helper();
  last_ticks = offset_ticks;

  // fixed offset
  fixed_offset = fix_off * 1e9;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::openPort Open the port to communicate with the IMU
 * @param port_name name of the port. For example /dev/ttyUSB0
 */
void microstrain_3dmgx1_imu::IMU::openPort(const char *port_name)
{
  closePort(); // In case it was previously open, try to close it first.

  // Open the port
  fd = open(port_name, O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR );
  if (fd < 0)
  {
    const char *extra_msg = "";
    switch (errno)
    {
      case EACCES:
        extra_msg = "You probably don't have premission to open the port for reading and writing.";
        break;
      case ENOENT:
        extra_msg = "The requested port does not exist. Is the IMU connected? Was the port name misspelled?";
        break;
    }

    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Unable to open serial port [%s]. %s. %s", port_name, strerror(errno), extra_msg);
  }

  // Lock the port
  struct flock fl;
  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len   = 0;
  fl.l_pid   = getpid();

  if (fcntl(fd, F_SETLK, &fl) != 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

  // Change port settings
  struct termios term;
  if (tcgetattr(fd, &term) < 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Unable to get serial port attributes. The port you specified (%s) may not be a serial port.", port_name);

  cfmakeraw( &term );
  cfsetispeed(&term, B38400);
  cfsetospeed(&term, B38400);

  if (tcsetattr(fd, TCSAFLUSH, &term) < 0 )
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.

  // Stop continuous mode
  stopContinuous();

  // Make sure queues are empty before we begin
  if (tcflush(fd, TCIOFLUSH) != 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Tcflush failed. Please report this error if you see it.");
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::stopContinuous Stops the continous mode
 */
void microstrain_3dmgx1_imu::IMU::stopContinuous()
{
  uint8_t cmd[3];

  cmd[0] = CMD_SET_CONTINUOUS;
  cmd[1] = 0x00;
  cmd[2] = 0x00;

  send(cmd, sizeof(cmd));

  usleep(1000000);

  if (tcflush(fd, TCIOFLUSH) != 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Tcflush failed");

  continuous = false;
}


/**
 * @brief microstrain_3dmgx1_imu::IMU::setContinuous Put the IMU into continuous mode
 * @param command command that is executed continously
 */
void microstrain_3dmgx1_imu::IMU::setContinuous(cmd command)
{
  uint8_t cmd[3];
  uint8_t rep[7];

  cmd[0] = CMD_SET_CONTINUOUS;
  cmd[1] = 0x00; //Confirms user intent
  cmd[2] = command;

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

  // Verify that continuous mode is set on correct command:
  if (rep[2] != command) {
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Problem activating Continous Mode");
  }

  continuous = true;
  extractTime(&rep[5]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::extractTime Extracts the time of each read trame
 * @param addr
 * @return
 */
uint64_t microstrain_3dmgx1_imu::IMU::extractTime(uint8_t* addr)
{
  uint32_t convertFactor = 10000000;//655360;
  uint32_t ticks = convert2int(addr);

  if (ticks < last_ticks) {
    wraps += 1;
  }

  last_ticks = ticks;

  uint64_t all_ticks = ((uint64_t)wraps << 32) - offset_ticks + ticks;

  return  start_time +  uint64_t((all_ticks * convertFactor)) + fixed_offset;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::closePort Close the IMU serial port
 */
void microstrain_3dmgx1_imu::IMU::closePort()
{
  if (fd != -1)
  {
    if (continuous)
      stopContinuous();

    if (close(fd) != 0)
      IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "Unable to close serial port; [%s]", strerror(errno));
    fd = -1;
  }
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getScales Read from the EEPROM the scaled constants to calculate the IMU measures from the read data
 */
void microstrain_3dmgx1_imu::IMU::getScales() {
    short address, value;

    address = M3D_GYROSCALE_ADDRESS;
    getEEPROMValue(address, &value);
    gyroGainScale = value;

    address = M3D_ACCELSCALE_ADDRESS;
    getEEPROMValue(address, &value);
    accelGainScale = value;

    address = M3D_MAGSCALE_ADDRESS;
    getEEPROMValue(address, &value);
    magGainScale = value;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::configureCalculationCycle Configure the IMU to get data to the rate of 100Hz for each message
 */
void microstrain_3dmgx1_imu::IMU::configureCalculationCycle() {
    short address, value;

    address = M3D_CALC_CYCLE_1_ADDRESS;
    value = 4;
    setEEPROMValue(address, value);

    address = M3D_CALC_CYCLE_2_ADDRESS;
    value = 10;
    setEEPROMValue(address, value);

    address = M3D_CALC_CYCLE_3_ADDRESS;
    value = 250;
    setEEPROMValue(address, value);

    address = M3D_CALC_CYCLE_4_ADDRESS;
    value = 10;
    setEEPROMValue(address, value);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getSerialNumber Get the serial number of the IMU
 * @param serialNum seral number returned
 */
void microstrain_3dmgx1_imu::IMU::getSerialNumber(int *serialNum) {
    uint8_t cmd[1];
    uint8_t  rep[5];

    cmd[0] = CMD_SERIAL_NUMBER;

    transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

    *serialNum = convert2int(&rep[1]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::computeHardIronData Computes the magnetic field data captures during the calibration process
 * @param calibrationType Calibration 3D or 2D
 * @param magnitudeZ magnitude Z obtained in the intermagnet.org web. Corresponds to the zone in which the IMU will work. It is use in 2D calibration.
 * @param time timestamp in which the data is computed
 * @param offset magnetic offset data calculate after calibration
 */
void microstrain_3dmgx1_imu::IMU::computeHardIronData(uint8_t calibrationType, int magnitudeZ, uint64_t *time, double *offset) {
    uint8_t cmd[6];
    uint8_t  rep[11];
    uint16_t mZ = uint16_t(magnitudeZ);
    int i;

    cmd[0] = CMD_COMPUTE_HARD_IRON_DATA;
    cmd[1] = 0x71;
    cmd[2] = 0x3E;
    cmd[3]= calibrationType;
    cmd[4] = mZ >> 8;
    cmd[5] = mZ;

    transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

    for (i=0; i<3; i++)
      offset[i] = double(convert2int(&rep[1 + i*2])/(COMMON_SCALE_CONSTANT/magGainScale));

    *time  = extractTime(&rep[7]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::collectHardIronData collect Magnetic data to calculates the maximum and minimum of each reference axis
 * @param time Time of the collection
 * @param mag magnetometer data obtained
 * @param max maximum data
 * @param min minimum data
 */
void microstrain_3dmgx1_imu::IMU::collectHardIronData(uint64_t *time, double *mag,double *max,double *min) {
    uint8_t cmd[1];
    uint8_t  rep[23];
    int i=0;

    cmd[0] = CMD_COLLECT_HARD_IRON_DATA;

    transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

    for (i=0; i<3; i++) {
        mag[i] = double(convert2int(&rep[1 + i*2])/(COMMON_SCALE_CONSTANT/magGainScale));
        min[i] = double(convert2int(&rep[7 + i*2])/(COMMON_SCALE_CONSTANT/magGainScale));
        max[i] = double(convert2int(&rep[13+ i*2])/(COMMON_SCALE_CONSTANT/magGainScale));
    }
    *time  = extractTime(&rep[19]);


}

/**
 * @brief microstrain_3dmgx1_imu::IMU::initializeMagCalibration Clean the hard iron offsets to start a new magnetometer calibration
 */
void microstrain_3dmgx1_imu::IMU::initializeMagCalibration() {
    uint8_t cmd[3];
    uint8_t  rep[5];

    cmd[0] = CMD_INITIALIZE_HARD_IRON_CALIBRATION;
    cmd[1] = 0x71;
    cmd[2] = 0x3E;

    transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getFirmwareVersion Reads the firmware version of the IMU
 * @param firmware firmware version read
 */
void microstrain_3dmgx1_imu::IMU::getFirmwareVersion(std::string *firmware) {
    uint8_t cmd[1];
    uint8_t  rep[5];
    int firmwareNum;
    int majorNum, minorNum, buildNum;


    cmd[0] = CMD_FIRWARE_VERSION;
    transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

    firmwareNum = convert2int(&rep[1]);

    if (firmwareNum > 0) {
        /* format for firmware number is #.#.## */
        majorNum = firmwareNum / 1000;
        minorNum = (firmwareNum % 1000) / 100;
        buildNum = firmwareNum % 100;
        //sprintf(firmware, "%d.%d.%d", majorNum, minorNum, buildNum);
        *firmware = std::to_string(majorNum)+"."+std::to_string(minorNum)+"."+std::to_string(buildNum);
    }
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getTemperature read the IMU temperature
 * @param temp temperature read.
 */
void microstrain_3dmgx1_imu::IMU::getTemperature(double *temp) {
    uint8_t  rep[7];
    double convertFactor = 5.0/65536.0;

    receive(CMD_TEMPERATURE, rep, sizeof(rep), 1000);
    *temp = (double(convert2int(&rep[1])* convertFactor)-0.5)*100.0;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getRawSensorOutput Reads the raw magnetic field, acceleration and angular rate data from the IMU
 * @param time timestamp in which the data is read
 * @param mag magnetic field components
 * @param accel linear acceleration components
 * @param angRate angular velocity components
 */
void microstrain_3dmgx1_imu::IMU::getRawSensorOutput(uint64_t *time, double *mag, double *accel, double *angRate) {
    uint8_t cmd[1];
    uint8_t rep[23];    
    int i;

    receive(CMD_RAW_SENSOR, rep, sizeof(rep), 1000);

    for (i=0; i<3; i++) {
        mag[i]     = double(convert2int(&rep[1 + i*2]));
        accel[i]   = double(convert2int(&rep[7 + i*2]));
        angRate[i] = double(convert2int(&rep[13+ i*2]));
        //time
    }
    *time  = extractTime(&rep[19]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getVectors reads the instantaneous or stabilized magnetic field, linear acceleration and angular velocity measusures from de IMU
 * @param time timestamp in which the data is obtained
 * @param mag magnetic field data
 * @param accel linear acceleration data
 * @param angRate angular velocity data
 * @param stableOption true if the read data is stabized and false if it is instantaneous
 */
void microstrain_3dmgx1_imu::IMU::getVectors(uint64_t *time, double *mag, double *accel, double *angRate, bool stableOption) {

    uint8_t cmd;
    uint8_t rep[23];

    int i;

    if (!stableOption)
        cmd = CMD_INSTANT_VECTOR;
    else
        cmd = CMD_GYRO_VECTOR;

    receive(cmd, rep, sizeof(rep), 1000);

    for (i=0; i<3; i++) {
        mag[i]     = double(convert2short(&rep[1 + i*2])/(COMMON_SCALE_CONSTANT/magGainScale));
        accel[i]   = double(convert2short(&rep[7 + i*2])/(COMMON_SCALE_CONSTANT/accelGainScale))*9.81;
        angRate[i] = double(convert2short(&rep[13+ i*2])/(COMMON_SCALE_CONSTANT/gyroGainScale));
    }
    *time  = extractTime(&rep[19]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getQuaternions reads the instantaneous or stabilized quaternion from the IMU
 * @param time timestamp in which the data is obtained
 * @param q quaternion read (w,x,y,z)
 * @param stableOption true if the read data is stabized and false if it is instantaneous
 */
void microstrain_3dmgx1_imu::IMU::getQuaternions(uint64_t *time, double *q, int stableOption) {
    uint8_t cmd;
    uint8_t  rep[13];
    int i;

    if (!stableOption)
        cmd = CMD_INSTANT_QUAT;
    else
        cmd = CMD_GYRO_QUAT;

    receive(cmd, rep, sizeof(rep), 1000);

    for (i=0; i<4; i++) {
        q[i] = double(convert2short(&rep[1 + i*2])/GYRO_SCALE_CONSTANT);
    }
    *time  = extractTime(&rep[9]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::captureGyroBias clean the gyro bias that the IMU captures automaticaly
 */
void microstrain_3dmgx1_imu::IMU::captureGyroBias() {
    uint8_t cmd[1];
    uint8_t  rep[5];


   cmd[0] = CMD_CAPTURE_GYRO_BIAS;

   transact(cmd, sizeof(cmd), rep, sizeof(rep), 30000);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getOrientMatrix reads the instantaneous or stabilized orientation matrix from the IMU
 * @param time timestamp in which the data is obtained
 * @param mx orientation matrix of nine elements read
 * @param stableOption true if the read data is stabized and false if it is instantaneous
 */
void microstrain_3dmgx1_imu::IMU::getOrientMatrix(uint64_t *time, double **mx, int stableOption) {
    uint8_t cmd;
    uint8_t  rep[23];
    int i, j;


    if (!stableOption)
        cmd = CMD_INSTANT_OR_MATRIX;
    else
        cmd = CMD_GYRO_OR_MATRIX;

    receive(cmd, rep, sizeof(rep), 1000);

    for (i=0; i<3; i++) {
        for (j=0;j<3;j++) {
            mx[i][j] = double(convert2short(&rep[1+2*(j*3+i)])/GYRO_SCALE_CONSTANT);
        }
    }
    *time  = extractTime(&rep[19]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getGyroStabQuatVectors reads the instantaneous or stabilized magnetic field, linear acceleration, angular velocity and the stabilized orientation (quaternion) measusures from de IMU
 * @param time timestamp in which the data is obtained
 * @param q quaternion read (w,x,y,z)
 * @param mag magnetic field data
 * @param accel linear acceleration data
 * @param angRate angular velocity data
 * @param stableOption true if the read data is stabized and false if it is instantaneous
 */
void microstrain_3dmgx1_imu::IMU::getGyroStabQuatVectors(uint64_t *time, double *q, double *mag, double *accel, double *angRate,bool stableOption) {

    uint8_t cmd;
    uint8_t  rep[31];

    int i;

    if (!stableOption)
        cmd = CMD_GYRO_QUAT_INSTANT_VECTOR;
    else
        cmd = CMD_GYRO_QUAT_VECTOR;

    receive(cmd, rep, sizeof(rep), 1000);


    /* quaternion data */
    for (i=0; i<4; i++) {
        q[i] = double(convert2short(&rep[1 + i*2])/GYRO_SCALE_CONSTANT);
    }

    /* vector data */
    for (i=0; i<3; i++) {
        mag[i]     = double(convert2short(&rep[9 + i*2])/(COMMON_SCALE_CONSTANT/magGainScale));
        accel[i]   = double(convert2short(&rep[15+ i*2])/(COMMON_SCALE_CONSTANT/accelGainScale))*9.81;
        angRate[i] = double(convert2short(&rep[21+ i*2])/(COMMON_SCALE_CONSTANT/gyroGainScale));
    }
    *time  = extractTime(&rep[27]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getEulerAngles reads the instantaneous or stabilized euler angles from the IMU
 * @param time timestamp in which the data is obtained
 * @param pitch pitch angle in degrees
 * @param roll roll angle in degrees
 * @param yaw yaw angle in degrees
 * @param stableOption true if the read data is stabized and false if it is instantaneous
 */
void microstrain_3dmgx1_imu::IMU::getEulerAngles(uint64_t *time, double *pitch, double *roll, double *yaw, bool stableOption) {
    uint8_t cmd;
    uint8_t  rep[11];
    double convertFactor = (360.0/65536.0);

    if (!stableOption)
        cmd = CMD_INSTANT_EULER;
    else
        cmd = CMD_GYRO_EULER;

    receive(cmd, rep, sizeof(rep), 1000);

    *roll  = double(convert2short(&rep[1]) * convertFactor);
    *pitch = double(convert2short(&rep[3]) * convertFactor);
    *yaw   = double(convert2short(&rep[5]) * convertFactor);
    *time  = extractTime(&rep[7]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::getEEPROMValue reads a value from the EEPROM of the IMU
 * @param address address to read
 * @param value value read
 */
void microstrain_3dmgx1_imu::IMU::getEEPROMValue(uint8_t address, short *value) {
    uint8_t cmdBuffer[3];
    uint8_t dataBuffer[7];

    /* check the address range - only 256 locations permitted. */
    if (address <0 || address > 255) {
      IMU_EXCEPT(microstrain_3dmgx1_imu::CorruptedDataException, "invalid address");
    }

    /* command requires an address */
    cmdBuffer[0] = CMD_SEND_EEPROM;
    cmdBuffer[1] = address >> 8;
    cmdBuffer[2] = address;


    transact(cmdBuffer, sizeof(cmdBuffer), dataBuffer, sizeof(dataBuffer), 1000);

    extractTime(&dataBuffer[3]);
    *value = convert2short(&dataBuffer[1]);
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::setEEPROMValue writes a value in the EEPROM of the IMU
 * @param address address to write
 * @param value value to write
 */
void microstrain_3dmgx1_imu::IMU::setEEPROMValue(uint8_t address, short value) {
    short valueCheck;
    uint8_t cmdBuffer[7];
    uint8_t dataBuffer[7];

    /* check the address range - only 256 locations permitted. */
    if (address <0 || address > 255) {
        IMU_EXCEPT(microstrain_3dmgx1_imu::CorruptedDataException, "invalid address");
    }

    /* command buffer */
    cmdBuffer[0] = CMD_PROG_EEPROM;
    cmdBuffer[1] = 0x71;
    cmdBuffer[2] = address >> 8;
    cmdBuffer[3] = address;
    cmdBuffer[4] = value >> 8;
    cmdBuffer[5] =  value;
    cmdBuffer[6] = 0xAA;  /* end of buffer */

    transact(cmdBuffer, sizeof(cmdBuffer), dataBuffer, sizeof(dataBuffer), 1000);

    valueCheck = convert2short(&dataBuffer[1]);
    extractTime(&dataBuffer[3]);

}

/**
 * @brief microstrain_3dmgx1_imu::IMU::send Sends a packet to the IMU.
 * @param cmd data to send
 * @param cmd_len length to the data to send
 * @return number of bytes sent
 */
int microstrain_3dmgx1_imu::IMU::send(uint8_t *cmd, int cmd_len)
{
  int bytes;

  bytes = write(fd, cmd, cmd_len);

  if (bytes < 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "error writing to IMU [%s]", strerror(errno));

  if (bytes != cmd_len)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "whole message not written to IMU");

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  if (tcdrain(fd) != 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "tcdrain failed");

  return bytes;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::read_with_timeout reads data from the IMU. It the timeout reaches throw a exception
 * @param fd file descritor
 * @param buff read data
 * @param count length of the read data
 * @param timeout timeout
 * @return number of bytes read
 */
int microstrain_3dmgx1_imu::IMU::read_with_timeout(int fd, uint8_t *buff, size_t count, int timeout)
{
  ssize_t nbytes;
  int retval;

  struct pollfd ufd[1];
  ufd[0].fd = fd;
  ufd[0].events = POLLIN;
//  timeout=0;
  if (timeout == 0)
    timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

  if ( (retval = poll(ufd, 1, timeout)) < 0 )
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "poll failed  [%s]", strerror(errno));

  if (retval == 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::TimeoutException, "timeout reached");

  nbytes = read(fd, buff, count);

  if (nbytes < 0)
    IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "read failed  [%s]", strerror(errno));

  return nbytes;
}

////////////////////////////////////////////////////////////////////////////////
// Send a packet and wait for a reply from the IMU.
// Returns the number of bytes read.
int microstrain_3dmgx1_imu::IMU::transact(uint8_t *cmd, int cmd_len, uint8_t *rep, int rep_len, int timeout)
{
  send(cmd, cmd_len);

  return receive(*cmd, rep, rep_len, timeout);
}

////////////////////////////////////////////////////////////////////////////////
//
//
/**
 * @brief microstrain_3dmgx1_imu::IMU::receive Receive a reply from the IMU.
 * @param command command that correspond with the read data
 * @param rep data read
 * @param rep_len
 * @param timeout
 * @return Returns the number of bytes read.
 */
int microstrain_3dmgx1_imu::IMU::receive(uint8_t command, uint8_t *rep, int rep_len, int timeout)
{
  int nbytes, bytes, skippedbytes;
  skippedbytes = 0;

  memset(rep,0,rep_len);

  while (*rep != command && skippedbytes < MAX_BYTES_SKIPPED)
  {
    read_with_timeout(fd, rep, 1, timeout);
    skippedbytes++;
  }

  // We now have 1 byte
  bytes = 1;

  // Read the rest of the message:
  while (bytes < rep_len)
  {
    nbytes = read_with_timeout(fd, rep + bytes, rep_len - bytes, timeout);

    if (nbytes < 0)
      IMU_EXCEPT(microstrain_3dmgx1_imu::Exception, "read failed  [%s]", strerror(errno));

    bytes += nbytes;
  }


  uint16_t checksum = calcChecksum(rep,rep_len);
  // If wrong throw Exception
  if (checksum != convert2int(&rep[rep_len-2]))
    IMU_EXCEPT(microstrain_3dmgx1_imu::CorruptedDataException, "invalid checksum.\n Make sure the IMU sensor is connected to this computer.");


  return bytes;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::convert2int convert buffer of bytes in a integer value
 * @param buffer data to convert
 * @return integer value
 */
int microstrain_3dmgx1_imu::IMU::convert2int(uint8_t* buffer) {
  int x = (buffer[0]&LSB_MASK)*256 + (buffer[1]&LSB_MASK);
  return x;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::convert2short convert buffer of bytes in a short value
 * @param buffer data to convert
 * @return short value
 */
short microstrain_3dmgx1_imu::IMU::convert2short(uint8_t* buffer) {
  short x = (buffer[0]&LSB_MASK)*256 + (buffer[1]&LSB_MASK);
  return x;
}

/**
 * @brief microstrain_3dmgx1_imu::IMU::calcChecksum Calculate checksum on a received data buffer
 * @param buffer data to calculate the checksum
 * @param length len of the buffer
 * @return checksum calculated
 */
int microstrain_3dmgx1_imu::IMU::calcChecksum( uint8_t* buffer, int length) {
  int CHECKSUM_MASK = 0xFFFF;
  int checkSum, i;

  if (length<4)
    return -1;

  checkSum = buffer[0] & LSB_MASK;
  for (i=1; i<length-2; i = i+2) {
    checkSum += convert2int(&buffer[i]);
  }
  return(checkSum & CHECKSUM_MASK);
}

bool microstrain_3dmgx1_imu::IMU::getContinuous() const
{
  return continuous;
}

void microstrain_3dmgx1_imu::IMU::setFixed_offset(double newFixed_offset)
{
  fixed_offset = u_int64_t(newFixed_offset * 1e9);
}


