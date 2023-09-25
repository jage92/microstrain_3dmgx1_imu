/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-2010  Willow Garage
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


#ifndef MS_3DMGX1_HH
#define MS_3DMGX1_HH

#include <fstream>
#include <stdexcept>
#include <stdint.h>
#include <string.h>
#include <string>
#include <iostream>

#define LSB_MASK 0xFF
#define MSB_MASK 0xFF00

//! Maximum bytes allowed to be skipped when seeking a message
#define MAX_BYTES_SKIPPED  1000
#define COMMON_SCALE_CONSTANT 32768000.0
#define GYRO_SCALE_CONSTANT 8192.0
namespace microstrain_3dmgx1_imu
{

  //! Macro for defining exception (std::runtime_error should be top parent)
  #define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }

  DEF_EXCEPTION(Exception, std::runtime_error);
  DEF_EXCEPTION(TimeoutException, Exception);
  DEF_EXCEPTION(CorruptedDataException, Exception);

  #undef DEF_EXCEPTION

  //! A class for interfacing to the microstrain 3dmgx1 and inertialink IMUs
  /*!
   * Note: This class is unreviewed and unsupported. It may change at any
   * time without notice.
   *
   * Many of the methods within this class may throw an
   * microstrain_3dmgx1_imu::exception, timeout_exception, or
   * corrupted_data_exception.
   *
   * Before using the IMU, it must be opened via the open_port method.
   * When finished using, it should be closed via the close_port
   * method.  Alternatively, close port will get called at
   * destruction.
   *
   * The library is primarily designed to be used in continuous mode,
   * which is enabled with the set_continuous method, and then serviced
   * with one of the receive methods.
   *
   * Implementation of specific polled message transactions can be
   * done with the transact method.
   *
   * Because the timing related to the USB stack is fairly
   * non-deterministic, but the IMU is internally known to be clocked
   * to a 100hz clock, we have wrapped a Kalman filter around calls to
   * get system time, and the internal imu time.  This is only known
   * to be reliable when operating in continuous mode, and if init_time
   * is called shortly prior to beginning to get readings.
   * 
   *
   * Example code:
   * \code
   *   microstrain_3dmgx1_imu::IMU imu;
   *   imu.open_port("/dev/ttyUSB0");
   *   imu.init_time();
   *   imu.init_gyros();
   *   imu.set_continuous(microstrain_3dmgx1_imu::IMU::CMD_ACCEL_ANGRATE_ORIENT);
   *   while (int i = 0 ; i < 100; i++)
   *   {
   *     double accel[3];
   *     double angrate[3];
   *     double orientation[9];
   *     imu.receive_accel_angrate_orientation(&time, accel, angrate, orientation);
   *   }
   *   imu.close_port();
   * \endcode
   */
  class IMU
  {
// from C++11, constexpr specifier is requred for double
//#if __cplusplus > 199711L
//#define const constexpr
//#endif
  private:
    //! The file descriptor
    int fd;


    //! Whether continuous mode is enabled
    bool continuous;

    //! Values to scale the IMU read data
    int gyroGainScale;
    int magGainScale;
    int accelGainScale;

    //! The last number of ticks for computing wraparound
    uint16_t last_ticks;

    double fixed_offset;

    //! The time at which the imu was started
    unsigned long long start_time;

    int convert2int(uint8_t* buffer);
    short convert2short(uint8_t* buffer);
    int calcChecksum( uint8_t* buffer, int length);
    char * explainError(int errornum);

    //! Send a command to the IMU and wait for a reply
    int transact(uint8_t *cmd, int cmd_len, uint8_t *rep, int rep_len, int timeout = 0);


    //! Receive a particular message from the IMU
    int receive(uint8_t command, uint8_t *rep, int rep_len, int timeout = 0);
    int read_with_timeout(int fd, uint8_t *buff, size_t count, int timeout);


//#if __cplusplus > 199711L
//#undef const
//#endif
public:
    //! Enumeration of possible IMU commands
    enum cmd {
      CMD_RAW_SENSOR =                        0x01,
      CMD_GYRO_VECTOR =                       0x02,
      CMD_INSTANT_VECTOR =                    0x03,
      CMD_INSTANT_QUAT =                      0x04,
      CMD_GYRO_QUAT =                         0x05,
      CMD_CAPTURE_GYRO_BIAS =                 0x06,
      CMD_TEMPERATURE =                       0x07,
      CMD_SEND_EEPROM =                       0x28,
      CMD_PROG_EEPROM =                       0x29,
      CMD_INSTANT_OR_MATRIX =                 0x0A,
      CMD_GYRO_OR_MATRIX =                    0x0B,
      CMD_GYRO_QUAT_VECTOR =                  0x0C,
      CMD_INSTANT_EULER =                     0x0D,
      CMD_GYRO_EULER =                        0x0E,
      CMD_SET_CONTINUOUS =                    0x10,
      CMD_GYRO_QUAT_INSTANT_VECTOR =          0x12,
      CMD_FIRWARE_VERSION =                   0xF0,
      CMD_SERIAL_NUMBER =                     0xF1,

      CMD_INITIALIZE_HARD_IRON_CALIBRATION =  0x40,
      CMD_COLLECT_HARD_IRON_DATA =            0x41,
      CMD_COMPUTE_HARD_IRON_DATA =            0x42,

      CMD_INSTANT_GYRO_QUAT_VECTOR =          0xDD, //NOT exists in the IMU, but it is neccesary to be instant quaternion and vectors together

      M3D_GYROSCALE_ADDRESS =                 130,
      M3D_ACCELSCALE_ADDRESS =                230,
      M3D_MAGSCALE_ADDRESS =                  232,

      M3D_CALC_CYCLE_1_ADDRESS =              238,
      M3D_CALC_CYCLE_2_ADDRESS =              240,
      M3D_CALC_CYCLE_3_ADDRESS =              242,
      M3D_CALC_CYCLE_4_ADDRESS =              246,
      NONE =                                  0x00
    };

    //! Constructor
    IMU();

    // Destructor
    ~IMU();

    //! Open the port
    /*! 
     * This must be done before the imu can be used.
     * 
     * \param port_name   A character array containing the name of the port
     *
     */
    void openPort(const char *port_name);

    //! Close the port
    void closePort();

    /*--------------------------------------------------------------------------
     * Sensor communication function prototypes.
     *--------------------------------------------------------------------------*/

    int mapDevice(int, int);
    int sendCommand(int deviceNum, char command, char *response, int responseLength);

    /*--------------------------------------------------------------------------
     * Sensor Function prototypes
     *
     * The following abbreviations are used to shorten
     * function names:
     *                Instant = Instantaneous
     *                Orient  = Orientation
     *                Stab    = Stabilized
     *                Quat    = Quaternion
     *--------------------------------------------------------------------------*/

    void initGyroScale(int);
    void getVectors(uint64_t *time, double *mag, double *accel, double *angRate, bool stableOption);

    void getOrientMatrix(uint64_t *time, double **mx, int stableOption);
    void getEulerAngles(uint64_t *time, double *pitch, double *roll, double *yaw, bool stableOption);

    void captureGyroBias();

    void getEEPROMValue(uint8_t address, short *value);
    void setEEPROMValue(uint8_t address, short value);

    void getGyroStabQuatVectors(uint64_t *time, double *q, double *mag, double *accel, double *angRate,bool stableOption);

    void getScales();

    void getSerialNumber(int *serialNum);
    void getFirmwareVersion(std::string *firmware);
    void getTemperature(double *temp);
    void getRawSensorOutput(uint64_t *time, double *mag, double *accel, double *angRate);
    void getQuaternions(uint64_t *time, double *q, int stableOption);
    void stopContinuous();

    void setContinuous(cmd command);

    void initializeMagCalibration();
    void collectHardIronData(uint64_t *time, double *mag, double *max, double *min);
    void computeHardIronData(uint8_t calibration2D,int magnitudeZ, uint64_t *time, double *offset);

    //! Send a single packet frmo the IMU
    int send(uint8_t *cmd, int cmd_len);

    /*-------------- end of m3dmgAdapter.h ----------------------*/
    bool getContinuous() const;
    uint64_t extractTime(uint8_t* addr);
    void initTime(double fix_off);
    void configureCalculationCycle();

    void setFixed_offset(double newFixed_offset);
  };

}
#endif
