//
// Created by renzoscholman on 15-11-2016.
//

#ifndef RASPBERRYPI_MAG_H
#define RASPBERRYPI_MAG_H

#include <bb8_sensor/9DOF/DOF9_sensor.h>

#define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x

typedef enum
{
  LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
  LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
  LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
  LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
  LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
  LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
  LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
  LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
  LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
  LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
  LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
  LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
  LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
  LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
  LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
} lsm303MagRegisters_t;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
typedef enum
{
  LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
  LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
  LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
  LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
  LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
  LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
  LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} lsm303MagGain;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER UPDATE RATE SETTINGS
    -----------------------------------------------------------------------*/
typedef enum
{
  LSM303_MAGRATE_0_7                        = 0x00,  // 0.75 Hz
  LSM303_MAGRATE_1_5                        = 0x01,  // 1.5 Hz
  LSM303_MAGRATE_3_0                        = 0x62,  // 3.0 Hz
  LSM303_MAGRATE_7_5                        = 0x03,  // 7.5 Hz
  LSM303_MAGRATE_15                         = 0x04,  // 15 Hz
  LSM303_MAGRATE_30                         = 0x05,  // 30 Hz
  LSM303_MAGRATE_75                         = 0x06,  // 75 Hz
  LSM303_MAGRATE_220                        = 0x07   // 200 Hz
} lsm303MagRate;
/*=========================================================================*/

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct lsm303MagData_s
{
  int16_t x;
  int16_t y;
  int16_t z;
} lsm303MagData;

/* Unified sensor driver for the magnetometer */
class Mag : public Adafruit_Sensor
{
 public:
  Mag(int32_t sensorID = -1);
  ~Mag();

  bool begin(void);
  void enableAutoRange(bool enable);
  void setMagGain(lsm303MagGain gain);
  void setMagRate(lsm303MagRate rate);
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

  lsm303MagData   raw;     // Last read magnetometer data will be available here
  lsm303MagGain   magGain;
  bool            autoRangeEnabled;

 private:
  int32_t         _sensorID;

  uint8_t read8(uint8_t reg);
  void read(void);
};

#endif //RASPBERRYPI_MAG_H
