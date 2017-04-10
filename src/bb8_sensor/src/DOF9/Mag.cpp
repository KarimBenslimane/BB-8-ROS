//
// Created by renzoscholman on 15-11-2016.
//

#include <bb8_sensor/9DOF/Mag.h>

static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

uint8_t Mag::read8(uint8_t reg) {
  uint8_t value;
  communication->receive(reg, 1, &value);
  return value;
}

void Mag::read() {
  uint8_t block[6];

  // DLHC: register address order is X,Z,Y with high bytes first
  communication->receive(0x80 | LSM303_REGISTER_MAG_OUT_X_H_M, sizeof(block), block);

  raw.x = (int16_t)(block[1] | block[0] << 8);
  raw.y = (int16_t)(block[5] | block[4] << 8);
  raw.z = (int16_t)(block[3] | block[2] << 8);
}

Mag::Mag(int32_t sensorID) {
  communication = new I2C(LSM303_ADDRESS_MAG);

  _sensorID = sensorID;
  autoRangeEnabled = false;

  // Clear the raw mag data
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;
}

Mag::~Mag(){
  delete communication;
}

bool Mag::begin() {
  // Enable the magnetometer
  communication->writeData(LSM303_REGISTER_MAG_MR_REG_M, 0x00);

  // LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
  // the default value (0b00010000/0x10)
  uint8_t reg1_a = read8(LSM303_REGISTER_MAG_CRA_REG_M);
  if (reg1_a != 0x10) {
    return false;
  }

  // Set the gain to a known level
  setMagGain(LSM303_MAGGAIN_1_3);

  return true;
}

/**************************************************************************/
/*!
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void Mag::enableAutoRange(bool enabled) {
  autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void Mag::setMagGain(lsm303MagGain gain) {
  communication->writeData(LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t) gain);

  magGain = gain;

  switch (gain) {
    case LSM303_MAGGAIN_1_3:
      _lsm303Mag_Gauss_LSB_XY = 1100;
          _lsm303Mag_Gauss_LSB_Z = 980;
          break;
    case LSM303_MAGGAIN_1_9:
      _lsm303Mag_Gauss_LSB_XY = 855;
          _lsm303Mag_Gauss_LSB_Z = 760;
          break;
    case LSM303_MAGGAIN_2_5:
      _lsm303Mag_Gauss_LSB_XY = 670;
          _lsm303Mag_Gauss_LSB_Z = 600;
          break;
    case LSM303_MAGGAIN_4_0:
      _lsm303Mag_Gauss_LSB_XY = 450;
          _lsm303Mag_Gauss_LSB_Z = 400;
          break;
    case LSM303_MAGGAIN_4_7:
      _lsm303Mag_Gauss_LSB_XY = 400;
          _lsm303Mag_Gauss_LSB_Z = 355;
          break;
    case LSM303_MAGGAIN_5_6:
      _lsm303Mag_Gauss_LSB_XY = 330;
          _lsm303Mag_Gauss_LSB_Z = 295;
          break;
    case LSM303_MAGGAIN_8_1:
      _lsm303Mag_Gauss_LSB_XY = 230;
          _lsm303Mag_Gauss_LSB_Z = 205;
          break;
  }
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's update rate
*/
/**************************************************************************/
void Mag::setMagRate(lsm303MagRate rate) {
  uint8_t reg_m = ((uint8_t) rate & 0x07) << 2;
  communication->writeData(LSM303_REGISTER_MAG_CRA_REG_M, (uint8_t) reg_m);
}


/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Mag::getEvent(sensors_event_t *event) {
  bool readingValid = false;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  while (!readingValid) {

    uint8_t reg_mg = read8(LSM303_REGISTER_MAG_SR_REG_Mg);
    if (!(reg_mg & 0x1)) {
      return false;
    }

    /* Read new data */
    read();

    /* Make sure the sensor isn't saturating if auto-ranging is enabled */
    if (!autoRangeEnabled) {
      readingValid = true;
    } else {
      /* Check if the sensor is saturating or not */
      if ((raw.x >= 2040) | (raw.x <= -2040) |
          (raw.y >= 2040) | (raw.y <= -2040) |
          (raw.z >= 2040) | (raw.z <= -2040)) {
        /* Saturating .... increase the range if we can */
        switch (magGain) {
          case LSM303_MAGGAIN_5_6:
            setMagGain(LSM303_MAGGAIN_8_1);
                readingValid = false;
                break;
          case LSM303_MAGGAIN_4_7:
            setMagGain(LSM303_MAGGAIN_5_6);
                readingValid = false;
                break;
          case LSM303_MAGGAIN_4_0:
            setMagGain(LSM303_MAGGAIN_4_7);
                readingValid = false;
                break;
          case LSM303_MAGGAIN_2_5:
            setMagGain(LSM303_MAGGAIN_4_0);
                readingValid = false;
                break;
          case LSM303_MAGGAIN_1_9:
            setMagGain(LSM303_MAGGAIN_2_5);
                readingValid = false;
                break;
          case LSM303_MAGGAIN_1_3:
            setMagGain(LSM303_MAGGAIN_1_9);
                readingValid = false;
                break;
          default:
            readingValid = true;
                break;
        }
      } else {
        /* All values are withing range */
        readingValid = true;
      }
    }
  }

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  timeval now;
  gettimeofday(&now, 0);
  event->timestamp = now.tv_sec * 1000000 + now.tv_usec;
  event->magnetic.x = (float) raw.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y = (float) raw.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z = (float) raw.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Mag::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->max_value = 0.0F; // TBD
  sensor->min_value = 0.0F; // TBD
  sensor->resolution = 0.0F; // TBD
}
