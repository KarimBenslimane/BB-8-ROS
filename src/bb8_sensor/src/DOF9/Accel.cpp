//
// Created by renzoscholman on 15-11-2016.
//

#include <bb8_sensor/9DOF/Accel.h>

static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb

Accel::Accel(int32_t sensorID) {
  communication = new I2C(LSM303_ADDRESS_ACCEL);
  _sensorID = sensorID;

  // Clear the raw mag data
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;
}

Accel::~Accel(){
  delete communication;
}

bool Accel::begin() {
  // Enable the accelerometer (100Hz)
  communication->writeData(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

  // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
  // if we are connected or not
  uint8_t reg1_a = read8(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
  if (reg1_a != 0x57) {
    return false;
  }

  return true;
}

uint8_t Accel::read8(uint8_t reg) {
  uint8_t value;
  communication->receive(reg, 1, &value);
  return value;
}

void Accel::read() {
  uint8_t block[6];

  // Get the data from the sensor
  communication->receive(0x80 | LSM303_REGISTER_ACCEL_OUT_X_L_A , sizeof(block), block);

  // Convert the data
  raw.x = (int16_t)(block[0] | (block[1] << 8)) >> 4;
  raw.y = (int16_t)(block[2] | (block[3] << 8)) >> 4;
  raw.z = (int16_t)(block[4] | (block[5] << 8)) >> 4;
}

bool Accel::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();


  timeval now;
  gettimeofday(&now, 0);

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = now.tv_sec * 1000000 + now.tv_usec;
  event->acceleration.x = (float)raw.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = (float)raw.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = (float)raw.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Accel::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F; // TBD
  sensor->min_value   = 0.0F; // TBD
  sensor->resolution  = 0.0F; // TBD
}
