//
// Created by renzoscholman on 15-11-2016.
//

#include <bb8_sensor/9DOF/Gyro.h>

Gyro::Gyro(int32_t sensorID) {
    communication = new I2C(L3GD20_ADDRESS);
    _sensorID = sensorID;
    _autoRangeEnabled = false;
}

Gyro::~Gyro() {
    delete communication;
}

bool Gyro::begin(gyroRange_t rng) {
    /* Set the range the an appropriate value */
    _range = rng;

    /* Clear the raw sensor data */
    raw.x = 0;
    raw.y = 0;
    raw.z = 0;


    /* Make sure we have the correct chip ID since this checks
       for correct address and that the IC is properly connected */
    uint8_t id = read8(GYRO_REGISTER_WHO_AM_I);
    //Serial.println(id, HEX);
    if ((id != L3GD20_ID) && (id != L3GD20H_ID)) {
        return false;
    }

    /* Set CTRL_REG1 (0x20)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
     7-6  DR1/0     Output data rate                                   00
     5-4  BW1/0     Bandwidth selection                                00
       3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
       2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
       1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
       0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

    /* Reset then switch to normal mode and enable all three channels */
    communication->writeData(GYRO_REGISTER_CTRL_REG1, 0x00);
    communication->writeData(GYRO_REGISTER_CTRL_REG1, 0x0F);
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG2 (0x21)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
     5-4  HPM1/0    High-pass filter mode selection                    00
     3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG3 (0x22)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
       6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
       5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
       4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
       3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
       2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
       1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
       0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG4 (0x23)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
       6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
     5-4  FS1/0     Full scale selection                               00
                                    00 = 250 dps
                                    01 = 500 dps
                                    10 = 2000 dps
                                    11 = 2000 dps
       0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

    /* Adjust resolution if requested */
    switch (_range) {
        case GYRO_RANGE_250DPS:
            communication->writeData(GYRO_REGISTER_CTRL_REG4, 0x00);
            break;
        case GYRO_RANGE_500DPS:
            communication->writeData(GYRO_REGISTER_CTRL_REG4, 0x10);
            break;
        case GYRO_RANGE_2000DPS:
            communication->writeData(GYRO_REGISTER_CTRL_REG4, 0x20);
            break;
    }
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG5 (0x24)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
       6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
       4  HPen      High-pass filter enable (0=disable,1=enable)        0
     3-2  INT1_SEL  INT1 Selection config                              00
     1-0  OUT_SEL   Out selection config                               00 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    return true;
}

//void Gyro::calibrate(float *data){
//    int8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
//    uint16_t ii, packet_count;
//    int32_t gyro_bias[3] = {0, 0, 0};
//    packet_count = 80;// How many sets of full gyro and accelerometer data for averaging
//
//    for (ii = 0; ii < packet_count; ii++) {
//
//        sensors_event_t *event;
//
//        get_event(&event);
//
//        gyro_bias[1]  += (int32_t) event.gyro.x;
//        gyro_bias[2]  += (int32_t) event.gyro.y;
//        gyro_bias[3]  += (int32_t) event.gyro.z;
//
//    }
//    gyro_bias[0]  /= (int32_t) packet_count;
//    gyro_bias[1]  /= (int32_t) packet_count;
//    gyro_bias[2]  /= (int32_t) packet_count;
//
//// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
//    data[3] = (-gyro_bias[1]/4)       & 0xFF;
//    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
//    data[5] = (-gyro_bias[2]/4)       & 0xFF;
//
//// Push gyro biases to hardware registers
//    writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
//    writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
//    writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
//    writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
//    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
//    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);
//
//    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
//    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
//    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
//
//// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
//// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
//// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
//// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
//// the accelerometer biases calculated above must be divided by 8.
//
//    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//    readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//    readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//    readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//
//    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
//    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
//
//    for(ii = 0; ii < 3; ii++) {
//        if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
//    }
//
//    // Construct total accelerometer bias, including calculated average accelerometer bias from above
//    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//    accel_bias_reg[1] -= (accel_bias[1]/8);
//    accel_bias_reg[2] -= (accel_bias[2]/8);
//
//    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//    data[1] = (accel_bias_reg[0])      & 0xFF;
//    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//    data[3] = (accel_bias_reg[1])      & 0xFF;
//    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//    data[5] = (accel_bias_reg[2])      & 0xFF;
//    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//
//    // Push accelerometer biases to hardware registers
////  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
////  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
////  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
////  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
////  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
////  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);
//
//// Output scaled accelerometer biases for manual subtraction in the main program
//    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
//    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
//    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
//}

uint8_t Gyro::read8(uint8_t reg) {
    uint8_t value;
    communication->receive(reg, 1, &value);
    return value;
}

void Gyro::enableAutoRange(bool enabled) {
    _autoRangeEnabled = enabled;
}

bool Gyro::getEvent(sensors_event_t *event) {
    bool readingValid = false;

    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    /* Clear the raw data placeholder */
    raw.x = 0;
    raw.y = 0;
    raw.z = 0;

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_GYROSCOPE;

    while (!readingValid) {
        uint8_t block[6];

        // Get the data from the sensor
        communication->receive(0x80 | GYRO_REGISTER_OUT_X_L, sizeof(block), block);

        // Convert the data
        raw.x = (int16_t)(block[0] | (block[1] << 8)) >> 4;
        raw.y = (int16_t)(block[2] | (block[3] << 8)) >> 4;
        raw.z = (int16_t)(block[4] | (block[5] << 8)) >> 4;

        timeval now;
        gettimeofday(&now, 0);
        event->timestamp = now.tv_sec * 1000000 + now.tv_usec;

        /* Shift values to create properly formed integer (low byte first) */
        event->gyro.x = raw.x;
        event->gyro.y = raw.y;
        event->gyro.z = raw.z;

        /* Make sure the sensor isn't saturating if auto-ranging is enabled */
        if (!_autoRangeEnabled) {
            readingValid = true;
        } else {
            /* Check if the sensor is saturating or not */
            if ((event->gyro.x >= 32760) | (event->gyro.x <= -32760) |
                (event->gyro.y >= 32760) | (event->gyro.y <= -32760) |
                (event->gyro.z >= 32760) | (event->gyro.z <= -32760)) {
                /* Saturating .... increase the range if we can */
                switch (_range) {
                    case GYRO_RANGE_500DPS:
                        /* Push the range up to 2000dps */
                        _range = GYRO_RANGE_2000DPS;
                        communication->writeData(GYRO_REGISTER_CTRL_REG1, 0x00);
                        communication->writeData(GYRO_REGISTER_CTRL_REG1, 0x0F);
                        communication->writeData(GYRO_REGISTER_CTRL_REG4, 0x20);
                        communication->writeData(GYRO_REGISTER_CTRL_REG5, 0x80);
                        readingValid = false;
                        // Serial.println("Changing range to 2000DPS");
                        break;
                    case GYRO_RANGE_250DPS:
                        /* Push the range up to 500dps */
                        _range = GYRO_RANGE_500DPS;
                        communication->writeData(GYRO_REGISTER_CTRL_REG1, 0x00);
                        communication->writeData(GYRO_REGISTER_CTRL_REG1, 0x0F);
                        communication->writeData(GYRO_REGISTER_CTRL_REG4, 0x10);
                        communication->writeData(GYRO_REGISTER_CTRL_REG5, 0x80);
                        readingValid = false;
                        // Serial.println("Changing range to 500DPS");
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

    /* Compensate values depending on the resolution */
    switch (_range) {
        case GYRO_RANGE_250DPS:
            event->gyro.x *= GYRO_SENSITIVITY_250DPS;
            event->gyro.y *= GYRO_SENSITIVITY_250DPS;
            event->gyro.z *= GYRO_SENSITIVITY_250DPS;
            break;
        case GYRO_RANGE_500DPS:
            event->gyro.x *= GYRO_SENSITIVITY_500DPS;
            event->gyro.y *= GYRO_SENSITIVITY_500DPS;
            event->gyro.z *= GYRO_SENSITIVITY_500DPS;
            break;
        case GYRO_RANGE_2000DPS:
            event->gyro.x *= GYRO_SENSITIVITY_2000DPS;
            event->gyro.y *= GYRO_SENSITIVITY_2000DPS;
            event->gyro.z *= GYRO_SENSITIVITY_2000DPS;
            break;
    }

    /* Convert values to rad/s */
    event->gyro.x *= SENSORS_DPS_TO_RADS;
    event->gyro.y *= SENSORS_DPS_TO_RADS;
    event->gyro.z *= SENSORS_DPS_TO_RADS;

    return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Gyro::getSensor(sensor_t *sensor) {
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy(sensor->name, "L3GD20", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_GYROSCOPE;
    sensor->min_delay = 0;
    sensor->max_value = (float) this->_range * SENSORS_DPS_TO_RADS;
    sensor->min_value = (this->_range * -1.0) * SENSORS_DPS_TO_RADS;
    sensor->resolution = 0.0F; // TBD
}
