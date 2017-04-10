//
// Created by renzoscholman on 15-11-2016.
//

#ifndef RASPBERRYPI_GYRO_H
#define RASPBERRYPI_GYRO_H

#include <bb8_sensor/9DOF/DOF9_sensor.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
#define L3GD20_ADDRESS           (0x6B)        // 1101011
#define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
#define L3GD20_ID                (0xD4)
#define L3GD20H_ID               (0xD7)
#define GYRO_SENSITIVITY_250DPS  (0.00875F)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS  (0.0175F)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS (0.070F)      // Roughly 18/256
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
typedef enum{                                             // DEFAULT    TYPE
    GYRO_REGISTER_WHO_AM_I = 0x0F,   // 11010100   r
    GYRO_REGISTER_CTRL_REG1 = 0x20,   // 00000111   rw
    GYRO_REGISTER_CTRL_REG2 = 0x21,   // 00000000   rw
    GYRO_REGISTER_CTRL_REG3 = 0x22,   // 00000000   rw
    GYRO_REGISTER_CTRL_REG4 = 0x23,   // 00000000   rw
    GYRO_REGISTER_CTRL_REG5 = 0x24,   // 00000000   rw
    GYRO_REGISTER_REFERENCE = 0x25,   // 00000000   rw
    GYRO_REGISTER_OUT_TEMP = 0x26,   //            r
    GYRO_REGISTER_STATUS_REG = 0x27,   //            r
    GYRO_REGISTER_OUT_X_L = 0x28,   //            r
    GYRO_REGISTER_OUT_X_H = 0x29,   //            r
    GYRO_REGISTER_OUT_Y_L = 0x2A,   //            r
    GYRO_REGISTER_OUT_Y_H = 0x2B,   //            r
    GYRO_REGISTER_OUT_Z_L = 0x2C,   //            r
    GYRO_REGISTER_OUT_Z_H = 0x2D,   //            r
    GYRO_REGISTER_FIFO_CTRL_REG = 0x2E,   // 00000000   rw
    GYRO_REGISTER_FIFO_SRC_REG = 0x2F,   //            r
    GYRO_REGISTER_INT1_CFG = 0x30,   // 00000000   rw
    GYRO_REGISTER_INT1_SRC = 0x31,   //            r
    GYRO_REGISTER_TSH_XH = 0x32,   // 00000000   rw
    GYRO_REGISTER_TSH_XL = 0x33,   // 00000000   rw
    GYRO_REGISTER_TSH_YH = 0x34,   // 00000000   rw
    GYRO_REGISTER_TSH_YL = 0x35,   // 00000000   rw
    GYRO_REGISTER_TSH_ZH = 0x36,   // 00000000   rw
    GYRO_REGISTER_TSH_ZL = 0x37,   // 00000000   rw
    GYRO_REGISTER_INT1_DURATION = 0x38    // 00000000   rw
} gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
typedef enum{
    GYRO_RANGE_250DPS = 250,
    GYRO_RANGE_500DPS = 500,
    GYRO_RANGE_2000DPS = 2000
} gyroRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
typedef struct gyroRawData_s{
    int16_t x;
    int16_t y;
    int16_t z;
} gyroRawData_t;

/*=========================================================================*/

class Gyro : public Adafruit_Sensor{
    public:
        Gyro(int32_t sensorID = -1);
        ~Gyro();
        bool begin(gyroRange_t rng = GYRO_RANGE_250DPS);
        void enableAutoRange(bool enabled);
        bool getEvent(sensors_event_t *);
        void getSensor(sensor_t *);
//        void calibrate(float* data);
        gyroRawData_t raw; /* Raw values from last sensor read */
    private:
        uint8_t read8(uint8_t reg);
        gyroRange_t _range;
        int32_t _sensorID;
        bool _autoRangeEnabled;
};

#endif //RASPBERRYPI_GYRO_H
