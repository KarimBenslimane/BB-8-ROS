//
// Created by renzoscholman on 14-11-2016.
//

#ifndef RASPBERRYPI_9DOF_H
#define RASPBERRYPI_9DOF_H

#include <math.h>
#include <stdint.h>
#include <bb8_sensor/9DOF/Accel.h>
#include <bb8_sensor/9DOF/Gyro.h>
#include <bb8_sensor/9DOF/Mag.h>
#include <bb8_sensor/9DOF/DOF9_sensor.h>
#include <bb8_sensor/Filter/Fusion.h>

/** Sensor axis */
typedef enum {
    SENSOR_AXIS_X = (1),
    SENSOR_AXIS_Y = (2),
    SENSOR_AXIS_Z = (3)
} sensors_axis_t;

class DOF9 {
    public:
        DOF9();
        ~DOF9();
        void update();
        void displaySensorDetails();
    private:
        Fusion *filter;
        Mag *mag_sensor;
        Accel *acc_sensor;
        Gyro *gyro_sensor;
};

#endif //RASPBERRYPI_9DOF_H
