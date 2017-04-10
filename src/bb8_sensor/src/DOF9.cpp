//
// Created by renzoscholman on 14-11-2016.
//

#include <bb8_sensor/DOF9.h>

DOF9::DOF9(void) {
    acc_sensor = new Accel(30301);
    mag_sensor = new Mag(30302);
    gyro_sensor = new Gyro(30303);
    filter = new Fusion();

    acc_sensor->begin();
    mag_sensor->begin();
    gyro_sensor->begin();

    displaySensorDetails();
}

void DOF9::displaySensorDetails(void) {
    sensor_t sensor;

    acc_sensor->getSensor(&sensor);
    printf("----------- ACCELEROMETER ----------\n");
    printf("Sensor:       %s\n", sensor.name);
    printf("Driver Ver:   %d\n", sensor.version);
    printf("Unique ID:    %d\n", sensor.sensor_id);
    printf("Max Value:    %f m/s^2\n", sensor.max_value);
    printf("Min Value:    %f m/s^2\n", sensor.min_value);
    printf("Resolution:   %f m/s^2\n", sensor.resolution);
    printf("------------------------------------\n");
    printf("\n");

    gyro_sensor->getSensor(&sensor);
    printf("----------- GYROSCOPE ----------\n");
    printf("Sensor:       %s\n", sensor.name);
    printf("Driver Ver:   %d\n", sensor.version);
    printf("Unique ID:    %d\n", sensor.sensor_id);
    printf("Max Value:    %f rad^2\n", sensor.max_value);
    printf("Min Value:    %f rad^2\n", sensor.min_value);
    printf("Resolution:   %f rad^2\n", sensor.resolution);
    printf("------------------------------------\n");
    printf("\n");

    mag_sensor->getSensor(&sensor);
    printf("----------- MAGNETOMETER ----------\n");
    printf("Sensor:       %s\n", sensor.name);
    printf("Driver Ver:   %d\n", sensor.version);
    printf("Unique ID:    %d\n", sensor.sensor_id);
    printf("Max Value:    %f uT\n", sensor.max_value);
    printf("Min Value:    %f uT\n", sensor.min_value);
    printf("Resolution:   %f uT\n", sensor.resolution);
    printf("------------------------------------\n");
    printf("\n");
}

void DOF9::update(void) {
    sensors_event_t data;
    sensors_vec_t orientation;

    /* Calculate pitch and roll from the raw accelerometer data */
    printf("\n");
    printf("\n");
    printf("Next iteration: \n");
    acc_sensor->getEvent(&data);
    mag_sensor->getEvent(&data);
    gyro_sensor->getEvent(&data);

    if (filter->update(&data)) {
        orientation.roll = filter->getRoll();
        orientation.pitch = filter->getPitch();
        orientation.heading = filter->getYaw();
        /* 'orientation' should have valid .roll and .pitch fields */
        printf("Roll:\t%f,\tPitch:\t%f,\tHeading:\t%f\n", orientation.roll, orientation.pitch, orientation.heading);
        printf("\n");
    }
}
