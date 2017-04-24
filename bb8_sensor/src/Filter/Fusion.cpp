//
// Created by karim on 3-2-17.
//

#include <bb8_sensor/Filter/Fusion.h>

Fusion::Fusion(){
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
}

bool Fusion::update(sensors_event_t *data){
    float _roll, _yaw, _pitch;
    /* Make sure the input is valid, not null, etc. */
    if (data == NULL) return false;

    float const PI_F = 3.14159265F;

    /* roll: Rotation around the X-axis. -180 <= roll <= 180                                          */
    /* a positive roll angle is defined to be a clockwise rotation about the positive X-axis          */
    /*                                                                                                */
    /*                    y                                                                           */
    /*      roll = atan2(---)                                                                         */
    /*                    z                                                                           */
    /*                                                                                                */
    /* where:  y, z are returned value from accelerometer sensor                                      */
    _roll = (float) atan2(data->acceleration.y, data->acceleration.z);

    /* pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         */
    /* a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         */
    /*                                                                                                */
    /*                                 -x                                                             */
    /*      pitch = atan(-------------------------------)                                             */
    /*                    y * sin(roll) + z * cos(roll)                                               */
    /*                                                                                                */
    /* where:  x, y, z are returned value from accelerometer sensor                                   */
    if (data->acceleration.y * sin(_roll) + data->acceleration.z * cos(_roll) == 0){
        _pitch = data->acceleration.x > 0 ? (PI_F / 2) : (-PI_F / 2);
    } else {
        _pitch = (float) atan(-data->acceleration.x / (data->acceleration.y * sin(_roll) + data->acceleration.z * cos(_roll)));
    }

    /* heading: Rotation around the Z-axis. -180 <= roll <= 180                                       */
    /* a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       */
    /*                                                                                                */
    /*                                       z * sin(roll) - y * cos(roll)                            */
    /*   heading = atan2(--------------------------------------------------------------------------)  */
    /*                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   */
    /*                                                                                                */
    /* where:  x, y, z are returned value from magnetometer sensor                                    */
    _yaw = (float) atan2(data->magnetic.z * sin(_roll) - data->magnetic.y * cos(_roll),
                         data->magnetic.x * cos(_pitch) + data->magnetic.y * sin(_pitch) * sin(_roll) + data->magnetic.z * sin(_pitch) * cos(_roll));

    /* Convert angular data to degree */
    roll = _roll * 180 / PI_F;
    pitch = _pitch * 180 / PI_F;
    yaw = _yaw * 180 / PI_F;

    return true;
};

float Fusion::getRoll(){
    return roll;
}

float Fusion::getPitch(){
    return pitch;
}

float Fusion::getYaw(){
    return yaw;
}
