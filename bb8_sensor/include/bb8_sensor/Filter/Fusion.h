//
// Created by karim on 3-2-17.
//

#ifndef BB_8_FUSION_H
#define BB_8_FUSION_H

#include <math.h>
#include <stdint.h>
#include <bb8_sensor/9DOF/DOF9_sensor.h>

class Fusion {
    public:
        Fusion();
        ~Fusion();
        bool update(sensors_event_t *data);
        float getPitch();
        float getRoll();
        float getYaw();
    private:
        float pitch, roll, yaw;
};

#endif //BB_8_FUSION_H
