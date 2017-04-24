//
// Created by renzoscholman on 15-11-2016.
//


#include <bb8_sensor/9DOF/DOF9_sensor.h>

Adafruit_Sensor::Adafruit_Sensor() {
  _autoRange = false;
}

Adafruit_Sensor::~Adafruit_Sensor(){
  // Silence is golden
}

// These must be defined by the subclass
void Adafruit_Sensor::enableAutoRange(bool enabled){
  _autoRange = enabled;
};
