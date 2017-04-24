//
// Created by renzoscholman on 2-11-2016.
//

#ifndef RASPBERRYPI_I2C_H
#define RASPBERRYPI_I2C_H
//#define DEBUG

//Needed for I2C port
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <bb8_communications/i2c-dev.h>
#include "ros/ros.h"

// Error handling
#include <errno.h>

// Debugging purposes
#include <stdio.h>

class I2C {
 public:
  I2C(int address);
  I2C();
  ~I2C();
  void setAddress(int address);
  int getAddress();
  bool isReady();
  bool send(unsigned short int data);
  void writeData(uint8_t reg, uint8_t value);
  void writeByte(uint8_t value);
  void readByte(uint8_t *value);
  void receive(uint8_t size, uint8_t command, uint8_t *data);
 private:
  void init(int address);
  int file_i2c;
  int length;
  int current_address;
  char *filename = (char*)"/dev/i2c-1";
  bool ready;
};

#endif //RASPBERRYPI_I2C_H
