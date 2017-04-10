//
// Created by renzoscholman on 2-11-2016.
//

#include <bb8_communications/i2c.h>

I2C::I2C(int address) {
    ROS_INFO("s%", "I2C Initialize...");
  // Initialize variables
  ready = false;
  current_address = -1;

  // Open the i2c bus
  if ((file_i2c = open(filename, O_RDWR)) < 0) {
#ifdef DEBUG
    ROS_INFO("Failed to open i2c device: %s\n", filename);
    ROS_INFO("File descriptor: %d\n", file_i2c);
    ROS_INFO("Got errno: %d\n", errno);
#endif
    // Failed to open i2c bus, return
    return;
  }

  // Set the address and initiate the connection
  setAddress(address);
}

I2C::I2C() {
  // Initialize variables
  ready = false;
  current_address = -1;

  // Open the i2c bus
  if ((file_i2c = open(filename, O_RDWR)) < 0) {
    #ifdef DEBUG
        ROS_INFO("Failed to open i2c device: %s\n", filename);
        ROS_INFO("File descriptor: %d\n", file_i2c);
        ROS_INFO("Got errno: %d\n", errno);
    #endif
    // Failed to open i2c bus, return
    return;
  }
}

I2C::~I2C() {
  close(file_i2c);
}

void I2C::setAddress(int address) {
  if (current_address != address) {
    init(address);
  }
}

int I2C::getAddress() {
  return current_address;
}

bool I2C::isReady() {
  return ready;
}

void I2C::init(int address) {
  ready = false;

  if (ioctl(file_i2c, I2C_SLAVE, address) < 0) {
    // Failed to get buss access and or talk to slave, set back to old status
#ifdef DEBUG
    printf("Failed to set address: %d\n", address);
    printf("Got errno: %d\n", errno);
#endif
    return;
  }
  current_address = address;

  ready = true;
}

bool I2C::send(unsigned short int data) {
  char *dataSend = new char[2];
  if (!ready) return false;

  dataSend[0] = (data >> 8) & 0xff;
  dataSend[1] = data & 0xff;
//#ifdef DEBUG
//#endif
  return write(file_i2c, dataSend, 2) == 2;
}

void I2C::readByte(uint8_t * value){
  // First check if we are ready
  if (!ready) return;

  *value = i2c_smbus_read_byte(file_i2c);
}

void I2C::writeByte(uint8_t value){
  // First check if we are ready
  if (!ready) return;

  i2c_smbus_write_byte(file_i2c, value);
}

void I2C::writeData(uint8_t reg, uint8_t value) {
  // First check if we are ready
  if (!ready) return;

  i2c_smbus_write_byte_data(file_i2c, reg, value);
}

void I2C::receive(uint8_t command, uint8_t size, uint8_t *data) {
  int result;
  // First check if we are ready
  if (!ready) return;

  // Next read the result
  result = i2c_smbus_read_i2c_block_data(file_i2c, command, size, data);

#ifdef DEBUG
  // Check if result was succesfull
  if (result != size) {
    printf("Failed to read block from I2C. Got size: %d requested size: %d\n", result, size);
  }
#endif
}
