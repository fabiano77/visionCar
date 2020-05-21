//#include <unistd.h>
//#include <fcntl.h>
//#include <cstring>
//#include <iostream>
//#include <sys/ioctl.h>
//#include <linux/i2c-dev.h>
//#include <wiringPiI2C.h>
//#include <cmath>
//#include "PCA9685.h"
////#include "Constants.h"
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
//#include <errno.h>
//#include <string.h>
//#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <asm/ioctl.h>
//
//#include "wiringPi.h"
//#include "wiringPiI2C.h"
//// I2C definitions
//
//#define I2C_SLAVE	0x0703
//#define I2C_SMBUS	0x0720	/* SMBus-level access */
//
//#define I2C_SMBUS_READ	1
//#define I2C_SMBUS_WRITE	0
//
//// SMBus transaction types
//
//#define I2C_SMBUS_QUICK		    0
//#define I2C_SMBUS_BYTE		    1
//#define I2C_SMBUS_BYTE_DATA	    2 
//#define I2C_SMBUS_WORD_DATA	    3
//#define I2C_SMBUS_PROC_CALL	    4
//#define I2C_SMBUS_BLOCK_DATA	    5
//#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
//#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
//#define I2C_SMBUS_I2C_BLOCK_DATA    8
//
//// SMBus messages
//
//#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */	
//#define I2C_SMBUS_I2C_BLOCK_MAX	32	/* Not specified but we use same structure */
//
//// Structures used in the ioctl() calls
//
//union i2c_smbus_data
//{
//  uint8_t  byte ;
//  uint16_t word ;
//  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2] ;	// block [0] is used for length + one more for PEC
//} ;
///*
//struct i2c_smbus_ioctl_data
//{
//  char read_write ;
//  uint8_t command ;
//  int size ;
//  union i2c_smbus_data *data ;
//} ;
//* */
//
//static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
//{
//  struct i2c_smbus_ioctl_data args ;
//
//  args.read_write = rw ;
//  args.command    = command ;
//  args.size       = size ;
//  args.data       = data ;
//  return ioctl (fd, I2C_SMBUS, &args) ;
//}
//
//
///*
// * wiringPiI2CRead:
// *	Simple device read
// *********************************************************************************
// */
//
//int wiringPiI2CRead (int fd)
//{
//  union i2c_smbus_data data ;
//
//  if (i2c_smbus_access (fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data))
//    return -1 ;
//  else
//    return data.byte & 0xFF ;
//}
//
//
///*
// * wiringPiI2CReadReg8: wiringPiI2CReadReg16:
// *	Read an 8 or 16-bit value from a regsiter on the device
// *********************************************************************************
// */
//
//int wiringPiI2CReadReg8 (int fd, int reg)
//{
//  union i2c_smbus_data data;
//
//  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data))
//    return -1 ;
//  else
//    return data.byte & 0xFF ;
//}
//
//int wiringPiI2CReadReg16 (int fd, int reg)
//{
//  union i2c_smbus_data data;
//
//  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data))
//    return -1 ;
//  else
//    return data.word & 0xFFFF ;
//}
//
//
///*
// * wiringPiI2CWrite:
// *	Simple device write
// *********************************************************************************
// */
//
//int wiringPiI2CWrite (int fd, int data)
//{
//  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, data, I2C_SMBUS_BYTE, NULL) ;
//}
//
//
///*
// * wiringPiI2CWriteReg8: wiringPiI2CWriteReg16:
// *	Write an 8 or 16-bit value to the given register
// *********************************************************************************
// */
//
//int wiringPiI2CWriteReg8 (int fd, int reg, int value)
//{
//  union i2c_smbus_data data ;
//
//  data.byte = value ;
//  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data) ;
//}
//
//int wiringPiI2CWriteReg16 (int fd, int reg, int value)
//{
//  union i2c_smbus_data data ;
//
//  data.word = value ;
//  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data) ;
//}
//
//
///*
// * wiringPiI2CSetupInterface:
// *	Undocumented access to set the interface explicitly - might be used
// *	for the Pi's 2nd I2C interface...
// *********************************************************************************
// */
//
//int wiringPiI2CSetupInterface (const char *device, int devId)
//{
//  int fd ;
//
////  if ((fd = open (device, O_RDWR)) < 0)
// //   return wiringPiFailure (WPI_ALMOST, "Unable to open I2C device: %s\n", strerror (errno)) ;
//
////  if (ioctl (fd, I2C_SLAVE, devId) < 0)
//    //return wiringPiFailure (WPI_ALMOST, "Unable to select I2C device: %s\n", strerror (errno)) ;
//
//  return fd ;
//}
//
//
///*
// * wiringPiI2CSetup:
// *	Open the I2C device, and regsiter the target device
// *********************************************************************************
// */
//
//int wiringPiI2CSetup (const int devId)
//{
//  int rev ;
//  const char *device ;
//    device = "/dev/i2c-1" ;
//  return wiringPiI2CSetupInterface (device, devId) ;
//}
//
//
//PCA9685::PCA9685(const std::string &device, int address) {
//  if((i2c_fd = open(device.c_str(), O_RDWR)) < 0) {
//    std::cout << "open: " << std::strerror(errno) << "\n";
//    exit(1);
//  }
//  int addr = address;
//  if(ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
//    std::cout << "ioctl: " << std::strerror(errno) << "\n";
//    exit(1);
//  }
//  set_all_pwm(0,0);
//  auto ret = wiringPiI2CWriteReg8(i2c_fd, MODE2, OUTDRV);
//  check_ret(ret, "set mode2");
//  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, ALLCALL);
//  check_ret(ret, "set mode1");
//  usleep(5'000);
//  auto mode1 = wiringPiI2CReadReg8(i2c_fd, MODE1);
//  check_ret(mode1, "read mode1");
//  mode1 = mode1 & ~SLEEP;
//  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, mode1);
//  check_ret(ret, "write mode1");
//  usleep(5'000);
//}
//
//void PCA9685::set_pwm_freq(double freq_hz) {
//  frequency = freq_hz;
//
//  auto prescaleval = 25000000.0; //    # 25MHz
//  prescaleval /= 4096.0; //       # 12-bit
//  prescaleval /= freq_hz;
//  prescaleval -= 1.0;
//
//  auto prescale = static_cast<int>(std::round(prescaleval));
//
//  auto oldmode = wiringPiI2CReadReg8(i2c_fd, MODE1);
//  check_ret(oldmode);
//
//  auto newmode = (oldmode & 0x7F) | SLEEP;
//  auto ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, newmode);
//  check_ret(ret);
//  ret = wiringPiI2CWriteReg8(i2c_fd, PRESCALE, prescale);
//  check_ret(ret);
//  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, oldmode);
//  check_ret(ret);
//  usleep(5'000);
//  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, oldmode | RESTART);
//  check_ret(ret);
//}
//
//void PCA9685::set_pwm(int channel, uint16_t on, uint16_t off) {
//  wiringPiI2CWriteReg8(i2c_fd, LED0_ON_L+4*channel, on & 0xFF);
//  wiringPiI2CWriteReg8(i2c_fd, LED0_ON_H+4*channel, on >> 8);
//  wiringPiI2CWriteReg8(i2c_fd, LED0_OFF_L+4*channel, off & 0xFF);
//  wiringPiI2CWriteReg8(i2c_fd, LED0_OFF_H+4*channel, off >> 8);
//}
//
//void PCA9685::set_all_pwm(uint16_t on, uint16_t off) {
//  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_ON_L, on & 0xFF);
//  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_ON_H, on >> 8);
//  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_OFF_L, off & 0xFF);
//  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_OFF_H, off >> 8);
//}
//
//void PCA9685::set_pwm_ms(int channel, double ms) {
//  auto period_ms = 1000.0 / frequency;
//  auto bits_per_ms = 4096 / period_ms;
//  auto bits = ms * bits_per_ms;
//  set_pwm(channel, 0, bits);
//}
//
//void PCA9685::check_ret(int ret, std::string msg) {
//  if(ret < 0) {
//    std::cerr << "ERROR" << std::endl;
//    std::cerr << std::strerror(errno) << std::endl;
//    std::cerr << msg << std::endl;
//    exit(1);
//  }
//}
//
