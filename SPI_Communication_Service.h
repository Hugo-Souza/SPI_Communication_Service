#ifndef SPI_COMMUNICATION_SERVICE_H_INCLUDED
#define SPI_COMMUNICATION_SERVICE_H_INCLUDED

#include "Bus_Device_Service.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include "Bus_Device_Service.h"

#define SPI_PATH "/dev/spidev"
#define HEX(x) setw(2) << setfill('0') << hex << (int)(x)
using namespace std;


class SPI_Communication
{
public:
    //  The SPI Mode 
    enum SPI_mode
    {
        MODE0 = 0, // Low at idle, capture on rising clock edge
        MODE1 = 1, // Low at idle, capture on falling clock edge
        MODE2 = 2, // High at idle, capture on falling clock edge
        MODE3 = 3  // High at idle, capture on rising clock edge 
    };

    // Constructor
    SPI_Communication(unsigned int channel, unsigned int device, SPI_Communication::SPI_mode operationMode, uint8_t bitsPerWord, uint32_t communicationSpeed, uint16_t communicationDelay);

    // Destructor
    ~SPI_Communication();

    // Open file connection of SPI device
    virtual int open_connection();

    // Close file connection of SPI device
    virtual void close_connection();

    // Select the device to enable communication
    virtual void enable_device_communication(unsigned int devicePinSelector);

    // Select the device to disable communication
    virtual void disable_device_communication(unsigned int devicePinSelector);

    // Communication of read/write message to spi device
    virtual int transfer_message(unsigned char sendMessage[], unsigned char receiveMessage[], unsigned int lengthMessage);

    // Read a char from register containing the message from spi device
    virtual unsigned char read_register_device(unsigned int registerAddress);
	
    // Read a sequence of char from register containing the message from spi device
    virtual unsigned char* read_registers_device(unsigned int lengthData, unsigned int fromAddress=0);
    
    // Write a char to the addressed register of spi device
    virtual int write_register_device(unsigned int registerAddress, unsigned char data);

    // Write a char to the spi device
    virtual int write_device(unsigned char data);

    // Write the sequence of char to the spi device
    virtual int write_device(unsigned char data[], unsigned int lengthData);

    // Sets the operating mode of spi device
    virtual int set_operation_mode(SPI_Communication::SPI_mode operationMode);

    // Sets the value of bits per word of spi device 
    virtual int set_bits_per_word(uint8_t bitsPerWord);

    // Sets the value of speed to communication of spi device
    virtual int set_communication_speed(uint32_t communicationSpeed);

    // Checks all parameters of the spi device
   virtual void debug_dump_registers(unsigned int numberOfRegisters = 0xff);

private:
    std::string fileName;
    SPI_mode operationMode;
    uint8_t bitsPerWord;
    uint16_t communicationDelay;
    uint32_t communicationSpeed;
    
};

#endif
