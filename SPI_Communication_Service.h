#ifndef SPI_COMMUNICATION_SERVICE_H_INCLUDED
#define SPI_COMMUNICATION_SERVICE_H_INCLUDED

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>


#define SPI_PATH "/dev/spidev"
using namespace std;


class SPI_Communication
{
public:
    // Constructor
    SPI_Communication(unsigned int channel, unsigned int device, int operationMode, uint8_t bitsPerWord, uint32_t communicationSpeed, uint16_t communicationDelay);

    // Destructor
    ~SPI_Communication();

    // Configure the SPI channel according to the parameters
    void spi_channel_configuration();

    // Select the device to enable communication
    void enable_device_communication(int devicePinSelector);

    // Select the device to disable communication
    void disable_device_communication(int devicePinSelector);

    // Select RaspberryPi's SPI channel and send the menssage
    void write_device();

    // Select RaspberryPi's SPI channel and read data from device
    void read_device();
private:
    unsigned int channel;
    unsigned int device;
    int fileDescriptor;
    std::string fileName;
    uint8_t bitsPerWord;
    uint32_t communicationSpeed
    uint16_t communicationDelay;
    int operationMode;
};

#endif