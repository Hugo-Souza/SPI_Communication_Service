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


class SPI_Communication
{
public:
    // Constructor
    SPI_Communication();

    // Destructor
    ~SPI_Communication();

    // Configure the SPI channel according to the parameters
    void spi_channel_configuration(int channel, int operationMode, int communicationSpeed, int bitsPerWord);

    // Select the device to enable communication
    void enable_device_communication(int devicePinSelector);

    // Select the device to disable communication
    void disable_device_communication(int devicePinSelector);

    // Select RaspberryPi's SPI channel and send the menssage
    void write_device(int channel, int communicationDelay, int communicationSpeed, int bitsPerWord, int data);

    // Select RaspberryPi's SPI channel and read data from device
    void read_device(int channel, int communicationDelay, int communicationSpeed, int bitsPerWord, int requisitionCode);
private:
    int descriptor;
};

#endif