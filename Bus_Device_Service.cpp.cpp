#include "Bus_Device_Service.h"

BUS_Device::BUS_Device(unsigned int bus, unsigned int device){
    this->bus = bus;
    this->device = device;
    this->file=-1;   
}

DUS_Device::~BUS_Device(){
    
}