#include "Bus_Device_Service.h"

BUS_Device::BUS_Device(unsigned int channel, unsigned int device){
    this->channel = channel;
    this->device = device;
    this->file=-1;   
}

BUS_Device::~BUS_Device(){
    
}
