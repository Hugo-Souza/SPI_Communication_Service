#include "Bus_Device_Service.h"


/*********************** BUS_Device *****************************
*****************************************************************
Operation: Creates a new bus device;
Input Parameters: Channel and device number of device;
Output: Nothing;
*/
BUS_Device::BUS_Device(unsigned int channel, unsigned int device)
{
    this->channel = channel;
    this->device = device;
    this->file=-1;   
}


/********************** ~BUS_Device *****************************
*****************************************************************
Operation: Destroys an existing bus device;
Input Parameters: Nothing;
Output: Nothing;
*/
BUS_Device::~BUS_Device()
{
    
}
