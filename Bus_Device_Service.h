#ifndef BUS_DEVICE_SERVICE_H_INCLUDED
#define BUS_DEVICE_SERVICE_H_INCLUDED

class BUS_Device 
{
public:
    // Constructor
    BUS_Device(unsigned int channel, unsigned int device);

    // Destructor
    virtual ~BUS_Device();

    // Open the file conection of device
    virtual int open_connection() = 0;

    // Close the file conection of device
    virtual void close_connection() = 0;

    // Read a char from register containing the device message
    virtual unsigned char read_register_device(unsigned int registerAddress) = 0;
	
    // Read a sequence of char from register containing the device message
    virtual unsigned char* read_registers_device(unsigned int lengthData, unsigned int fromAddress = 0) = 0;
	
    // Write the value to the device
    virtual int write_device(unsigned char data) = 0;
	
    // Write the value to the addressed register
    virtual int write_register_device(unsigned int registerAddress, unsigned char data) = 0;
	
    // Performs a parameter check of the device registers
    virtual void debug_dump_registers_device(unsigned int numberOfRegisters = 0xff) = 0;

protected:
    unsigned int channel;
    unsigned int device;
    int file;
};

#endif