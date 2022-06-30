#include "SPI_Communication_Service.h"


/******************* SPI_Communication **************************
*****************************************************************
Operation: Creates a new SPI communication;
Input Parameters: Channel, device number and communication parameters of SPI device;
Output: Nothing;
*/
SPI_Communication::SPI_Communication(unsigned int channel, unsigned int device, SPI_Communication::SPI_mode operationMode, uint8_t bitsPerWord, uint32_t communicationSpeed, uint16_t communicationDelay):BUS_Device(channel, device)
{
    stringstream s;
    s << SPI_PATH << channel << "." << device;

    this->fileName = string(s.str());
    this->operationMode = operationMode;
    this->bitsPerWord = bitsPerWord;
    this->communicationDelay = communicationDelay;
    this->communicationSpeed = communicationSpeed;

    this->open_connection();
}


/******************* ~SPI_Communication **************************
******************************************************************
Operation: Destroys an existing SPI communication;
Input Parameters: Nothing;
Output: Nothing;
*/
SPI_Communication::~SPI_Communication()
{
    this->close_connection();
}


/********************** open_connection **************************
******************************************************************
Operation: Open a connection to the SPI device via the device file;
Input Parameters: Nothing;
Output: 0 if the connection was made or -1 otherwise;
*/
int SPI_Communication::open_connection()
{
	//cout << "Opening the file: " << fileName.c_str() << endl;
    if((this->file = ::open(fileName.c_str(), O_RDWR)) < 0)
	{
        perror("SPI: Can't open device.");
        return -1;
    }

    if(this->set_operation_mode(this->operationMode) < 0)
	{
        perror("SPI: Can't set operation mode.");
        return -1;
    } 

    if(this->set_bits_per_word(this->bitsPerWord) < 0)
	{
        perror("SPI: Can't set bits per word.");
        return -1;
    }

    if(this->set_communication_speed(this->communicationSpeed) < 0){
        perror("SPI: Can't set communication speed.");
        return -1;
    }

    return 0;
}

/********************** close_connection *************************
******************************************************************
Operation: Close a connection to the SPI device via the device file;
Input Parameters: Nothing;
Output: Nothing;
*/
void SPI_Communication::close_connection()
{
    ::close(this->file);
    this->file = -1;
}

/*************** enable_device_communication *********************
******************************************************************
Operation: Enables the connection between the device and the controller via GPIO;
Input Parameters: GPIO pin where the SPI device is connected;
Output: Nothing;
*/
void SPI_Communication::enable_device_communication(unsigned int devicePinSelector)
{
    wiringPiSetup();

    pinMode(devicePinSelector, OUTPUT);

    digitalWrite(devicePinSelector, LOW);
}


/************** disable_device_communication *********************
******************************************************************
Operation: Disables the connection between the device and the controller via GPIO;
Input Parameters: GPIO pin where the SPI device is connected;
Output: Nothing;
*/
void SPI_Communication::disable_device_communication(unsigned int devicePinSelector)
{
    wiringPiSetup();

    pinMode(devicePinSelector, OUTPUT);

    digitalWrite(devicePinSelector, HIGH);
}


/******************** transfer_message ***************************
******************************************************************
Operation: Performs data transfer between the SPI device and the controller;
Input Parameters:Array containing the messages (data) to be sent and received
				 by the SPI device and their size;
Output: 0 if data transfer has been done or -1 otherwise;
*/
int SPI_Communication::transfer_message(unsigned char sendMessage[], unsigned char receiveMessage[], int lengthMessage)
{
    struct spi_ioc_transfer transfer;

    transfer.tx_buf = (unsigned long) sendMessage;
	transfer.rx_buf = (unsigned long) receiveMessage;
    transfer.len = lengthMessage;
    transfer.speed_hz = this->communicationSpeed;
	transfer.bits_per_word = this->bitsPerWord;
	transfer.delay_usecs = this->communicationDelay;

    int status = ioctl(this->file, SPI_IOC_MESSAGE(1), &transfer);
	if(status < 0) 
	{
		perror("SPI: SPI_IOC_MESSAGE Failed");
		return -1;
	}

	return status;
}


/******************** read_register_device ***********************
******************************************************************
Operation: Read a single data register from the SPI device;
Input Parameters: Address of the register to be read;
Output: Data read from register (single char);
*/
unsigned char SPI_Communication::read_register_device(unsigned int registerAddress)
{
    unsigned char sendMessage[2], receiveMessage[2];

	memset(sendMessage, 0, sizeof sendMessage);
	memset(receiveMessage, 0, sizeof receiveMessage);
	
	sendMessage[0] = (unsigned char) (0x80 + registerAddress);
	this->transfer_message(sendMessage, receiveMessage, 2);
    
	//cout << "The data that was received is: " << (int) receiveMessage[1] << endl;
	
	return receiveMessage[1];
}


/******************** read_registers_device **********************
******************************************************************
Operation: Read a sequence of data registers from SPI device;
Input Parameters: Length of data to be read and initial address of register;
Output: Data read from register (array of char);
*/
unsigned char* SPI_Communication::read_registers_device(unsigned int lengthData, unsigned int fromAddress=0)
{
    unsigned char* data = new unsigned char[lengthData];
	unsigned char sendMessage[lengthData+1], receiveMessage[lengthData+1];

	memset(sendMessage, 0, sizeof sendMessage);
	sendMessage[0] =  (unsigned char) (0x80 + 0x40 + fromAddress); // Set read bit and MB bit
	
	this->transfer_message(sendMessage, receiveMessage, lengthData+1);
	memcpy(data, receiveMessage+1, lengthData);  // Ignore the first (address) byte in the array returned
	
	return data;
}


/******************** write_register_device **********************
******************************************************************
Operation: Write data to an SPI device register;
Input Parameters: Address of the register to be written to the SPI device and the data;
Output: 0 if the data was written in the registers or -1 otherwise;
*/
int SPI_Communication::write_register_device(unsigned int registerAddress, unsigned char data)
{
    unsigned char sendMessage[2], receiveMessage[2];
	
	memset(receiveMessage, 0, sizeof receiveMessage);
	
	sendMessage[0] = (unsigned char) registerAddress;
	sendMessage[1] = data;
	
	//cout << "The data that was written is: " << (int) send[1] << endl;
	this->transfer_message(sendMessage, receiveMessage, 2);
	
	return 0;
}

/************************ write_device ***************************
******************************************************************
Operation: Write a single data to SPI device;
Input Parameters: Array containing the data to be written to the SPI device;
Output: 0 if the data was written to the device or -1 otherwise;
*/
int SPI_Communication::write_device(unsigned char data)
{
    unsigned char null_return = 0x00;

	this->transfer_message(&data, &null_return, 1);
	
	return 0;
}



/************************ write_device ***************************
******************************************************************
Operation: Write a sequence of data to SPI device;
Input Parameters: Array containing the data to be written to the SPI device and its size;
Output: 0 if the data was written to the device or -1 otherwise;
*/
int SPI_Communication::write_device(unsigned char data[], int lengthData)
{
    unsigned char null_return = 0x00;
	
	this->transfer_message(data, &null_return, lengthData);
	
	return 0;
}



/******************** set_operation_mode *************************
******************************************************************
Operation: Sets the operation mode parameter of SPI communication;
Input Parameters: Operation mode (MODE0, MODE1, MODE2, MODE3);
Output: 0 if operation mode has been set or -1 otherwise;
*/
int SPI_Communication::set_operation_mode(SPI_Communication::SPI_mode operationMode)
{
    this->operationMode = operationMode;

    if(ioctl(this->file, SPI_IOC_WR_MODE, &this->operationMode) < 0)
	{
		perror("SPI: Can't set SPI mode.");
		return -1;
	}

	if(ioctl(this->file, SPI_IOC_RD_MODE, &this->operationMode)  < 0)
	{
		perror("SPI: Can't get SPI mode.");
		return -1;
	}

	return 0;
}



/******************** set_bits_per_word ***************************
******************************************************************
Operation: Sets the bit-per-word communication parameter of SPI communication;
Input Parameters: Number of bits-per-word value;
Output: 0 if number of bits-per-word has been set or -1 otherwise;
*/
int SPI_Communication::set_bits_per_word(uint8_t bitsPerWord)
{
    this->bitsPerWord = bitsPerWord;

    if(ioctl(this->file, SPI_IOC_WR_BITS_PER_WORD, &this->bitsPerWord) < 0)
	{
		perror("SPI: Can't set bits per word.");
		return -1;
	}

	if(ioctl(this->file, SPI_IOC_RD_BITS_PER_WORD, &this->bitsPerWord) < 0)
	{
		perror("SPI: Can't get bits per word.");
		return -1;
	}

	return 0;
}



/******************** set_communication_speed ***************************
******************************************************************
Operation: Sets the communication speed parameter;
Input Parameters: Communication speed value in Hz;
Output: 0 if communication speed has been set or -1 otherwise;
*/
int SPI_Communication::set_communication_speed(uint32_t communicationSpeed)
{
    this->communicationSpeed = communicationSpeed;

    if(ioctl(this->file, SPI_IOC_WR_MAX_SPEED_HZ, &this->communicationSpeed) < 0)
	{
		perror("SPI: Can't set max speed HZ");
		return -1;
	}

	if(ioctl(this->file, SPI_IOC_RD_MAX_SPEED_HZ, &this->communicationSpeed) < 0)
	{
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

	return 0;
}



/******************* debug_dump_registers ************************
******************************************************************
Operation: Informs the communication parameters and the content of the SPI device registers;
Input Parameters: Number of registers;
Output: Nothing;
*/
void SPI_Communication::debug_dump_registers(unsigned int numberOfRegisters = 0xff)
{
    cout << "SPI Mode: " << this->operationMode << endl;
	cout << "Bits per word: " << (int)this->bitsPerWord << endl;
	cout << "Max speed: " << this->communicationSpeed << endl;
	cout << "Dumping Registers for Debug Purposes:" << endl;

	unsigned char *registers = this->read_registers_device(numberOfRegisters);
	for(int i=0; i<(int)numberOfRegisters; i++)
	{
		cout << HEX(*(registers+i)) << " ";
		if (i%16==15) cout << endl;
	}

	cout << dec;
}











/******************* close_spi_communication **************************
***********************************************************************
Operation: Closes communication with the SPI;
Input Parameters: Nothing;
Output: Nothing;
*/
void SPI_Communication::close_spi_communication()
{
    ::close(this->fileName);
    this->fileName = -1;
}
