#include "SPI_Communication_Service.h"


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

SPI_Communication::~SPI_Communication()
{
    this->close_connection();
}

int SPI_Communication::open_connection()
{
    if((this->file = ::open(fileName.c_str(), O_RDWR)) < 0){
        perror("SPI: Can't open device.");
        return -1;
    }
    if(this->set_operation_mode(this->operationMode) < 0){
        perror("SPI: Can't set operation mode.");
        return -1;
    } 
    if(this->set_bits_per_word(this->bitsPerWord) < 0){
        perror("SPI: Can't set bits per word.");
        return -1;
    }
    if(this->set_communication_speed(this->communicationSpeed) < 0){
        perror("SPI: Can't set communication speed.");
        return -1;
    }

    return 0;
}

void SPI_Communication::close_connection()
{
    ::close(this->file);
    this->file = -1;
}

void SPI_Communication::enable_device_communication(unsigned int devicePinSelector)
{
    wiringPiSetup();

    pinMode(devicePinSelector,OUTPUT);

    digitalWrite(devicePinSelector,LOW);
}

void SPI_Communication::disable_device_communication(unsigned int devicePinSelector)
{
    wiringPiSetup();

    pinMode(devicePinSelector,OUTPUT);

    digitalWrite(devicePinSelector,HIGH);
}

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
	if (status < 0) {
		perror("SPI: SPI_IOC_MESSAGE Failed");
		return -1;
	}
	return status;
}

unsigned char SPI_Communication::read_register_device(unsigned int registerAddress)
{
    unsigned char sendMessage[2], receiveMessage[2];
	memset(sendMessage, 0, sizeof sendMessage);
	memset(receiveMessage, 0, sizeof receiveMessage);
	sendMessage[0] = (unsigned char) (0x80 + registerAddress);
	this->transfer_message(sendMessage, receiveMessage, 2);
    //cout << "The value that was received is: " << (int) receive[1] << endl;
	return receiveMessage[1];
}

unsigned char* SPI_Communication::read_registers_device(unsigned int number, unsigned int fromAddress=0)
{
    unsigned char* data = new unsigned char[number];
	unsigned char sendMessage[number+1], receiveMessage[number+1];
	memset(sendMessage, 0, sizeof sendMessage);
	sendMessage[0] =  (unsigned char) (0x80 + 0x40 + fromAddress); //set read bit and MB bit
	this->transfer_message(sendMessage, receiveMessage, number+1);
	memcpy(data, receiveMessage+1, number);  //ignore the first (address) byte in the array returned
	return data;
}

int SPI_Communication::write_register_device(unsigned int registerAddress, unsigned char value)
{
    unsigned char sendMessage[2], receiveMessage[2];
	memset(receiveMessage, 0, sizeof receiveMessage);
	sendMessage[0] = (unsigned char) registerAddress;
	sendMessage[1] = value;
	//cout << "The value that was written is: " << (int) send[1] << endl;
	this->transfer_message(sendMessage, receiveMessage, 2);
	return 0;
}

int SPI_Communication::write_device(unsigned char value)
{
    unsigned char null_return = 0x00;
	this->transfer_message(&value, &null_return, 1);
	return 0;
}

int SPI_Communication::write_device(unsigned char value[], int length)
{
    unsigned char null_return = 0x00;
	this->transfer_message(value, &null_return, length);
	return 0;
}

int SPI_Communication::set_operation_mode(SPI_Communication::SPI_mode operationMode)
{
    this->operationMode = operationMode;
    if (ioctl(this->file, SPI_IOC_WR_MODE, &this->operationMode) < 0){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}
	if (ioctl(this->file, SPI_IOC_RD_MODE, &this->operationMode)  < 0){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}
	return 0;
}

int SPI_Communication::set_bits_per_word(uint8_t bitsPerWord)
{
    this->bitsPerWord = bitsPerWord;
    if (ioctl(this->file, SPI_IOC_WR_BITS_PER_WORD, &this->bitsPerWord) < 0){
		perror("SPI: Can't set bits per word.");
		return -1;
	}
	if (ioctl(this->file, SPI_IOC_RD_BITS_PER_WORD, &this->bitsPerWord) < 0){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	return 0;
}

int SPI_Communication::set_communication_speed(uint32_t communicationSpeed)
{
    this->communicationSpeed = communicationSpeed;
    if (ioctl(this->file, SPI_IOC_WR_MAX_SPEED_HZ, &this->communicationSpeed) < 0){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if (ioctl(this->file, SPI_IOC_RD_MAX_SPEED_HZ, &this->communicationSpeed) < 0){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}
	return 0;
}

void SPI_Communication::debugDumpRegisters(unsigned int registers = 0xff)
{
    cout << "SPI Mode: " << this->operationMode << endl;
	cout << "Bits per word: " << (int)this->bitsPerWord << endl;
	cout << "Max speed: " << this->communicationSpeed << endl;
	cout << "Dumping Registers for Debug Purposes:" << endl;
	unsigned char *registers = this->read_registers_device(number);
	for(int i=0; i<(int)number; i++){
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
