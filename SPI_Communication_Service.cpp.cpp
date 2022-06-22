#include "SPI_Communication_Service.h"


// Default construction
SPI_Communication::SPI_Communication(unsigned int channel, unsigned int device, int operationMode, uint8_t bitsPerWord, uint32_t communicationSpeed, uint16_t communicationDelay)
{
    stringstream s;
    s << SPI_PATH << channel << "." << device;
    this->fileName = string(s.str());
	this->operationMode = operationMode;
	this->bitsPerWord = bitsPerWord;
	this->communicationSpeed = communicationSpeed;
	this->communicationDelay = communicationDelay;
	this->spi_channel_communication();
}

// Default destructor
SPI_Communication::~SPI_Communication()
{
    this->close_spi_communication();
}


/******************* spi_channel_configuration **************************
*************************************************************************
Operation: Configure the SPI channel according to the parameters;
Input Parameters: Nothing;
Output: Status of configuration.
*/
int SPI_Communication::spi_channel_configuration()
{
    int error;

    this->fileDescriptor = ::open(this->fileName.c_str(), O_RDWR);
    if(this->fileDescriptor < 0){
        perror("SPI: Can't open device.");
        return -1;
    }
	
    error = this->set_operation_mode(this->operationMode);
    if(error < 0){
        perror("SPI: Can't set operation mode.");
        return -1;
    }

    error = this->set_communication_speed(this->communicationSpeed);
    if(error < 0){
        perror("SPI: Can't set communication speed.");
        return -1;
    }

    error = this->set_bits_per_word(this->bitsPerWord);
    if(error < 0){
        perror("SPI: Can't set bits per word.");
        return -1;
    }

	return 0;
}

/******************* enable_device_communication **************************
***************************************************************************
Operation: Select the SPI device from GPio pin to enable communication, changing to low level;
Input Parameters: Pin of RaspberryPi GPio;
Output: Nothing;
*/
void SPI_Communication::enable_device_communication(int devicePinSelector)
{
    // Activate the library
    wiringPiSetup();

    // Activate pin as OUTPUT
    pinMode(devicePinSelector, OUTPUT);

    // To disable the pin, it is passed from HIGH to LOW level
    digitalWrite(devicePinSelector, LOW);
}


/******************* disable_device_communication **************************
****************************************************************************
Operation: Select the SPI device from GPio pin to disable communication, changing to high level;
Input Parameters: Pin of RaspberryPi GPio;
Output: Nothing;
*/
void SPI_Communication::disable_device_communication(int devicePinSelector)
{
    // Activate the library
    wiringPiSetup();
    
    // Activate pin as OUTPUT
    pinMode(devicePinSelector, OUTPUT);

    // To disable the pin, it is passed from LOW to HIGH level
    digitalWrite(devicePinSelector, HIGH); 
}

/******************* write_device **************************
************************************************************
Operation: Select RaspberryPi's SPI channel and send the menssage;
Input Parameters: data and length of data to be written;
Output: Message in case of error;
*/

void SPI_Communication::write_device(int data, int writeLength)
{
    // Input and Output Control Structure
    struct spi_ioc_transfer spiMessage[1];

    // Store in memory
    memset(spiMessage, 0, sizeof(spiMessage));

    // Configures the structure with the values of the data and its length to be written by the device
    spiMessage[0].tx_buf = (unsigned long)data;
    spiMessage[0].len = writeLength;    

    // Writes the data and prints a message in case of error
    if (ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message) < 0) 
    {
        perror("Error writing SPI data");
        abort();
    }
}

/******************* read_device **************************
***********************************************************
Operation: Select RaspberryPi's SPI channel and read data from device;
Input Parameters: requisition code and length of data to be written;
Output: Message in case of error;
*/
void SPI_Communication::read_device(int requisitionCode, int readLength)
{
    // Input and Output Control Structure
    struct spi_ioc_transfer spiMessage[1];

    // Store in memory
    memset(spi_message, 0, sizeof(spi_message));

    // Configures the structure with the values of the request code of the word and its length to be read from the device
    spiMessage[0].rx_buf = (unsigned long)requisitionCode;
    spiMessage[0].len = readLength;

    // Writes the data and prints a message in case of error
    if (ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message) < 0) 
    {
        perror("Error reading SPI data");
        abort();
    }
}

/*
Falta colocar os cabeçalhos das funções abaixo
*/
int SPI_Communication::set_operation_mode(int operationMode)
{
    this->mode = operationMode;
    error = octl(this->fileName, SPI_IOC_WR_MODE, &this->operationMode);
	if (error < 0){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}

    error = ioctl(this->fileName, SPI_IOC_RD_MODE, &this->operationMode);
	if (error < 0){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}

	return 0;
}

int SPI_Communication::set_communication_speed(uint32_t communicationSpeed)
{
    this->communicationSpeed = communicationSpeed;

    error = ioctl(this->fileName, SPI_IOC_WR_MAX_SPEED_HZ, &this->communicationSpeed);
	if(error < 0){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}

    error = ioctl(this->fileName, SPI_IOC_RD_MAX_SPEED_HZ, &this->communicationSpeed);
	if(error < 0){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

	return 0;
}

int SPI_Communication::set_bits_per_word(uint8_t bitsPerWord)
{
    this->bitsPerWord = bitsPerWord;

    error = ioctl(this->fileName, SPI_IOC_WR_BITS_PER_WORD, &this->bitsPerWord);
	if(error < 0){
		perror("SPI: Can't set bits per word.");
		return -1;
	}

    error = ioctl(this->file, SPI_IOC_RD_BITS_PER_WORD, &this->bitsPerWord);
	if(error < 0){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	return 0
}

void SPI_Communication::close_spi_communication()
{
    ::close(this->fileName);
    this->fileName = -1;
}

