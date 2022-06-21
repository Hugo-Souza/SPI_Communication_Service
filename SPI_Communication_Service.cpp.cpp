#include "SPI_Communication_Service.h"


// Default construction
SPI_Communication::SPI_Communication(int channel, int operationMode, int communicationSpeed, int bitsPerWord)
{
    
}

// Default destructor
SPI_Communication::~SPI_Communication()
{

}

/******************* spi_channel_configuration **************************
*************************************************************************
Operation: Configure the SPI channel according to the parameters;
Input Parameters: Nothing;
Output: Status of configuration;
*/
void SPI_Communication::spi_channel_configuration()
{
    
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
