// SPI_Interface
#include "SPI_Communication_Service.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>


using namespace std;

int main(int argc, char* argv[]){
    unsigned char send[2], receive[2];

    // Inicializando novo dispositivo SPI
    SPI_Communication *SPIDevice1 = new SPI_Communication(0,0,MODE3,16,2500,0); // canal, dispositivo, modo, bits, speed, delay
    
    // Habilitando comunicação pelo chip select
    SPIDevice1->enable_device_communication(21); // Pino gpio 

    // Escrevendo char
    SPIDevice1->write_device(data); // Unico char

    // Escrevendo palavra
    SPIDevice1->write_device(data[],lengthData); // Array de chars

    // Escrevendo char em um registrador do dispositivo
    SPIDevice1->write_register_device(registerAddress, data); // Endereço do registrador do dispositivo, char

    // Lendo registrador
    SPIDevice1->read_register_device(registerAddress); // Endereço do registrador

    // Lendo vários registradores
    SPIDevice1->read_registers_device(lengthData, fromAddress=0) // Quantidade de chars a serem retornados, endereço do registrador base


    // Desabilitando comunicação pelo chip select
    SPIDevice1->disable_device_communication(21); // Pino gpio 


}



