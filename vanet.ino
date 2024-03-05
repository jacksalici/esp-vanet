/*
To test connection, type the following into the serial monitor:
  AT Z
  AT E0
  AT S0
  AT AL
  AT TP A0
*/

#include "BluetoothSerial.h"


BluetoothSerial SerialBT;


#define DEBUG_PORT Serial
#define ELM_PORT   SerialBT
uint8_t address[6] = { 0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03 };

void setup()
{

  DEBUG_PORT.begin(115200);
  ELM_PORT.begin("ESP32test", true);
  ELM_PORT.setPin("1234");

  DEBUG_PORT.println("Attempting to connect to ELM327...");

  if (!ELM_PORT.connect(address))
  {
    DEBUG_PORT.println("Couldn't connect to OBD scanner");
    while(1);
  }

  DEBUG_PORT.println("Connected to ELM327");
  DEBUG_PORT.println("Ensure your serial monitor line ending is set to 'Carriage Return'");
  DEBUG_PORT.println("Type and send commands/queries to your ELM327 through the serial monitor");
  DEBUG_PORT.println();
}


void loop()
{
  if(DEBUG_PORT.available())
  {
    char c = DEBUG_PORT.read();

    DEBUG_PORT.write(c);
    ELM_PORT.write(c);
  }

  if(ELM_PORT.available())
  {
    char c = ELM_PORT.read();

    if(c == '>')
      DEBUG_PORT.println();

    DEBUG_PORT.write(c);
  }
}