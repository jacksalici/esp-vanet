
#include "BluetoothSerial.h"
#include "ELMduino.h"

BluetoothSerial SerialBT;


#define DEBUG_PORT Serial
#define ELM_PORT   SerialBT
uint8_t address[6] = { 0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03 };
float rpm = 0;

ELM327 myELM327;

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
  myELM327.begin(ELM_PORT, true, 2000);
}


void loop()
{
  rpm = myELM327.rpm();
      
    if (myELM327.nb_rx_state == ELM_SUCCESS)
    {
      DEBUG_PORT.print("rpm: ");
      DEBUG_PORT.println(rpm);
    }
    else
    {
      DEBUG_PORT.println("ERROR");
      myELM327.printError();
      
    }
}