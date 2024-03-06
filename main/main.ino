
#include "BluetoothSerial.h"
#include "ELMduino.h"

BluetoothSerial SerialBT;

#define DEBUG_PORT Serial
#define ELM_PORT SerialBT
uint8_t address[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03};
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
    while (1)n
      ;
  }

  DEBUG_PORT.println("Connected to ELM327");
  myELM327.begin(ELM_PORT, false, 2000);
}

void loop()
{
  float tempRPM = myELM327.rpm();

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    rpm = (uint32_t)tempRPM;
    DEBUG_PORT.print("RPM: ");
    DEBUG_PORT.println(rpm);
  }
  else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    myELM327.printError();

}