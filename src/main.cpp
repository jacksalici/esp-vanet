
#include "BluetoothSerial.h"
#include <painlessMesh.h>
#include "ELMduino.h"
#include "Arduino.h"

BluetoothSerial SerialBT;

#define DEBUG_PORT Serial
#define ELM_PORT SerialBT
uint8_t address[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03}; // ELM327 MAC CODE

float rpm = 0;
float speed = 0;

ELM327 myELM327;
bool connectedOBD = false

#ifdef LED_BUILTIN
#define LED LED_BUILTIN
#else
#define LED 2
#endif

#define BLINK_PERIOD 3000  // milliseconds until cycle repeat
#define BLINK_DURATION 100 // milliseconds LED is on for

#define MESH_SSID "VANET"
#define MESH_PASSWORD "vanet123"
#define MESH_PORT 5555

    // Prototypes
void sendMessage();

void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void delayReceivedCallback(uint32_t from, int32_t delay);

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage); // start with a one second interval

void setup()
{

  DEBUG_PORT.begin(115200);
  ELM_PORT.begin("ESP32test", true);
  ELM_PORT.setPin("1234");

  DEBUG_PORT.println("Attempting to connect to ELM327...");

  if (!ELM_PORT.connect(address))
  {
    DEBUG_PORT.println("Couldn't connect to ELM327");
  }
  else
  {
    myELM327.begin(ELM_PORT, false, 2000);
    connectedOBD = true DEBUG_PORT.println("Connected to ELM327");
  }

  pinMode(LED, OUTPUT);

  mesh.setDebugMsgTypes(ERROR | DEBUG); // set before init() so that you can see error messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
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
  {
    myELM327.printError();
  }

  mesh.update();
}

void sendMessage()
{
  //Message format: <connected with obd>/<speed>/<rpm>/<id>
  String msg = String(connectedOBD) + "/" + String(speed) "/" + String(rpm) "/" + mesh.getNodeId()
  mesh.sendBroadcast(msg);


  if (calc_delay)
  {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end())
    {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  DEBUG_PORT.printf("Sending message: %s\n", msg.c_str());

  taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 5)); // between 1 and 5 seconds
}

void receivedCallback(uint32_t from, String &msg)
{
  DEBUG_PORT.printf("Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId)
{
  DEBUG_PORT.printf("New Connection, nodeId = %u, %s", nodeId, mesh.subConnectionJson(true).c_str());
}

void changedConnectionCallback()
{
  DEBUG_PORT.printf("Changed connections\n");
  
  nodes = mesh.getNodeList();

  DEBUG_PORT.printf("Num nodes: %d\n", nodes.size());
  DEBUG_PORT.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end())
  {
    DEBUG_PORT.printf(" %u", *node);
    node++;
  }
  DEBUG_PORT.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset)
{
  DEBUG_PORT.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay)
{
  DEBUG_PORT.printf("Delay to node %u is %d us\n", from, delay);
}
