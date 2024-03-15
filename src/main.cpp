#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define DEBUG_PORT Serial

#if CHIP_ESP32
#include "ELMduino.h"
#include <BluetoothSerial.h>
#define ELM_PORT SerialBT

BluetoothSerial SerialBT;

uint8_t elm_address[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03}; // ELM327 MAC CODE
ELM327 myELM327;

#endif

const uint8_t broadcast_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t slave;

float rpm = 0;
float speed = 0;

bool connectedOBD = false;

bool coded(esp_err_t code){
	bool ret = false;
	switch (code)
	{
	case ESP_OK:
		Serial.println("Pair success");
		ret = true;
		break;

	case ESP_ERR_ESPNOW_NOT_INIT:
		Serial.println("ESPNOW Not Init");
		break;

	case ESP_ERR_ESPNOW_ARG:
		Serial.println("Invalid Argument");
		break;

	case ESP_ERR_ESPNOW_FULL:
		Serial.println("Peer list full");
		break;
		
	case ESP_ERR_ESPNOW_NO_MEM:
		Serial.println("Out of memory");
		break;
	
	case ESP_ERR_ESPNOW_NOT_FOUND:
		Serial.println("Peer not found.");
		break;

	case ESP_ERR_ESPNOW_EXIST:
		Serial.println("Peer Exists");
		ret = true;
		break;

	default:
		Serial.println("Not sure what happened");
		break;
	}

	return false;
}

bool manageSlave()
{
	const esp_now_peer_info_t *peer = &slave;
	return coded(esp_now_add_peer(peer));	
}

void deletePeer()
{
	const esp_now_peer_info_t *peer = &slave;
	const uint8_t *peer_addr = slave.peer_addr;
	coded(esp_now_del_peer(peer_addr));
}

// send data
void sendData(uint8_t counter)
{
	uint8_t data = counter;
	// const uint8_t *peer_addr = NULL;
	const uint8_t *peer_addr = slave.peer_addr;

	Serial.print("Sending: ");
	Serial.println(data);
	esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
	Serial.print("Send Status: ");
	coded(result);
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	char macStr[18];
	snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
			 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	Serial.print("Last Packet Sent to: ");
	Serial.println(macStr);
	Serial.print("Last Packet Send Status: ");
	Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
	char macStr[18];
	snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
			 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	Serial.print("Last Packet Recv from: ");
	Serial.println(macStr);
	Serial.print("Last Packet Recv Data: ");
	Serial.println(*data);
	Serial.println("");
}

// Init ESP Now with fallback
void InitESPNow()
{
	if (esp_now_init() == ESP_OK)
	{
		Serial.println("ESPNow Init Success");
	}
	else
	{
		Serial.println("ESPNow Init Failed");
		// Retry InitESPNow, add a counte and then restart?
		// InitESPNow();
		// or Simply Restart
		ESP.restart();
	}
}
void initBroadcastSlave()
{
	// clear slave data
	memset(&slave, 0, sizeof(slave));
	for (int ii = 0; ii < 6; ++ii)
	{
		slave.peer_addr[ii] = (uint8_t)0xff;
	}
	slave.encrypt = 0; // no encryption
	manageSlave();
}

// Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage); // start with a one second interval

void setup()
{

	DEBUG_PORT.begin(115200);

#if CHIP_ESP32

	ELM_PORT.begin("ESP32", true);
	ELM_PORT.setPin("1234");

	if (!ELM_PORT.connect(elm_address))
	{
		DEBUG_PORT.println("Couldn't connect to ELM327");
	}
	else
	{
		myELM327.begin(ELM_PORT, false, 2000);
		connectedOBD = true;
		DEBUG_PORT.println("Connected to ELM327");
	}

#else
	DEBUG_PORT.println("Bluetooth not defined, ELM327 not connected");

#endif

	WiFi.mode(WIFI_STA);

	InitESPNow();
	// Once ESPNow is successfully Init, we will register for Send CB to
	// get the status of Trasnmitted packet
	esp_now_register_send_cb(OnDataSent);
	esp_now_register_recv_cb(OnDataRecv);

	// add a broadcast peer
	initBroadcastSlave();
}

void loop()
{
#if CHIP_ESP32

	if (connectedOBD)
	{
		float tempRPM = myELM327.rpm();

		if (myELM327.nb_rx_state == ELM_SUCCESS)
		{
			rpm = (uint32_t)tempRPM;
			DEBUG_PORT.print("RPM: ");
			DEBUG_PORT.println(rpm);
			sendData(rpm);
		}
		else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
		{
			myELM327.printError();
		}
	}
#endif
	sendData(0x00);
}
