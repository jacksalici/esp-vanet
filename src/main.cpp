#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "ELMduino.h"
#include <BluetoothSerial.h>

#define DEBUG_PORT Serial
#define ELM_PORT SerialBT

BluetoothSerial SerialBT;

uint8_t elm_address[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03}; // ELM327 MAC CODE
const uint8_t broadcast_address[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
esp_now_peer_info_t slave;

float rpm = 0;
float speed = 0;

ELM327 myELM327;
bool connectedOBD = false;

#ifdef LED_BUILTIN
#define LED LED_BUILTIN
#else
#define LED 2
#endif




// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
		Serial.print("Slave Status: ");
		const esp_now_peer_info_t *peer = &slave;
		const uint8_t *peer_addr = slave.peer_addr;
		// check if the peer exists
		bool exists = esp_now_is_peer_exist(peer_addr);
		if (exists) {
			// Slave already paired.
			Serial.println("Already Paired");
			return true;
		}
		else {
			// Slave not paired, attempt pair
			esp_err_t addStatus = esp_now_add_peer(peer);
			if (addStatus == ESP_OK) {
				// Pair success
				Serial.println("Pair success");
				return true;
			}
			else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
				// How did we get so far!!
				Serial.println("ESPNOW Not Init");
				return false;
			}
			else if (addStatus == ESP_ERR_ESPNOW_ARG) {
				Serial.println("Invalid Argument");
				return false;
			}
			else if (addStatus == ESP_ERR_ESPNOW_FULL) {
				Serial.println("Peer list full");
				return false;
			}
			else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
				Serial.println("Out of memory");
				return false;
			}
			else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
				Serial.println("Peer Exists");
				return true;
			}
			else {
				Serial.println("Not sure what happened");
				return false;
			}
		}

}

void deletePeer() {
	const esp_now_peer_info_t *peer = &slave;
	const uint8_t *peer_addr = slave.peer_addr;
	esp_err_t delStatus = esp_now_del_peer(peer_addr);
	Serial.print("Slave Delete Status: ");
	if (delStatus == ESP_OK) {
		// Delete success
		Serial.println("Success");
	}
	else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
		// How did we get so far!!
		Serial.println("ESPNOW Not Init");
	}
	else if (delStatus == ESP_ERR_ESPNOW_ARG) {
		Serial.println("Invalid Argument");
	}
	else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
		Serial.println("Peer not found.");
	}
	else {
		Serial.println("Not sure what happened");
	}
}


// send data
void sendData(uint8_t counter) {
	uint8_t data = counter;
	// const uint8_t *peer_addr = NULL;
	const uint8_t *peer_addr = slave.peer_addr;

	Serial.print("Sending: "); Serial.println(data);
	esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
	Serial.print("Send Status: ");
	if (result == ESP_OK) {
		Serial.println("Success");
	}
	else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
		// How did we get so far!!
		Serial.println("ESPNOW not Init.");
	}
	else if (result == ESP_ERR_ESPNOW_ARG) {
		Serial.println("Invalid Argument");
	}
	else if (result == ESP_ERR_ESPNOW_INTERNAL) {
		Serial.println("Internal Error");
	}
	else if (result == ESP_ERR_ESPNOW_NO_MEM) {
		Serial.println("ESP_ERR_ESPNOW_NO_MEM");
	}
	else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
		Serial.println("Peer not found.");
	}
	else {
		Serial.println("Not sure what happened");
	}
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
	char macStr[18];
	snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	Serial.print("Last Packet Sent to: "); Serial.println(macStr);
	Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
	char macStr[18];
	snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	Serial.print("Last Packet Recv from: "); Serial.println(macStr);
	Serial.print("Last Packet Recv Data: "); Serial.println(*data);
	Serial.println("");
}

// Init ESP Now with fallback
void InitESPNow() {
	if (esp_now_init() == ESP_OK) {
		Serial.println("ESPNow Init Success");
	}
	else {
		Serial.println("ESPNow Init Failed");
		// Retry InitESPNow, add a counte and then restart?
		// InitESPNow();
		// or Simply Restart
		ESP.restart();
	}
}
void initBroadcastSlave() {
	// clear slave data
	memset(&slave, 0, sizeof(slave));
	for (int ii = 0; ii < 6; ++ii) {
		slave.peer_addr[ii] = (uint8_t)0xff;
	}
	slave.encrypt = 0; // no encryption
	manageSlave();
}



//Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage); // start with a one second interval

void setup()
{

  DEBUG_PORT.begin(115200);
  ELM_PORT.begin("ESP32test", true);
  ELM_PORT.setPin("1234");

  WiFi.mode(WIFI_STA);

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

  pinMode(LED, OUTPUT);

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

