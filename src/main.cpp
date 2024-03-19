#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define DEBUG_PORT Serial

template <typename T>
Print &operator<<(Print &printer, T value)
{
	printer.print(value);
	return printer;
}

enum type_message : uint8_t
{
	CAM = 0,
	DEMN = 1
};

typedef struct message
{
	type_message type;
	uint8_t address[6];
	uint8_t coordinates[3];
	uint8_t severity;
	uint8_t speed;
	uint8_t debug;

} message;

#if CHIP_ESP32
#include "ELMduino.h"
#include <BluetoothSerial.h>
#define ELM_PORT SerialBT

BluetoothSerial SerialBT;

uint8_t elm_address[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03}; // ELM327 MAC CODE
ELM327 myELM327;

#endif

const uint8_t broadcast_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t broadcast_peer;

bool connectedOBD = false;

bool coded(esp_err_t code, String str = "")
{
	if (str!=""){
		DEBUG_PORT << "LOG: " << str << ": ";
	}
	bool ret = false;
	switch (code)
	{
	case ESP_OK:
		DEBUG_PORT << "Success\n";
		ret = true;
		break;

	case ESP_ERR_ESPNOW_NOT_INIT:
		DEBUG_PORT << "ESPNOW Not Init\n";
		break;

	case ESP_ERR_ESPNOW_ARG:
		DEBUG_PORT << "Invalid Argument\n";
		break;

	case ESP_ERR_ESPNOW_FULL:
		DEBUG_PORT << "Peer list full\n";
		break;

	case ESP_ERR_ESPNOW_NO_MEM:
		DEBUG_PORT << "Out of memory\n";
		break;

	case ESP_ERR_ESPNOW_NOT_FOUND:
		DEBUG_PORT << "Peer not found.\n";
		break;

	case ESP_ERR_ESPNOW_EXIST:
		DEBUG_PORT << "Peer Exists\n";
		ret = true;
		break;

	default:
		DEBUG_PORT << "Not sure what happened\n";
	}

	return ret;
}

void addPeer()
{
	const esp_now_peer_info_t *peer = &broadcast_peer;
	coded(esp_now_add_peer(peer), "Peering added");
}

void deletePeer()
{
	const uint8_t *peer_addr = broadcast_peer.peer_addr;
	coded(esp_now_del_peer(peer_addr), "Peering deleted");
}

void sendData(message data)
{
	const uint8_t *peer_addr = broadcast_peer.peer_addr;
	esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&data, sizeof(data));

	coded(result, "Packet outgoing, status");
}

void sendCAM(uint8_t speed, uint8_t debug = 0x00)
{

	message m;
	esp_read_mac(m.address, ESP_MAC_WIFI_SOFTAP);

	m.type = CAM;

	const uint8_t a[3] = {0, 0, 0};
	memcpy(m.coordinates, a, 3);
	m.speed = speed;
	m.severity = 0;
	m.debug = debug;

	sendData(m);
}

void sendDENM(uint8_t speed, uint8_t severity, uint8_t debug = 0x00)
{

	message m;
	esp_read_mac(m.address, ESP_MAC_WIFI_SOFTAP);

	m.type = CAM;

	const uint8_t a[3] = {0, 0, 0};
	memcpy(m.coordinates, a, 3);
	m.speed = speed;
	m.severity = severity;
	m.debug = debug;


	sendData(m);
}

void elaborateMessage(const uint8_t *data, int data_len)
{
	message *mex = (message *)data;

	char macStr[18];
	snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
			 mex->address[0], mex->address[1], mex->address[2], mex->address[3], mex->address[4], mex->address[5]);

	DEBUG_PORT << (mex->type == 0 ? "DEN MESSAGE" : "CA MESSAGE") << " PACKET FROM " << macStr << " - SPEED " << mex->speed << " - SEVERITY: " << mex->severity << (mex->debug == 0x01 ? " DEBUG MODE\n" : "\n");
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	char macStr[18];
	snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
			 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	DEBUG_PORT << "LOG: Packet sent to " << macStr << ", " << (status == ESP_NOW_SEND_SUCCESS ? "delivery succeded." : "delivery failed.") << "\n";
}

void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
	char macStr[18];
	snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
			 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	DEBUG_PORT << "LOG: Packet received from " << macStr << ".\n";

	elaborateMessage(data, data_len);
}

void initESPNow()
{
	DEBUG_PORT << "LOG: ";
	if (esp_now_init() == ESP_OK)
	{
		DEBUG_PORT << "ESPNow Init Success\n";
	}
	else
	{
		DEBUG_PORT << "ESPNow Init Failed\n";
		ESP.restart();
	}
}
void initBroadcastPeer()
{
	// clear slave data
	memset(&broadcast_peer, 0, sizeof(broadcast_peer));
	memcpy(broadcast_peer.peer_addr, broadcast_address, 6 * sizeof(uint8_t));
	broadcast_peer.encrypt = 0; // no encryption
	addPeer();
}

long lastKphTime = 0;
uint8_t lastKphValue = 0;
uint8_t currentKphValue = 0;

short demn_gForceThreshold = 9.8 * 3.6 * 1;

long cam_lastGenMillis = -1000;
short cam_kphThreshold = 0.5 * 3.6;
short cam_genMinMillis = 100;
short cam_genMaxMillis = 1000;

void setup()
{

	DEBUG_PORT.begin(115200);
	DEBUG_PORT << "LOG: ";

#if CHIP_ESP32

	ELM_PORT.begin("ESP32", true);
	ELM_PORT.setPin("1234");

	if (!ELM_PORT.connect(elm_address))
	{
		DEBUG_PORT << "Couldn't connect to ELM327\n";
	}
	else
	{
		myELM327.begin(ELM_PORT, false, 2000);
		connectedOBD = true;
		DEBUG_PORT << "Connected to ELM327\n";
	}

#else
	DEBUG_PORT << "Bluetooth not defined, ELM327 not connected\n";

#endif

	WiFi.mode(WIFI_STA);

	initESPNow();

	esp_now_register_send_cb(onDataSent);
	esp_now_register_recv_cb(onDataReceived);

	initBroadcastPeer();
}

void loop()
{
#if CHIP_ESP32

	if (connectedOBD)
	{
		int32_t tempKph = myELM327.kph();

		if (myELM327.nb_rx_state == ELM_SUCCESS)
		{
			// if is DENM case, send it asap
			if (abs((int32_t)lastKphValue - tempKph)/(millis()-lastKphTime) > demn_gForceThreshold){
				sendDENM((uint8_t)tempKph, 3);
			}

			currentKphValue = (uint8_t)tempKph;
			lastKphTime = millis();
		
		}
		else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
		{
			myELM327.printError();
		}
	}
#endif

	// if cam generation costraint are respected, generate cam (with or without debug mode)
	if (
		(millis() > cam_lastGenMillis + cam_genMaxMillis) ||
		((millis() > cam_lastGenMillis + cam_genMinMillis) &&
		 (abs(lastKphValue - currentKphValue) > cam_kphThreshold)))
	{
		if (connectedOBD)
		{
			sendCAM(currentKphValue);
		}
		else
		{ // debug
			sendCAM((uint8_t)random(255), 0x01);
		}
		cam_lastGenMillis = millis();
	}

	lastKphValue = currentKphValue;
}
