# ESP-VANET 🚗🛜🚒
> **ESP32-based VANET-like inter-vehicular messaging network**

## Abstract

It's known how far an efficient and well-established _V2V_ and _V2X_ network would lead toward a better travel safety, rarer the traffic congestions and lower the fuel emissions. 

To achieve that goals, both the US and the EU have made proposals that could lead to standardize the V2X/V2V communication. In particular, the European Telecommunications Standards Institute
(ETSI) has proposed a middleware solutions for sharing information between near vehicules. These are the _Cooperative Awareness Message (__CAM__)_ and _Decentralized Environmental Notification Message (__DENM__)_.

The ESP32 is a low-cost low-power microcontroller family by Espressif very popular in the industry due its high efficiciency and versatility. Besides the WiFi and Bluetooth standard connectivity it also offer ___ESP-NOW___ a wireless communication protocol between microcontrollers. 

This project simulates the CAM and DENM messages using a ESP32 mesh built over ESP-NOW. 

## Technical Details
### ESP-NOW
> ESP-NOW is a wireless communication protocol defined by Espressif, which enables the direct, quick and low-power control of smart devices, without the need of a router. ESP-NOW can work with Wi-Fi and Bluetooth LE, and supports the ESP8266, ESP32, ESP32-S and ESP32-C series of SoCs. [(Source)](https://www.espressif.com/en/solutions/low-power-solutions/esp-now)

Moreover, it is connectionless, each device can be paired with up to 20 other peers, the max leght of the payload is 250 Byte and the standard bit rate is 1 Mbps. As in WiFi, each device has an unique MAC address that can be used to send messages to it. For broadcast messaging, the address is `0xff:0xff:0xff:0xff:0xff:0xff`.

### CAM and DEMN


### OBD port
