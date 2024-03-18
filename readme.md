# ESP-VANET 🚗🛜🚒
> **ESP32-based VANET-like inter-vehicular messaging network**

## Abstract

It's known how far an efficient and well-established _V2V_ and _V2X_ network would lead toward a better travel safety, rarer the traffic congestions and lower the fuel emissions. 

To achieve that goals, both the US and the EU have made proposals that could lead to standardize the V2X/V2V communication. In particular, the European Telecommunications Standards Institute
(ETSI) has proposed a middleware solutions for sharing information between near vehicules. These are the _Cooperative Awareness Message (__CAM__)_ and _Decentralized Environmental Notification Message (__DENM__)_.

The ESP32 is a low-cost low-power microcontroller family by Espressif very popular in the industry due its high efficiciency and versatility. Besides the WiFi and Bluetooth standard connectivity it also offer ___ESP-NOW___ a wireless communication protocol between microcontrollers. 

This project simulates the CAM and DENM messages using a ESP32 mesh built over ESP-NOW. 

The advantage is to leverage the costraints needed to deploy a system like this using a general available MCU and the OBD port which is already present in all the cars.

## Technical Details
### ESP-NOW
> ESP-NOW is a wireless communication protocol defined by Espressif, which enables the direct, quick and low-power control of smart devices, without the need of a router. ESP-NOW can work with Wi-Fi and Bluetooth LE, and supports the ESP8266, ESP32, ESP32-S and ESP32-C series of SoCs. [(Source)](https://www.espressif.com/en/solutions/low-power-solutions/esp-now)

Moreover, it is connectionless, each device can be paired with up to 20 other peers, the max leght of the payload is 250 Byte and the standard bit rate is 1 Mbps. As in WiFi, each device has an unique MAC address that can be used to send messages to it. For broadcast messaging, the address is `0xff:0xff:0xff:0xff:0xff:0xff`.

### CAM and DEMN

ETSI TC ITS (Intelligent Transport Systems) has defined a _Basic Set of Application_ provided to managing two type of messages.

>Cooperative Awareness Messages (CAMs) are messages exchanged in the ITS network between ITS-Ss to create and maintain awareness of each other and to support cooperative performance of vehicles using the road network. A CAM contains status and attribute information of the originating ITS-S. The content varies depending on the type of the ITS-S. For vehicle ITS-Ss the status information includes time, position, motion state, activated systems, etc. and the attribute information includes data about the dimensions, vehicle type and role in the road traffic, etc. [Source](https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.03.02_60/en_30263702v010302p.pdf)

> A DENM contains information related to a road hazard or an abnormal traffic conditions, such as its type and its position. The DEN basic service delivers the DENM as payload to the ITS networking & transport layer for the message dissemination. [Source](https://www.etsi.org/deliver/etsi_en/302600_302699/30263703/01.02.02_60/en_30263703v010202p.pdf)

DEMN messages indeed are sent only when some event occuous, like accident or enviromental emergencies. In this project, they are only sent when a the car has a certaint deceleration, as when an accident occours. 


### OBD port
