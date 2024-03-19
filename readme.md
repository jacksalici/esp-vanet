# ESP-VANET 🚒

> **ESP32-based VANET-like inter-vehicular messaging network**

![project architecture](misc/architecture.svg)

## Abstract

It's known how far an efficient and well-established _V2V_ and _V2X_ network would lead toward better travel safety, rarer traffic congestions and lower fuel emissions.  

To achieve those goals, both the US and the EU have made proposals that could lead to standardizing the V2X/V2V communication. In particular, the European Telecommunications Standards Institute
(ETSI) has proposed a middleware solution for sharing information between nearby vehicles. These are the _Cooperative Awareness Message (**CAM**)_ and _Decentralized Environmental Notification Message (**DENM**)_.

The ESP32 is a low-cost low-power microcontroller family by Espressif very popular in the industry due to its high efficiency and versatility. Besides the WiFi and Bluetooth standard connectivity, it also offers **ESP-NOW** a wireless communication protocol between microcontrollers.  

This project simulates the CAM and DENM messages using an ESP32 mesh built over ESP-NOW and is aimed to leverage the constraints needed to deploy a system like this, since a generally available MCU and the OBD port - which is already present in all the cars - are the only things needed for the code to work.

## Technical Details

### ESP-NOW

> ESP-NOW is a wireless communication protocol defined by Espressif, which enables the direct, quick and low-power control of smart devices, without the need of a router. ESP-NOW can work with Wi-Fi and Bluetooth LE, and supports the ESP8266, ESP32, ESP32-S and ESP32-C series of SoCs. [(Source)](https://www.espressif.com/en/solutions/low-power-solutions/esp-now)

Moreover, it is connectionless, each device can be paired with up to 20 other peers, the max length of the payload is 250 Byte and the standard bit rate is 1 Mbps. As in WiFi, each device has a unique MAC address that can be used to send messages to it. For broadcast messaging, the address is `0xff:0xff:0xff:0xff:0xff:0xff`.

### CAM and DEMN

ETSI TC ITS (Intelligent Transport Systems) has defined a _Basic Set of Applications_ provided to manage two types of messages.

>Cooperative Awareness Messages (CAMs) are messages exchanged in the ITS network between ITS-Ss to create and maintain awareness of each other and to support cooperative performance of vehicles using the road network. A CAM contains status and attribute information of the originating ITS-S. The content varies depending on the type of the ITS-S. For vehicle ITS-Ss the status information includes time, position, motion state, activated systems, etc. and the attribute information includes data about the dimensions, vehicle type and role in the road traffic, etc. [(Source)](https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.03.02_60/en_30263702v010302p.pdf)

> A DENM contains information related to a road hazard or an abnormal traffic conditions, such as its type and its position. The DEN basic service delivers the DENM as payload to the ITS networking & transport layer for the message dissemination. [(Source)](https://www.etsi.org/deliver/etsi_en/302600_302699/30263703/01.02.02_60/en_30263703v010202p.pdf)

DEMN messages indeed are sent only when some event occurs, like an accident or environmental emergency. In this project, they are only sent when a car has a certain deceleration, as when an accident occurs.  

On the other hand, CAM messages are sent with a variable frequency (1-10Hz), that is lower when the car doesn't move. There are indeed two conditions provided by the standard for the message generation trigger:

1. The time elapsed from the last is greater than a dynamic value and at least one of the following:
    - The delta heading is more than 4°,
    - The delta distance is more than 4m,
    - The delta speed is more than 0.5m/s

2. The time elapsed is greater than the provided max value.

For this project the time values are constant and only the second last movement constraint is adopted, this is due to some technical constraints explained below.  

### OBD-2 port adaptor

The On Board Diagnostic is 