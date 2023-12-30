#ifndef _DCSBIOS_ESP32_DEFINES_H_
#define _DCSBIOS_ESP32_DEFINES_H_

#include <SPI.h>

#ifndef ARDUINO_ARCH_ESP32
	#error "This code is designed to run on ESP32! Please check your build settings."
#endif

#if !defined(DCSBIOS_ESP32_WIFI_SSID) && !defined(DCSBIOS_ESP32_TCP)
    #error "Ethernet mode only supports TCP."
#endif

#if !defined(DCSBIOS_ESP32_WIFI_SSID) && !defined(DCSBIOS_ESP32_MASTER_IP)
    #error "Ethernet mode does not support mDNS and requires DCSBIOS_ESP32_MASTER_IP and DCSBIOS_ESP32_MASTER_PORT."
#endif

#if defined(DCSBIOS_ESP32_ENC28J60) && defined(DCSBIOS_ESP32_WIZNET)
	#error "You must select either DCSBIOS_ESP32_WIZNET or DCSBIOS_ESP32_ENC28J60."
#endif

#ifndef DCSBIOS_ESP32_ETHERNET_CS
	#define DCSBIOS_ESP32_ETHERNET_CS SS
#endif

#ifndef DCSBIOS_ESP32_LOCAL_PORT
	#define DCSBIOS_ESP32_LOCAL_PORT 7779
#endif

#ifndef DCSBIOS_ESP32_MASTER_PORT
	#define DCSBIOS_ESP32_MASTER_PORT DCSBIOS_ESP32_LOCAL_PORT
#endif

#ifndef DCSBIOS_ESP32_MULTICAST_GROUP
	#define DCSBIOS_ESP32_MULTICAST_GROUP IPAddress(232, 0, 1, 3)
#endif

#ifdef DCSBIOS_ESP32_TCP
#define DCSBIOS_ESP32_SERVICE "_dcs-bios", "_tcp"
#else
#define DCSBIOS_ESP32_SERVICE "_dcs-bios", "_udp"
#endif

#ifndef DCSBIOS_ESP32_ASSOCIATION_TIMEOUT
	#define DCSBIOS_ESP32_ASSOCIATION_TIMEOUT 10000
#endif

#ifndef DCSBIOS_ESP32_SEND_RETRIES
	#define DCSBIOS_ESP32_SEND_RETRIES 3
#endif

#ifndef DCSBIOS_ESP32_SEND_RETRY_DELAY
	#define DCSBIOS_ESP32_SEND_RETRY_DELAY 250
#endif

#ifndef DCSBIOS_ESP32_RECEIVE_TIMEOUT
	#define DCSBIOS_ESP32_RECEIVE_TIMEOUT 3000
#endif

#ifndef DCSBIOS_ESP32_KEEPALIVE_PERIOD
	#define DCSBIOS_ESP32_KEEPALIVE_PERIOD 1000
#endif

#endif