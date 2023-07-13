#include <Arduino.h>

#ifndef SETTINGS_H
#define SETTINGS_H

const char ssid[] = "myssid";             // Network SSID (name)
const char pass[] = "mypass";             // Network password
const char mqtt_broker[] = "192.168.1.1"; // MQTT broker address
const uint16_t mqtt_port = 1234;          // MQTT broker port

#endif
