#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <Wire.h>
#include "settings.h"

const uint8_t i2c_address = 0x28;       // I2C address of this device
const uint8_t slave_i2c_address = 0x27; // I2C address of the controller

const char topic_abort[] = "cmnd/watering/abort";
const char topic_run[] = "cmnd/watering/run";
const char topic_set_flowrate[] = "cmnd/watering/set/flowrate";
const char topic_set_amount[] = "cmnd/watering/set/amount";
const char topic_set_hole[] = "cmnd/watering/set/hole";
const char topic_status[] = "watering/status";
const char topic_set_timeout_flow_initial[] = "cmnd/watering/set/timeout_flow_initial";
const char topic_set_timeout_flow_mid[] = "cmnd/watering/set/timeout_flow_mid";

const uint8_t hole_count = 7;           // Number of holes in the wheel
const uint32_t refresh_interval = 1000; // How often to request state from the controller, in milliseconds

enum __attribute__((__packed__)) AbortReason
{
  Unknown,
  None,
  Manual,
  NoFlowDetected,
  NoFlowDetectedMidWatering,
  FlowRateNotSet,
};

const char abortReasonNames[][32] = {
    "Unknown",
    "None",
    "Manual",
    "No flow detected",
    "No flow detected mid-watering",
    "Flow rate not set",
};

// Warning: uint32_t does not work here for some reason
struct __attribute__((packed)) State
{
  uint16_t servoPos = 0;
  uint8_t servoIndex = 0;
  uint8_t abort = false;
  uint8_t running = false;
  uint8_t pumpOn = false;
  uint16_t flowRate;                         // ml/s
  uint16_t amounts[hole_count];              // ml
  uint16_t timeout_flow_mid = 3 * 1000;      // Timeout for flow sensor pulses after pump is turned on (ms)
  uint16_t timeout_flow_initial = 10 * 1000; // Timeout for initial flow sensor pulses (ms)
  uint8_t lastAbortReason = Unknown;
} state;

WiFiClient net;         // WiFi network client
MQTTClient client(512); // MQTT Client

uint32_t lastStateRefresh = 0; // Last time we checked for messages

uint8_t readByte()
{
  if (Wire.available() < 1)
    Serial.println("Not enough bytes available");
  return Wire.read();
}

uint16_t readShort()
{
  if (Wire.available() < 2)
    Serial.println("Not enough bytes available");
  uint16_t value = Wire.read();
  value |= (uint16_t)Wire.read() << 8;
  return value;
}

void writeByte(uint8_t value)
{
  Wire.write(value);
}

void writeShort(uint16_t value)
{
  Wire.write((uint8_t)value);
  Wire.write((uint8_t)(value >> 8));
}

void writeLong(uint32_t value)
{
  Wire.write((uint8_t)value);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value >> 16));
  Wire.write((uint8_t)(value >> 24));
}

void connect()
{
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("arduino", "public", "public"))
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe(topic_abort);
  client.subscribe(topic_run);
  client.subscribe(topic_set_flowrate);
  client.subscribe(topic_set_amount);
  client.subscribe(topic_set_hole);
  client.subscribe(topic_set_timeout_flow_initial);
  client.subscribe(topic_set_timeout_flow_mid);
}

void messageReceived(String &topic, String &payload)
{
  if (topic.equals(topic_set_flowrate))
  {
    uint16_t flowrate = payload.toInt();
    Serial.println("Sending command to set flow rate calibration to " + String(flowrate));
    Wire.beginTransmission(slave_i2c_address);
    writeByte(0x42);
    writeByte(0x69);
    writeByte(0);
    writeShort(payload.toInt());
    Wire.endTransmission();
  }
  else if (topic.equals(topic_set_amount))
  {
    uint8_t index = payload.substring(0, 1).toInt();
    uint16_t amount = payload.substring(2).toInt();
    Serial.println("Sending command to set amount for hole " + String(index + 1) + " to " + String(amount));
    Wire.beginTransmission(slave_i2c_address);
    writeByte(0x42);
    writeByte(0x69);
    writeByte(1);
    writeByte(index);
    writeShort(amount);
    Wire.endTransmission();
  }
  else if (topic.equals(topic_abort))
  {
    Serial.println("Sending command to abort");
    Wire.beginTransmission(slave_i2c_address);
    writeByte(0x42);
    writeByte(0x69);
    writeByte(2);
    Wire.endTransmission();
  }
  else if (topic.equals(topic_run))
  {
    Serial.println("Sending command to execute watering");
    Wire.beginTransmission(slave_i2c_address);
    writeByte(0x42);
    writeByte(0x69);
    writeByte(3);
    Wire.endTransmission();
  }
  else if (topic.equals(topic_set_hole))
  {
    uint8_t index = payload.toInt();
    Serial.println("Sending command to go to hole " + String(index + 1));
    Wire.beginTransmission(slave_i2c_address);
    writeByte(0x42);
    writeByte(0x69);
    writeByte(4);
    writeByte(index);
    Wire.endTransmission();
  }
  else if (topic.equals(topic_set_timeout_flow_initial))
  {
    uint32_t timeout_ms = payload.toInt();
    Serial.println("Sending command to set timeout_flow_initial to " + String(timeout_ms) + " ms");
    Wire.beginTransmission(slave_i2c_address);
    writeByte(0x42);
    writeByte(0x69);
    writeByte(5);
    writeLong(timeout_ms);
    Wire.endTransmission();
  }
  else if (topic.equals(topic_set_timeout_flow_mid))
  {
    uint32_t timeout_ms = payload.toInt();
    Serial.println("Sending command to set timeout_flow_mid to " + String(timeout_ms) + " ms");
    Wire.beginTransmission(slave_i2c_address);
    writeByte(0x42);
    writeByte(0x69);
    writeByte(6);
    writeLong(timeout_ms);
    Wire.endTransmission();
  }
  else
  {
    Serial.println("Unknown topic: " + topic);
    return;
  }
  Serial.println("Sending command done");
}

void publishState()
{
  client.publish(topic_status, F("online"));
  client.publish("watering/servo/pos", String(state.servoPos));
  client.publish("watering/servo/curidx", String(state.servoIndex + 1));
  client.publish("watering/aborting", state.abort ? F("1") : F("0"));
  client.publish("watering/running", state.running ? F("1") : F("0"));
  client.publish("watering/pump/on", state.pumpOn ? F("1") : F("0"));
  if (state.lastAbortReason >= 0 && state.lastAbortReason < sizeof(abortReasonNames) / sizeof(abortReasonNames[0]))
  {
    client.publish("watering/abortreason", abortReasonNames[state.lastAbortReason]);
  }
  else
  {
    client.publish("watering/abortreason", "Unknown");
  }
  client.publish("watering/config/flowrate", String(state.flowRate));
  client.publish("watering/config/timeout_flow_initial", String(state.timeout_flow_initial));
  client.publish("watering/config/timeout_flow_mid", String(state.timeout_flow_mid));
  for (int i = 0; i < hole_count; i++)
  {
    client.publish("watering/config/amount/" + String(i), String(state.amounts[i]));
  }
}

void onI2CReceive()
{
  while (Wire.available())
  {
    uint8_t magicByte_1 = readByte();
    uint8_t magicByte_2 = readByte();
    if (magicByte_1 != 0x91 || magicByte_2 != 0x14)
    {
      Serial.println(F("Invalid magic bytes"));
      continue;
    }

    Wire.readBytes((uint8_t *)&state, sizeof(state));
    publishState();
  }
}
void setup()
{
  Serial.begin(9600);
  Serial.println(F("Starting..."));

  // Initialize I2C
  Wire.begin(i2c_address);

  // Connect to Wifi
  WiFi.begin(ssid, pass);

  // Setup MQTT client
  client.setWill(topic_status, "offline");
  client.begin(mqtt_broker, mqtt_port, net);
  client.onMessage(messageReceived);

  // Connect to MQTT broker and subscribe to topics
  connect();
}

void assertPublishError()
{
  if (client.lastError() != LWMQTT_SUCCESS)
  {
    Serial.print(F("MQTT publishing error, code: "));
    Serial.println(client.lastError());
  }
}

void publishConfig()
{
  Serial.println(F("Publishing config..."));
  client.publish(
      F("homeassistant/button/watering/run/config"),
      F("{\
\"name\":\"Run\",\
\"availability_topic\":\"watering/status\",\
\"command_topic\":\"cmnd/watering/run\",\
\"payload_press\":\"1\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:play\",\
\"unique_id\":\"watering_run\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/button/watering/abort/config"),
      F("{\
\"name\":\"Abort\",\
\"availability_topic\":\"watering/status\",\
\"command_topic\":\"cmnd/watering/abort\",\
\"payload_press\":\"1\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:alert-octagon\",\
\"unique_id\":\"watering_abort\"\
}"),
      true,
      1);
  for (int i = 0; i < hole_count; i++)
  {
    client.publish(
        F("homeassistant/button/watering/gotohole") + String(i + 1) + F("/config"),
        F("{\
\"name\":\"Go To Hole ") +
            String(i + 1) + F("\",\
\"availability_topic\":\"watering/status\",\
\"command_topic\":\"cmnd/watering/set/hole\",\
\"payload_press\":\"") +
            String(i) + F("\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:numeric-") +
            String(i + 1) + F("-circle-outline\",\
\"unique_id\":\"watering_gotohole") +
            String(i + 1) + F("\"\
}"),
        true,
        1);
    assertPublishError();
  }
  client.publish(
      F("homeassistant/sensor/watering/curidx/config"),
      F("{\
\"name\":\"Current Hole\",\
\"availability_topic\":\"watering/status\",\
\"state_topic\":\"watering/servo/curidx\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:numeric\",\
\"state_class\":\"measurement\",\
\"unique_id\":\"watering_curidx\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/sensor/watering/servopos/config"),
      F("{\
\"name\":\"Servo Position\",\
\"availability_topic\":\"watering/status\",\
\"state_topic\":\"watering/servo/pos\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:angle-acute\",\
\"state_class\":\"measurement\",\
\"unique_id\":\"watering_servopos\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/binary_sensor/watering/aborting/config"),
      F("{\
\"name\":\"Aborting\",\
\"availability_topic\":\"watering/status\",\
\"state_topic\":\"watering/aborting\",\
\"payload_off\":\"0\",\
\"payload_on\":\"1\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:octagon\",\
\"state_class\":\"measurement\",\
\"unique_id\":\"watering_aborting\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/binary_sensor/watering/running/config"),
      F("{\
\"name\":\"Watering\",\
\"availability_topic\":\"watering/status\",\
\"state_topic\":\"watering/running\",\
\"payload_off\":\"0\",\
\"payload_on\":\"1\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:refresh\",\
\"state_class\":\"measurement\",\
\"unique_id\":\"watering_running\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/binary_sensor/watering/pumpon/config"),
      F("{\
\"name\":\"Pump\",\
\"availability_topic\":\"watering/status\",\
\"state_topic\":\"watering/pump/on\",\
\"payload_off\":\"0\",\
\"payload_on\":\"1\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:water-pump\",\
\"state_class\":\"measurement\",\
\"unique_id\":\"watering_pumpon\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/sensor/watering/status/config"),
      F("{\
\"name\":\"Status\",\
\"state_topic\":\"watering/status\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:power-standby\",\
\"state_class\":\"measurement\",\
\"unique_id\":\"watering_status\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/sensor/watering/abortreason/config"),
      F("{\
\"name\":\"Abort Reason\",\
\"availability_topic\":\"watering/status\",\
\"state_topic\":\"watering/abortreason\",\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:text\",\
\"unique_id\":\"watering_abortreason\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/number/watering/flowrate/config"),
      F("{\
\"name\":\"Flow Rate Calibration\",\
\"availability_topic\":\"watering/status\",\
\"command_topic\":\"cmnd/watering/set/flowrate\",\
\"state_topic\":\"watering/config/flowrate\",\
\"min\":1,\
\"max\":1000,\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:cup-water\",\
\"unit_of_measurement\":\"ml/s\",\
\"unique_id\":\"watering_flowrate\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/number/watering/timeoutflowinitial/config"),
      F("{\
\"name\":\"Initial Flow Timeout\",\
\"availability_topic\":\"watering/status\",\
\"command_topic\":\"cmnd/watering/set/timeout_flow_initial\",\
\"state_topic\":\"watering/config/timeout_flow_initial\",\
\"min\":0,\
\"max\":50000,\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:timer-play-outline\",\
\"unit_of_measurement\":\"ms\",\
\"unique_id\":\"watering_timeout_flow_initial\"\
}"),
      true,
      1);
  assertPublishError();
  client.publish(
      F("homeassistant/number/watering/timeoutflowmid/config"),
      F("{\
\"name\":\"Mid-Watering Flow Timeout\",\
\"availability_topic\":\"watering/status\",\
\"command_topic\":\"cmnd/watering/set/timeout_flow_mid\",\
\"state_topic\":\"watering/config/timeout_flow_mid\",\
\"min\":0,\
\"max\":50000,\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"icon\":\"mdi:timer-star-outline\",\
\"unit_of_measurement\":\"ms\",\
\"unique_id\":\"watering_timeout_flow_mid\"\
}"),
      true,
      1);
  assertPublishError();
  for (int i = 0; i < hole_count; i++)
  {
    client.publish(
        F("homeassistant/number/watering/amount_") + String(i) + F("/config"),
        F("{\
\"name\":\"Water Amount ") +
            String(i + 1) + F("\",\
\"availability_topic\":\"watering/status\",\
\"command_topic\":\"cmnd/watering/set/amount\",\
\"command_template\":\"") +
            String(i) + F(",{{ value }}\",\
\"state_topic\":\"watering/config/amount/") +
            String(i) + F("\",\
\"min\":0,\
\"max\":10000,\
\"device\":{\
\"name\":\"Watering\",\
\"manufacturer\":\"Daniel\",\
\"identifiers\":[\"watering\"]\
},\
\"device_class\":\"volume\",\
\"icon\":\"mdi:numeric-") +
            String(i + 1) + F("\",\
\"unit_of_measurement\":\"ml\",\
\"unique_id\":\"watering_amount_") +
            String(i) + F("\"\
}"),
        true,
        1);
  }
  assertPublishError();
  Serial.println(F("Publishing config done"));
}

bool hasPublishedConfig = false;

void loop()
{
  client.loop();
  delay(10); // <- fixes some issues with WiFi stability

  // Try reconnect if we lost the WiFi connection
  if (!client.connected())
  {
    connect();
  }

  // Publish homeassistant discovery config if we haven't already
  if (!hasPublishedConfig)
  {
    publishConfig();
    hasPublishedConfig = true;
  }

  // Request state from controller
  if (millis() - lastStateRefresh > refresh_interval)
  {
    lastStateRefresh = millis();

    Wire.requestFrom(slave_i2c_address, 2 + sizeof(state));
    onI2CReceive();
  }
}