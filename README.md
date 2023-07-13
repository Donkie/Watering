# Automatic Watering System

The code for a project about a automatic plant watering system.

It is centralized around only requiring a single servo to choose between 7 different plants to water. This is done using a 3D-printed splitter that is rotated by the servo.

The water is pumped using a 12V pump. The watering amount is calculated by measuring the time that the water has been flowing, using a flow sensor.

The core is driven by an Arduino Uno. It communicates with a ESP8266 NodeMCU over I2C in order to be controlled. The NodeMCU exposes commands and states over a MQTT interface.

## Hardware
* Arduino Uno Rev3
* NodeMCU v0.9
* 12V water pump 240L/h
* Flow sensor 1-30L/min G1/2"
* FS5106B Servo
