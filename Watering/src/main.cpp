
#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>

// #define SIMULATE_FLOW_SENSOR

const float servo_step = 25;                  // Rotation step of the servo, should be 22.5 but for some reason this works better
const uint16_t servo_offset = 6 + servo_step; // Start position of the servo
const uint16_t servo_speed = 15;              // Rotation speed of the servo, lower=faster
const uint8_t hole_count = 7;                 // Number of holes in the wheel
const uint8_t servo_servoPin = 9;             // Pin for the servo
const uint8_t pump_relayPin = 4;              // Pin for the relay that controls the pump1
const uint8_t flowSensor_pin = 3;             // Pin for flow sensor
const uint32_t flowStopTime = 5000;           // Time to wait for water to flow back after pump is stopped before moving to next hole, ms (5s)
const uint8_t i2c_address = 0x27;             // I2C address of this device

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

const int eeprom_flowrate = 0;     // EEPROM address for flow rate
const int eeprom_amount_first = 2; // EEPROM address for first water amount
const int eeprom_last_abort_reason = eeprom_amount_first + hole_count * sizeof(uint16_t);
const int eeprom_timeout_flow_initial = eeprom_last_abort_reason + sizeof(AbortReason);
const int eeprom_timeout_flow_mid = eeprom_timeout_flow_initial + sizeof(uint32_t);

bool queuedGoTo = false;
int queuedGoToHole = 0;

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

Servo servo;

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
    uint16_t result = Wire.read();
    result |= Wire.read() << 8;
    return result;
}

uint32_t readLong()
{
    if (Wire.available() < 4)
        Serial.println("Not enough bytes available");
    uint32_t result = Wire.read();
    result |= ((uint32_t)Wire.read()) << 8;
    result |= ((uint32_t)Wire.read()) << 16;
    result |= ((uint32_t)Wire.read()) << 24;
    return result;
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

void setFlowRate(uint16_t flowRate)
{
    Serial.print(F("Setting flow rate calibration to "));
    Serial.println(flowRate);

    state.flowRate = flowRate;
    EEPROM.put(eeprom_flowrate, flowRate);
}

void setWaterAmount(uint8_t index, uint16_t amount)
{
    Serial.print(F("Setting water amount for hole "));
    Serial.print(index + 1);
    Serial.print(F(" to "));
    Serial.println(amount);

    state.amounts[index] = amount;
    EEPROM.put(eeprom_amount_first + index * sizeof(amount), amount);
}

void setLastAbortReason(AbortReason reason)
{
    Serial.print(F("Setting last abort reason to "));
    Serial.println(reason);

    state.lastAbortReason = reason;
    EEPROM.put(eeprom_last_abort_reason, reason);
}

void setTimeoutFlowInitial(uint32_t timeout)
{
    Serial.print(F("Setting timeout_flow_initial to "));
    Serial.print(timeout);
    Serial.println("ms");

    state.timeout_flow_initial = timeout;
    EEPROM.put(eeprom_timeout_flow_initial, timeout);
}

void setTimeoutFlowMid(uint32_t timeout)
{
    Serial.print(F("Setting timeout_flow_mid to "));
    Serial.print(timeout);
    Serial.println("ms");

    state.timeout_flow_mid = timeout;
    EEPROM.put(eeprom_timeout_flow_mid, timeout);
}

void loadConfig()
{
    EEPROM.get(eeprom_flowrate, state.flowRate);

    for (int i = 0; i < hole_count; i++)
        EEPROM.get(eeprom_amount_first + i * sizeof(state.amounts[0]), state.amounts[i]);

    EEPROM.get(eeprom_last_abort_reason, state.lastAbortReason);
    EEPROM.get(eeprom_timeout_flow_initial, state.timeout_flow_initial);
    EEPROM.get(eeprom_timeout_flow_mid, state.timeout_flow_mid);
}

void moveServoToPos(uint16_t targetPos)
{
    while (state.servoPos != targetPos && !state.abort)
    {
        if (state.servoPos < targetPos)
        {
            state.servoPos++;
        }
        else
        {
            state.servoPos--;
        }
        servo.write(state.servoPos);
        delay(servo_speed);
    }
}

void moveServoToHole(uint8_t index)
{
    Serial.print(F("Moving servo to hole "));
    Serial.println(index + 1);
    uint16_t targetPos = servo_offset + (uint16_t)(index * servo_step);
    if (targetPos < state.servoPos)
    {
        // Move to 0 first if we are moving backwards to limit backlash
        moveServoToPos(0);
    }
    if (state.abort)
        return;
    moveServoToPos(targetPos);
    state.servoIndex = index;
}

void setPump(bool on)
{
    if (on)
    {
        Serial.println(F("Turning pump on"));
        digitalWrite(pump_relayPin, HIGH);
    }
    else
    {
        Serial.println(F("Turning pump off"));
        digitalWrite(pump_relayPin, LOW);
    }
    state.pumpOn = on;
}

bool isFlowing(uint32_t timeout_us)
{
#ifdef SIMULATE_FLOW_SENSOR
    delay(1000);
    return true;
#endif

    uint32_t duration = pulseIn(flowSensor_pin, LOW, timeout_us);
    return duration > 0;
}

bool waitUntilFlow()
{
    Serial.println(F("Waiting for flow..."));
    return isFlowing(((uint32_t)state.timeout_flow_initial) * 1000UL);
}

bool safeWait(uint32_t waitTime)
{
    uint32_t lastDetectedFlow = millis();
    uint32_t targetTime = millis() + waitTime;
    while (millis() < targetTime && !state.abort)
    {
        if (isFlowing(50UL * 1000UL))
        {
            lastDetectedFlow = millis();
        }
        if ((millis() - lastDetectedFlow) > state.timeout_flow_mid)
        {
            Serial.println(F("No flow detected mid watering"));
            setLastAbortReason(NoFlowDetectedMidWatering);
            return false;
        }
        if (state.abort)
        {
            Serial.println(F("Manual abort"));
            setLastAbortReason(Manual);
            return false;
        }
    }
    return true;
}

bool waterHole(uint8_t index)
{
    Serial.print(F("Watering hole "));
    Serial.println(index);

    moveServoToHole(index);

    if (state.abort)
    {
        Serial.println(F("Manual abort"));
        setLastAbortReason(Manual);
        return false;
    }

    setPump(true);
    bool flow = waitUntilFlow();
    if (!flow)
    {
        setPump(false);
        Serial.println(F("No flow detected before watering started"));
        setLastAbortReason(NoFlowDetected);
        return false;
    }

    if (state.abort)
    {
        setPump(false);
        Serial.println(F("Manual abort"));
        setLastAbortReason(Manual);
        return false;
    }

    uint32_t waitAmount = (uint32_t)((float)state.amounts[index] / (float)state.flowRate * 1000.0);
    Serial.print(F("Watering for "));
    Serial.print(waitAmount);
    Serial.println(F("ms"));
    if (!safeWait(waitAmount))
    {
        setPump(false);
        return false;
    }

    Serial.println(F("Desired amount reached"));
    setPump(false);

    Serial.println(F("Waiting for flow to stop..."));
    for (int i = 0; i < 50; i++)
    {
        if (state.abort)
        {
            Serial.println(F("Manual abort"));
            setLastAbortReason(Manual);
            return false;
        }
        delay(100);
    }

    Serial.println(F("Watering done"));

    return true;
}

void performWateringSequence()
{
    if (state.flowRate <= 0)
    {
        Serial.println(F("Flow rate is not set, aborting"));
        setLastAbortReason(FlowRateNotSet);
        return;
    }
    Serial.println(F("Performing watering sequence"));
    for (int i = 0; i < hole_count; i++)
    {
        if (state.amounts[i] <= 0)
        {
            continue;
        }
        if (!waterHole(i))
        {
            Serial.println(F("Watering failed, aborting"));
            return;
        }
    }
    if (state.abort)
    {
        Serial.println(F("Watering sequence aborted"));
    }
    else
    {
        Serial.println(F("Watering sequence done"));
    }
}

void loop()
{
    if (state.running)
    {
        if (!state.abort)
        {
            setLastAbortReason(None);
            performWateringSequence();
        }
        state.running = false;
        state.abort = false;
    }

    // Safety, turn off pump if it is on
    if (state.pumpOn)
    {
        setPump(false);
    }

    if (queuedGoTo)
    {
        Serial.print(F("Going to hole "));
        Serial.println(queuedGoToHole + 1);
        queuedGoTo = false;
        moveServoToHole(queuedGoToHole);
    }

    delay(100);
}

void onI2CReceive(int byteCount)
{
    Serial.print(F("Received "));
    Serial.print(byteCount);
    Serial.println(F(" bytes from I2C"));
    while (Wire.available())
    {
        uint8_t magicByte_1 = readByte();
        uint8_t magicByte_2 = readByte();
        if (magicByte_1 != 0x42 || magicByte_2 != 0x69)
        {
            Serial.println(F("Invalid magic bytes"));
            continue;
        }
        uint8_t messageId = readByte();
        Serial.print(F("Message ID: "));
        Serial.println(messageId);
        switch (messageId)
        {
        case 0:
            setFlowRate(readShort());
            break;
        case 1:
        {
            uint8_t index = readByte();
            uint16_t amount = readShort();
            setWaterAmount(index, amount);
        }
        break;
        case 2:
        {
            state.abort = true;
            Serial.println(F("Aborting"));
        }
        break;
        case 3:
        {
            state.running = true;
            Serial.println(F("Queueing watering sequence"));
        }
        break;
        case 4:
        {
            queuedGoTo = true;
            queuedGoToHole = readByte();
            Serial.print(F("Queueing go to hole "));
            Serial.println(queuedGoToHole + 1);
        }
        break;
        case 5:
        {
            uint32_t timeout = readLong();
            setTimeoutFlowInitial(timeout);
        }
        break;
        case 6:
        {
            uint32_t timeout = readLong();
            setTimeoutFlowMid(timeout);
        }
        break;
        default:
            Serial.println(F("Unknown message ID"));
            break;
        }
    }
}

void onI2CRequest()
{
    writeByte(0x91);
    writeByte(0x14);
    Wire.write((uint8_t *)&state, sizeof(state));
}

void setup()
{
    Serial.begin(9600);
    Serial.println(F("Starting..."));
    Serial.println(F("Loading stored .."));
    loadConfig();
    Serial.println(F("Config loaded"));
    Serial.print(F("Flow rate: "));
    Serial.print(state.flowRate);
    Serial.println(F(" ml/s"));
    Serial.print(F("Amounts: "));
    for (int i = 0; i < hole_count; i++)
    {
        Serial.print(F("Hole "));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(state.amounts[i]);
        Serial.print(F(" ml"));
        if (i < hole_count - 1)
            Serial.print(F(", "));
    }
    Serial.println();
    Serial.print(F("Timeouts: "));
    Serial.print(state.timeout_flow_initial);
    Serial.print(F(" ms, "));
    Serial.print(state.timeout_flow_mid);
    Serial.println(F(" ms"));
    Serial.print("Last abort reason: ");
    Serial.println(abortReasonNames[state.lastAbortReason]);

    // Attach servo
    servo.attach(servo_servoPin);

    // Set pump relay pin to output
    pinMode(pump_relayPin, OUTPUT);

    // Set flow sensor pin to input
    pinMode(flowSensor_pin, INPUT);

    // Set servo to 0
    servo.write(servo_offset);

    // Set pump to off
    digitalWrite(pump_relayPin, LOW);

    // Register I2C receiver
    Wire.begin(i2c_address);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

#ifdef SIMULATE_FLOW_SENSOR
    Serial.println(F("!!! Simulating flow sensor !!!"));
#endif

    Serial.println(F("Ready"));
}