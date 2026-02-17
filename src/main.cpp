#include <Arduino.h>

long previousMillis = 0;

#define ENCODER_STEP_PER_UNIT 10

volatile long encoderStepCount = 2048*ENCODER_STEP_PER_UNIT;  // Global step count for quadrature encoder

#define USE_HALF_DUPLEX 1

enum State
{
    WAITING_FIRST_FF,
    WAITING_SECOND_FF,
    WAITING_ID,
    WAITING_LENGTH,
    RECEIVING_PAYLOAD,
};

#define REG_FIRMWARE_MAIN_VERSION 0
#define REG_FIRMWARE_SUB_VERSION 1
#define REG_SERVO_MAIN_VERSION 3
#define REG_SERVO_SUB_VERSION 4
#define REG_ID 5
#define REG_BAUD_RATE 6
#define REG_RETURN_DELAY_TIME 7
#define REG_STATUS_RETURN_LEVEL 8
#define REG_CW_ANGLE_LIMIT_L 9
#define REG_CW_ANGLE_LIMIT_H 10
#define REG_CCW_ANGLE_LIMIT_L 11
#define REG_CCW_ANGLE_LIMIT_H 12
#define REG_TEMPERATURE_LIMIT 13
#define REG_MAX_VOLTAGE_LIMIT 14
#define REG_MIN_VOLTAGE_LIMIT 15
#define REG_MAX_TORQUE_L 16
#define REG_MAX_TORQUE_H 17
#define REG_SETTING_BYTE 18
#define REG_PROTECTION_SWITCH 19
#define REG_ALARM_LED 20
#define REG_P_GAIN 0x15
#define REG_D_GAIN 0x16
#define REG_I_GAIN 0x17

#define REG_TARGET_POSITION_L 0x2A
#define REG_TARGET_POSITION_H 0x2B
#define REG_OPERATION_TIME_L 0x2C
#define REG_OPERATION_TIME_H 0x2D
#define REG_OPERATION_SPEED_L 0x2E
#define REG_OPERATION_SPEED_H 0x2F

#define REG_POSITION_L 0x38
#define REG_POSITION_H 0x39
#define REG_SPEED_L 0x3A
#define REG_SPEED_H 0x3B
#define REG_CURRENT_POWER_L 0x3C
#define REG_CURRENT_POWER_H 0x3D
#define REG_VOLTAGE 0x3E
#define REG_TEMPERATURE 0x3F
#define REG_MOVING 0x42
#define REG_CURRENT_L 0x45
#define REG_CURRENT_H 0x46



#define PIN_ENC_A PB6
#define PIN_ENC_B PB7

#define PIN_MOTOR_IN1 PB8
#define PIN_MOTOR_IN2 PB5
#define PIN_MOTOR_EN PB9

#define PIN_MANUAL_MODE_UP A1
#define PIN_MANUAL_MODE_DOWN A0

#define PIN_LIMIT_LOWER PA8
#define PIN_LIMIT_UPPER PA9

class ST3215Handler
{
public:
    ST3215Handler()
    {
        mMemoryTable[REG_FIRMWARE_MAIN_VERSION] = 3;
        mMemoryTable[REG_FIRMWARE_SUB_VERSION] = 6;
        mMemoryTable[REG_SERVO_MAIN_VERSION] = 9;
        mMemoryTable[REG_SERVO_SUB_VERSION] = 3;
        mMemoryTable[REG_ID] = mMyId;
        mMemoryTable[REG_BAUD_RATE] = 1; // 500Kbps

        mMemoryTable[REG_TARGET_POSITION_L] = 0;
        mMemoryTable[REG_TARGET_POSITION_H] = 8;
    }

    void begin()
    {
        #if USE_HALF_DUPLEX
        Serial3.setHalfDuplex();
        #endif
        Serial3.begin(500000);;
        // pinMode(PB10, INPUT);
        #if USE_HALF_DUPLEX
        Serial3.enableHalfDuplexRx();
        #endif
    }

    bool handleCommunication()
    {
        bool received = false;

        while (Serial3.available())
        {
            const uint8_t byte = Serial3.read();
            // Serial.printf("Received byte: 0x%02X\n", byte);
            handleByte(byte);
            #if USE_HALF_DUPLEX
            Serial3.enableHalfDuplexRx();
            #endif
            received = true;
        }
        return received;
    }

    // private:
    int mMyId = 0x01;
    State state = WAITING_FIRST_FF;
    int mCctiveId = -1;
    int mPayloadLength = 0;
    int payloadIndex = 0;
    uint8_t payload[256];

    void handleByte(uint8_t byte)
    {
        switch (state)
        {
        case WAITING_FIRST_FF:
            if (byte == 0xFF)
            {
                state = WAITING_SECOND_FF;
            }
            break;
        case WAITING_SECOND_FF:
            if (byte == 0xFF)
            {
                state = WAITING_ID;
            }
            else
            {
                state = WAITING_FIRST_FF;
            }
            break;
        case WAITING_ID:
            if (byte != 0XFF)
            {
                mCctiveId = byte;
                state = WAITING_LENGTH;
            }
            else
            {
                state = WAITING_FIRST_FF;
            }
            break;
        case WAITING_LENGTH:
            mPayloadLength = byte;
            payloadIndex = 0;
            state = RECEIVING_PAYLOAD;
            break;
        case RECEIVING_PAYLOAD:
            payload[payloadIndex++] = byte;
            if (payloadIndex >= mPayloadLength)
            {
                checkCommand();
                state = WAITING_FIRST_FF;
            }
            break;
        }
    }

    void checkCommand()
    {
        if (mCctiveId != mMyId)
        {
            // Serial.printf("Ignoring command for ID 0x%02X\n", mCctiveId);
            return;
        }

        if (mPayloadLength < 2)
        {
            Serial.println("Received command with empty payload.");
            return;
        }

        // TODO check checksum here

        processCommand();
    }

    void processCommand()
    {
        // Serial.printf("Processing command 0x%02X with payload length %d\n", payload[0], mPayloadLength);
        // // Process the payload as needed
        // Serial.print("Payload: ");
        // for (int i = 0; i < mPayloadLength; ++i)
        // {
        //     Serial.printf("0x%02X ", payload[i]);
        // }
        // Serial.println();

        switch (payload[0])
        {
        case 0x01: // ping
            handlePing();
            break;
        case 0x02: // read
            handleRead();
            break;
        case 0x03: // write
            handleWrite();
            break;
        default:
            Serial.printf("Unknown command: 0x%02X\n", payload[0]);
        }
    }

    uint8_t calculateChecksum(const uint8_t *data, size_t length)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < length; ++i)
        {
            sum += data[i];
        }
        return ~sum;
    }

    void tx(const uint8_t *payload, size_t length)
    {
        uint8_t rawData[length + 7];
        rawData[0] = 0xFF;
        rawData[1] = 0xFF;
        rawData[2] = mMyId;
        rawData[3] = length + 2; // length
        rawData[4] = 0;          // status ok

        for (size_t i = 0; i < length; ++i)
        {
            rawData[i + 5] = payload[i];
        }

        const uint8_t checksum = calculateChecksum(rawData + 2, length + 3);
        rawData[length + 5] = checksum;

        Serial3.write(rawData, length + 6);

        // Serial.print("sent: ");
        // for (int i = 0; i < length + 6; ++i)
        // {
        //     Serial.printf("0x%02X ", rawData[i]);
        // }
        // Serial.println();
    }

    void tx()
    {
        tx(nullptr, 0);
    }

    void handlePing()
    {
        tx();
    }

    void handleRead()
    {
        const size_t addr = payload[1];
        const size_t num = payload[2];

        if (num == 0)
        {
            Serial.println("Read request with zero length.");
            return;
        }

        if (num > 4)
        {
            Serial.println("Read request too large.");
            return;
        }

        if (addr + num > mMemoryTable.size())
        {
            Serial.println("Read request out of bounds.");
            return;
        }

        uint8_t response[num];
        for (size_t i = 0; i < num; ++i)
        {
            response[i] = mMemoryTable[addr + i];
        }
        tx(response, num);
    }

    void handleWrite()
    {
        if (mPayloadLength < 3)
        {
            Serial.println("Write request with insufficient length.");
            return;
        }

        const size_t addr = payload[1];
        const size_t num = mPayloadLength - 3; // 3 = write command + addr + checksum

        Serial.printf("Write request to addr 0x%02X with %d bytes\n", addr, num);
        for (int i = 0; i < num; ++i)
        {
            Serial.printf("0x%02X ", payload[i + 2]);
        }
        Serial.println();

        if (addr + num > mMemoryTable.size())
        {
            Serial.println("Write request out of bounds.");
            return;
        }

        for (size_t i = 0; i < num; ++i)
        {
            mMemoryTable[addr + i] = payload[2 + i];
        }

        tx();
    }

    std::array<uint8_t, 71> mMemoryTable;
};

ST3215Handler st3215Handler;

// Interrupt handler for quadrature encoder on PIN_ENC_A
void encoderInterrupt()
{
    // Read the current state of both encoder pins
    const bool encA = digitalRead(PIN_ENC_A);
    const bool encB = digitalRead(PIN_ENC_B);
    
    // Determine direction based on the quadrature pattern
    // If A and B are in the same state, we're moving in one direction
    // If they're in opposite states, we're moving in the other direction
    if (encA == encB)
    {
        encoderStepCount++; 
    }
    else
    {
        encoderStepCount--;
    }
}

static int _lastMotorSpeed = -1000000000; // impossible initial value to ensure first update

void setMotorSpeed(int speed)
{
    
    if (speed == _lastMotorSpeed)
    {
        return;
    }
    _lastMotorSpeed = speed;

    Serial.printf("Setting motor speed to %d\n", speed);

    if(speed > 0 && digitalRead(PIN_LIMIT_UPPER) == LOW) {
        encoderStepCount = 4095*ENCODER_STEP_PER_UNIT; // reset to max position if upper limit is hit
        speed = 0;
    } else if (speed < 0 && digitalRead(PIN_LIMIT_LOWER) == LOW) {
        encoderStepCount = 0; // reset to min position if lower limit is hit
        speed = 0;
    }

    if (speed > 0)
    {
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
        analogWrite(PIN_MOTOR_EN, speed);
    }
    else if (speed < 0)
    {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
        analogWrite(PIN_MOTOR_EN, -speed);
    }
    else
    {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, LOW);
        analogWrite(PIN_MOTOR_EN, 0);
    }
}

void checkMotorLimits()
{
    if (digitalRead(PIN_LIMIT_UPPER) == LOW) {
        encoderStepCount = 4095*ENCODER_STEP_PER_UNIT; // reset to max position if upper limit is hit
        if(_lastMotorSpeed > 0) {
            setMotorSpeed(0);
        }
    }
    else if (digitalRead(PIN_LIMIT_LOWER) == LOW) {
        encoderStepCount = 0; // reset to min position if lower limit is hit
        if(_lastMotorSpeed < 0) {
            setMotorSpeed(0);
        }
    }
}

class MotorController
{
public:
    const int maxSpeed = 255;

    void begin()
    {
        pinMode(PIN_MOTOR_IN1, OUTPUT);
        pinMode(PIN_MOTOR_IN2, OUTPUT);
        pinMode(PIN_MOTOR_EN, OUTPUT);
        analogWriteResolution(8);
        setMotorSpeed(0);
    }

    void doLoop(int targetPosition)
    {
        const int currentPosition = encoderStepCount / ENCODER_STEP_PER_UNIT;
        const int error = targetPosition - currentPosition;

        if(abs(error) < 20) {
            setMotorSpeed(0);
            return; // within deadband
        }

        // Simple P controller
        const int Kp = 5; // Proportional gain
        int controlSignal = Kp * error;

        // Clamp control signal to motor speed limits
        if (controlSignal > maxSpeed) {
            controlSignal = maxSpeed;
        }
        else if (controlSignal < -maxSpeed) {
            controlSignal = -maxSpeed;
        }
        else if (abs(controlSignal) < 50) {
            controlSignal = 0; // deadband
        }

        setMotorSpeed(controlSignal);
    }

    void applySetpointFromEncoder()
    {
        const int currentPosition = encoderStepCount / ENCODER_STEP_PER_UNIT;
        st3215Handler.mMemoryTable[REG_TARGET_POSITION_H] = (currentPosition >> 8) & 0xFF;
        st3215Handler.mMemoryTable[REG_TARGET_POSITION_L] = currentPosition & 0xFF;
    }
};

MotorController motor;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(1000000);
    st3215Handler.begin();

    Serial.println("Setup complete.");
    previousMillis = millis();

    motor.begin();
    
    // Setup quadrature encoder pins and interrupt
    pinMode(PIN_ENC_A, INPUT);
    pinMode(PIN_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderInterrupt, CHANGE);

    pinMode(PIN_MANUAL_MODE_UP, INPUT_PULLUP);
    pinMode(PIN_MANUAL_MODE_DOWN, INPUT_PULLUP);

    pinMode(PIN_LIMIT_LOWER, INPUT_PULLUP);
    pinMode(PIN_LIMIT_UPPER, INPUT_PULLUP);
}

void loop()
{
    const auto received = st3215Handler.handleCommunication();
    if (received)
    {
        previousMillis = millis();
    }
    else if (millis() - previousMillis >= 5000)
    {
        previousMillis = millis();
        Serial.println("No command received in the past.");
    }

    const auto ms = millis();
    static int lastMs = 0;
    const auto setpointPos = st3215Handler.mMemoryTable[REG_TARGET_POSITION_L] | (st3215Handler.mMemoryTable[REG_TARGET_POSITION_H] << 8);
    static int lastSetpointPos = 0;

    if (digitalRead(PIN_MANUAL_MODE_UP) == LOW) {
        setMotorSpeed(200); // example speed for manual mode up
        motor.applySetpointFromEncoder();
    } else if (digitalRead(PIN_MANUAL_MODE_DOWN) == LOW) {
        setMotorSpeed(-200); // example speed for manual mode down
        motor.applySetpointFromEncoder();
    } else if (ms - lastMs >= 10 || setpointPos != lastSetpointPos) {
        lastMs = ms;
        lastSetpointPos = setpointPos;

        motor.doLoop(setpointPos);
    }

    checkMotorLimits();

    const int step = encoderStepCount / ENCODER_STEP_PER_UNIT;
    st3215Handler.mMemoryTable[REG_POSITION_H] = (step >> 8) & 0xFF; // example current position high byte
    st3215Handler.mMemoryTable[REG_POSITION_L] = step & 0xFF;        // example current position low byte
}
