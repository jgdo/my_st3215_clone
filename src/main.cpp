#include <Arduino.h>
#include <SoftFram.h>
#include <SoftWire.h>
#include <EEPROM.h>
#include <atomic>
#include <cstring>
#include <optional>

long previousMillis = 0;

#define INITIAL_SERVO_ID 0x01

#define MIN_PHYSICAL_POSITION 0
#define MAX_PHYSICAL_POSITION 3730

#define ENCODER_STEP_PER_UNIT 12

#define MOTOR_CONTROL_INTERVAL_MS 10

std::atomic<int32_t> encoderStepCount = 2048 * ENCODER_STEP_PER_UNIT; // Global step count for quadrature encoder
int32_t framEncoderStepCount = 0;
bool framAvailable = false;
bool framEncoderStepCountValid = false;
bool framWriteErrorReported = false;

constexpr uint8_t CRC8_POLYNOMIAL = 0x07;

uint8_t crc8(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0xFF;
    for (size_t i = 0; i < length; ++i)
    {
        checksum ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            checksum = (checksum & 0x80) ? static_cast<uint8_t>((checksum << 1) ^ CRC8_POLYNOMIAL)
                                         : static_cast<uint8_t>(checksum << 1);
        }
    }
    return checksum;
}

void encodeEncoderStepCount(int32_t value, uint8_t *record)
{
    std::memcpy(record, &value, sizeof(value));
    record[sizeof(value)] = crc8(record, sizeof(value));
}

int32_t decodeEncoderStepCount(const uint8_t *record)
{
    int32_t value;
    std::memcpy(&value, record, sizeof(value));
    return value;
}

enum State
{
    WAITING_FIRST_FF,
    WAITING_SECOND_FF,
    WAITING_ID,
    WAITING_LENGTH,
    RECEIVING_PAYLOAD,
    WAIT_FOR_READ_SYNC_TRANSMISSION_TIME,
};

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83

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

#define REG_CW_DEAD_BAND 26
#define REG_CCW_DEAD_BAND 27

#define REG_TARGET_POSITION_L 0x2A
#define REG_TARGET_POSITION_H 0x2B
#define REG_OPERATION_TIME_L 0x2C
#define REG_OPERATION_TIME_H 0x2D
#define REG_OPERATION_SPEED_L 0x2E
#define REG_OPERATION_SPEED_H 0x2F

#define REG_LOCK 0x37

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

#define PIN_LIMIT_LOWER PA9
#define PIN_LIMIT_UPPER PA8

#define PIN_FRAM_SDA PB1
#define PIN_FRAM_SCL PB0

class ST3215Handler
{
    static constexpr size_t FIRST_EEPROM_REG = 5;
    static constexpr size_t LAST_EEPROM_REG = 39;
    static constexpr size_t EEPROM_REG_COUNT = LAST_EEPROM_REG - FIRST_EEPROM_REG + 1;
    static constexpr uint32_t EEPROM_CRC_ADDR = 0;
    static_assert(FIRST_EEPROM_REG > 0, "FIRST_EEPROM_REG must not start at 0 because EEPROM_CRC_ADDR is at 0");

public:
    ST3215Handler()
    {
        // Hardcoded values
        mMemoryTable[REG_FIRMWARE_MAIN_VERSION] = 3;
        mMemoryTable[REG_FIRMWARE_SUB_VERSION] = 6;
        mMemoryTable[REG_SERVO_MAIN_VERSION] = 9;
        mMemoryTable[REG_SERVO_SUB_VERSION] = 3;
        mMemoryTable[REG_ID] = INITIAL_SERVO_ID;

        // EEPROM default values
        mMemoryTable[REG_BAUD_RATE] = 1; // 500Kbps
        mMemoryTable[REG_LOCK] = 1;
        mMemoryTable[REG_CW_DEAD_BAND] = 1;
        mMemoryTable[REG_CCW_DEAD_BAND] = 1;
        mMemoryTable[REG_P_GAIN] = 90;
        mMemoryTable[REG_I_GAIN] = 10;
        mMemoryTable[REG_D_GAIN] = 30;

        // will be set to current position later
        mMemoryTable[REG_TARGET_POSITION_L] = 0;
        mMemoryTable[REG_TARGET_POSITION_H] = 8;
    }

    void begin()
    {
        loadEEPROM();

        Serial3.setHalfDuplex();
        Serial3.begin(500000);
        Serial3.enableHalfDuplexRx();
    }

    bool handleReadSyncTransmission()
    {
        if (state != WAIT_FOR_READ_SYNC_TRANSMISSION_TIME)
        {
            return false;
        }

        // discard all bytes received in between
        while (Serial3.available())
        {
            Serial3.read();
        }

        const auto elapsed = micros() - mReadSyncTransmissionTime_us;
        if (elapsed < mReadSyncTransmissionDelay_us)
        {
            return true;
        }

        tx(mSyncReadResponseBuffer, mSyncReadResponseLength);
        state = WAITING_FIRST_FF;
        return true;
    }

    bool handleCommunication()
    {
        bool received = false;

        if (handleReadSyncTransmission())
        {
            return true;
        }

        while (Serial3.available() && state != WAIT_FOR_READ_SYNC_TRANSMISSION_TIME)
        {
            const uint8_t byte = Serial3.read();
            // Serial.printf("Received byte: 0x%02X\n", byte);
            handleByte(byte);
            received = true;
        }
        return received;
    }

    void writeEEPROM()
    {
        eeprom_buffer_fill(); // load current flash contents so untouched bytes are preserved

        bool changed = false;

        for (size_t reg = FIRST_EEPROM_REG; reg <= LAST_EEPROM_REG; ++reg)
        {
            if (eeprom_buffered_read_byte(reg) != mMemoryTable[reg])
            {
                eeprom_buffered_write_byte(reg, mMemoryTable[reg]);
                changed = true;
            }
        }

        const uint8_t checksum = crc8(mMemoryTable.data() + FIRST_EEPROM_REG, EEPROM_REG_COUNT);
        if (eeprom_buffered_read_byte(EEPROM_CRC_ADDR) != checksum)
        {
            eeprom_buffered_write_byte(EEPROM_CRC_ADDR, checksum);
            changed = true;
        }

        if (changed)
        {
            eeprom_buffer_flush();
            Serial.print("Written memory to EEPROM, checksum is ");
            Serial.println(checksum);

            loadEEPROM();
        }
    }

    void loadEEPROM()
    {
        eeprom_buffer_fill();

        uint8_t stored[EEPROM_REG_COUNT];
        for (size_t i = 0; i < EEPROM_REG_COUNT; ++i)
        {
            stored[i] = eeprom_buffered_read_byte(FIRST_EEPROM_REG + i);
        }
        if (crc8(stored, EEPROM_REG_COUNT) != eeprom_buffered_read_byte(EEPROM_CRC_ADDR))
        {
            Serial.println("Invalid EEPROM CRC, keeping defaults.");
            return; // no valid data stored yet, keep defaults
        }

        std::memcpy(&mMemoryTable[FIRST_EEPROM_REG], stored, EEPROM_REG_COUNT);
        Serial.print("Loaded EEPROM values successfully. Checksum is ");
        Serial.println(eeprom_buffered_read_byte(EEPROM_CRC_ADDR));
    }

    int cwDeadBand() const { return mMemoryTable[REG_CW_DEAD_BAND]; }
    int ccwDeadBand() const { return mMemoryTable[REG_CCW_DEAD_BAND]; }
    int kp() const { return mMemoryTable[REG_P_GAIN]; }
    int ki() const { return mMemoryTable[REG_I_GAIN]; }
    int kd() const { return mMemoryTable[REG_D_GAIN]; }

    // private:
    State state = WAITING_FIRST_FF;
    int mActiveId = -1;
    int mPayloadLength = 0;
    int payloadIndex = 0;
    uint8_t payload[256]; // [0] is instruction, [1] is length, [2...] is data

    uint8_t mSyncReadResponseBuffer[256];
    int mSyncReadResponseLength = 0;
    uint32_t mReadSyncTransmissionDelay_us = 0;
    uint32_t mReadSyncTransmissionTime_us = 0;

    auto myID()
    {
        return mMemoryTable[REG_ID];
    }

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
                mActiveId = byte;
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
                if (state != WAIT_FOR_READ_SYNC_TRANSMISSION_TIME)
                {
                    state = WAITING_FIRST_FF;
                }
            }
            break;
        }
    }

    void checkCommand()
    {
        if (mActiveId != myID() && mActiveId != 0xFE)
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
        case INST_PING:
            handlePing();
            break;
        case INST_READ:
            handleRead();
            break;
        case INST_WRITE:
            handleWrite();
            break;
        case INST_SYNC_READ: // sync read
            handleSyncRead();
            break;
        case INST_SYNC_WRITE: // sync write
            handleSyncWrite();
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
        uint8_t rawData[length + 6];
        rawData[0] = 0xFF;
        rawData[1] = 0xFF;
        rawData[2] = myID();
        rawData[3] = length + 2; // length
        rawData[4] = 0;          // status ok

        for (size_t i = 0; i < length; ++i)
        {
            rawData[i + 5] = payload[i];
        }

        const uint8_t checksum = calculateChecksum(rawData + 2, length + 3);
        rawData[length + 5] = checksum;

        Serial3.write(rawData, length + 6);
        Serial3.flush();
        Serial3.enableHalfDuplexRx();

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

        // Serial.printf("Write request to addr 0x%02X with %d bytes\n", addr, num);
        // for (int i = 0; i < num; ++i)
        // {
        //     Serial.printf("0x%02X ", payload[i + 2]);
        // }
        // Serial.println();

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

    void handleSyncRead()
    {
        const size_t addr = payload[1];
        const size_t numReadBytes = payload[2];
        const size_t numServos = mPayloadLength - 4;

        int my_servo_position = -1;

        for (size_t i = 0; i < numServos; ++i)
        {
            const size_t servo_id = payload[3 + i];
            if (servo_id == myID())
            {
                my_servo_position = i;
                // Handle reading from this servo as needed
                break;
            }
        }

        if (my_servo_position == -1)
        {
            Serial.println("Sync read request does not include this servo.");
            return;
        }

        if (numReadBytes == 0)
        {
            Serial.println("Read request with zero length.");
            return;
        }

        if (addr + numReadBytes > mMemoryTable.size())
        {
            Serial.println("Read request out of bounds.");
            return;
        }

        for (size_t i = 0; i < numReadBytes; ++i)
        {
            mSyncReadResponseBuffer[i] = mMemoryTable[addr + i];
        }
        mSyncReadResponseLength = numReadBytes;

        const float byteDelayS = (1.0f / 500000) * 10;               // 10 bits per byte
        const float packageDelayS = byteDelayS * (numReadBytes + 9); // 6 bytes from protocol, and a implementation i found online additionally adds 3 bytes of delay
        mReadSyncTransmissionDelay_us = static_cast<uint32_t>(packageDelayS * my_servo_position * 1000000);
        mReadSyncTransmissionTime_us = micros();
        state = WAIT_FOR_READ_SYNC_TRANSMISSION_TIME;

        // Serial.printf("Sync read of %d bytes, own id at position %d, transmission delay will be: %u us, state: %d\n", numReadBytes, my_servo_position, mReadSyncTransmissionDelay_us, state);
    }

    void handleSyncWrite()
    {
        const size_t addr = payload[1];
        const size_t num_bytes = payload[2];

        if (addr + num_bytes > mMemoryTable.size())
        {
            Serial.println("Sync write request out of bounds.");
            return;
        }

        const size_t nServos = (mPayloadLength - 4) / (num_bytes + 1); // 4 bytes are {inst, addr, num, checksum}, plus 1 byte per servo ID

        for (size_t i = 0; i < nServos; ++i)
        {
            const size_t segmentStart = 3 + i * (num_bytes + 1);
            const size_t servo_id = payload[segmentStart];
            if (servo_id == myID())
            {
                memcpy(&mMemoryTable[addr], &payload[segmentStart + 1], num_bytes);
                break;
            }
        }

        // no response for sync write!
    }

    std::array<uint8_t, 71> mMemoryTable;
};

ST3215Handler st3215Handler;
uint8_t framRxBuf[64], framTxBuf[64];
SoftWire framWire{PIN_FRAM_SDA, PIN_FRAM_SCL};
SoftFRAM fram{&framWire};

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
        encoderStepCount--;
    }
    else
    {
        encoderStepCount++;
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

    // Serial.printf("Setting motor speed to %d\n", speed);

    if (speed > 0 && digitalRead(PIN_LIMIT_UPPER) == LOW)
    {
        encoderStepCount = MAX_PHYSICAL_POSITION * ENCODER_STEP_PER_UNIT; // reset to max position if upper limit is hit
        speed = 0;
    }
    else if (speed < 0 && digitalRead(PIN_LIMIT_LOWER) == LOW)
    {
        encoderStepCount = MIN_PHYSICAL_POSITION * ENCODER_STEP_PER_UNIT; // reset to min position if lower limit is hit
        speed = 0;
    }

    if (speed > 0)
    {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
        analogWrite(PIN_MOTOR_EN, speed);
    }
    else if (speed < 0)
    {
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
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
    if (digitalRead(PIN_LIMIT_UPPER) == LOW)
    {
        encoderStepCount = MAX_PHYSICAL_POSITION * ENCODER_STEP_PER_UNIT; // reset to max position if upper limit is hit
        if (_lastMotorSpeed > 0)
        {
            setMotorSpeed(0);
        }
    }
    else if (digitalRead(PIN_LIMIT_LOWER) == LOW)
    {
        encoderStepCount = MIN_PHYSICAL_POSITION * ENCODER_STEP_PER_UNIT; // reset to min position if lower limit is hit
        if (_lastMotorSpeed < 0)
        {
            setMotorSpeed(0);
        }
    }
}

class MotorController
{
public:
    const int maxSpeed = 255;
    std::optional<uint32_t> lastUpdateMicros = std::nullopt;
    std::optional<int32_t> lastCount = std::nullopt;

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
        const int targetCount = targetPosition * ENCODER_STEP_PER_UNIT;
        const int currentCount = encoderStepCount;
        const float posError = targetCount - currentCount;

        const uint32_t currentMicros = micros();
        const float dt = lastUpdateMicros.has_value() ? (currentMicros - lastUpdateMicros.value()) * 1e-6f : MOTOR_CONTROL_INTERVAL_MS * 1e-3f;
        lastUpdateMicros = currentMicros;

        const float vel = lastCount.has_value() ? (currentCount - lastCount.value()) / static_cast<float>(ENCODER_STEP_PER_UNIT) / dt : 0.0f;
        lastCount = currentCount;

        const int cwDeadBand = st3215Handler.cwDeadBand() * ENCODER_STEP_PER_UNIT;
        const int ccwDeadBand = st3215Handler.ccwDeadBand() * ENCODER_STEP_PER_UNIT;

        if (posError > -ccwDeadBand && posError < cwDeadBand)
        {
            setMotorSpeed(0);
            return; // within deadband
        }

        const float Kp = st3215Handler.kp() / static_cast<float>(ENCODER_STEP_PER_UNIT);
        const float Ki = st3215Handler.ki() / static_cast<float>(ENCODER_STEP_PER_UNIT);
        const float Kd = st3215Handler.kd() / static_cast<float>(ENCODER_STEP_PER_UNIT);

        // PID controller
        static float integral = 0.0f;
        static float previousError = 0.0f;

        const float integralIncrease = posError * dt;
        integral += integralIncrease;
        const float derivative = (posError - previousError) / dt;
        previousError = posError;

        const float controlSignal = Kp * posError + Ki * integral + Kd * derivative;
        const int pwm = pwmFromControlSignal(controlSignal);
        if (pwm == maxSpeed || pwm == -maxSpeed)
        {
            // Motor is saturated
            integral -= integralIncrease; // Anti-windup: revert the last integral increase
        }

        setMotorSpeed(pwm);
    }

    int pwmFromControlSignal(float controlSignal)
    {
        const int pwm = static_cast<int>(controlSignal);
        // Clamp control signal to motor speed limits
        if (pwm > maxSpeed)
        {
            return maxSpeed;
        }
        else if (pwm < -maxSpeed)
        {
            return -maxSpeed;
        }
        else if (abs(pwm) < 50)
        {
            return 0; // deadband
        }

        return pwm;
    }

    void applySetpointFromEncoder()
    {
        const int currentPosition = encoderStepCount / ENCODER_STEP_PER_UNIT;
        st3215Handler.mMemoryTable[REG_TARGET_POSITION_H] = (currentPosition >> 8) & 0xFF;
        st3215Handler.mMemoryTable[REG_TARGET_POSITION_L] = currentPosition & 0xFF;
    }
};

MotorController motor;

void initializeFram()
{
    framWire.setRxBuffer(framRxBuf, sizeof(framRxBuf));
    framWire.setTxBuffer(framTxBuf, sizeof(framTxBuf));
    framWire.setDelay_us(5);
    framWire.setTimeout(40);
    framWire.begin();

    const int framInitResult = fram.begin();
    if (framInitResult != FRAM_OK)
    {
        Serial.printf("FRAM initialization failed: %d\n", fram.lastError());
    }
    else
    {
        framAvailable = true;
        Serial.printf("FRAM size: %lu bytes\n", static_cast<unsigned long>(fram.getSizeBytes()));

        uint8_t framRecord[sizeof(int32_t) + 1];
        fram.read(0, framRecord, sizeof(framRecord));
        const int framReadError = fram.lastError();
        if (framReadError != FRAM_OK)
        {
            Serial.printf("FRAM encoder step count read failed: %d\n", framReadError);
        }
        else if (crc8(framRecord, sizeof(int32_t)) == framRecord[sizeof(int32_t)])
        {
            const int32_t storedStepCount = decodeEncoderStepCount(framRecord);
            encoderStepCount.store(storedStepCount);
            framEncoderStepCount = storedStepCount;
            framEncoderStepCountValid = true;
            motor.applySetpointFromEncoder();
            Serial.printf("FRAM encoder step count loaded: %ld\n", static_cast<long>(storedStepCount));
        }
        else
        {
            Serial.println("FRAM encoder step count checksum mismatch. Using default step count value.");
        }
    }
}

void saveEncoderStepCountToFram()
{
    if (!framAvailable)
    {
        return;
    }

    const int32_t currentStepCount = encoderStepCount.load();
    if (framEncoderStepCountValid && framEncoderStepCount == currentStepCount)
    {
        return;
    }

    uint8_t framRecord[sizeof(int32_t) + 1];
    encodeEncoderStepCount(currentStepCount, framRecord);
    fram.write(0, framRecord, sizeof(framRecord));
    const int framWriteError = fram.lastError();
    if (framWriteError != FRAM_OK)
    {
        if (!framWriteErrorReported)
        {
            Serial.printf("FRAM encoder step count write failed: %d\n", framWriteError);
            framWriteErrorReported = true;
        }
    }
    else
    {
        framEncoderStepCount = currentStepCount;
        framEncoderStepCountValid = true;
        framWriteErrorReported = false;
    }
}

void checkWriteEeprom()
{
    static uint8_t lastLock = 1;
    const auto lock = st3215Handler.mMemoryTable[REG_LOCK];
    if (lock != lastLock && lock == 1) // lock activated
    {
        st3215Handler.writeEEPROM();
    }
    lastLock = lock;
}

void setup()
{
    delay(2000);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(1000000);

    st3215Handler.begin();

    previousMillis = millis();

    motor.begin();
    initializeFram();

    // Setup quadrature encoder pins and interrupt
    pinMode(PIN_ENC_A, INPUT);
    pinMode(PIN_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderInterrupt, CHANGE);

    pinMode(PIN_MANUAL_MODE_UP, INPUT_PULLUP);
    pinMode(PIN_MANUAL_MODE_DOWN, INPUT_PULLUP);

    pinMode(PIN_LIMIT_LOWER, INPUT_PULLUP);
    pinMode(PIN_LIMIT_UPPER, INPUT_PULLUP);

    Serial.println("Setup complete.");
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
        Serial.printf("No command received in the past. State is %d\n", st3215Handler.state);
        // Serial.printf("FRAM size: %lu bytes\n", static_cast<unsigned long>(fram.getSizeBytes()));
    }

    const auto ms = millis();
    static int lastMs = 0;
    const auto setpointPosRaw = st3215Handler.mMemoryTable[REG_TARGET_POSITION_L] | (st3215Handler.mMemoryTable[REG_TARGET_POSITION_H] << 8);
    const auto setpointPos = constrain(setpointPosRaw, MIN_PHYSICAL_POSITION, MAX_PHYSICAL_POSITION);
    static int lastSetpointPos = 0;

    if (digitalRead(PIN_MANUAL_MODE_UP) == LOW)
    {
        setMotorSpeed(-200);
        motor.applySetpointFromEncoder();
    }
    else if (digitalRead(PIN_MANUAL_MODE_DOWN) == LOW)
    {
        setMotorSpeed(200);
        motor.applySetpointFromEncoder();
    }
    else if (ms - lastMs >= MOTOR_CONTROL_INTERVAL_MS || setpointPos != lastSetpointPos)
    {
        lastMs = ms;
        lastSetpointPos = setpointPos;

        motor.doLoop(setpointPos);
    }

    checkMotorLimits();

    const uint16_t step = constrain(encoderStepCount / ENCODER_STEP_PER_UNIT, MIN_PHYSICAL_POSITION, MAX_PHYSICAL_POSITION);
    st3215Handler.mMemoryTable[REG_POSITION_H] = (step >> 8) & 0xFF;
    st3215Handler.mMemoryTable[REG_POSITION_L] = step & 0xFF;

    saveEncoderStepCountToFram();

    checkWriteEeprom();
}
