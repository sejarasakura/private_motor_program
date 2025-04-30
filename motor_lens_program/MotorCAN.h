#ifndef MOTORCAN_H
#define MOTORCAN_H

#include <mcp_can.h>
#include <SPI.h>

class MotorCAN {
  public:
    // Constructor: pass the chip‐select (CS) pin used by the CAN module
    MotorCAN(uint8_t csPin);

    // Initialize the CAN bus; speed and clock are defined by the MCP_CAN library
    bool begin(uint8_t speed, uint8_t clock);

    // Retrieve any error code from the CAN module
    uint8_t getError();

    // --- Command functions (each sends one CAN frame) ---
    void sendReadMotorStatus1(uint8_t motorId);      // Command 1: 0x9A
    void sendClearError(uint8_t motorId);              // Command 2: 0x9B
    void sendReadMotorStatus2(uint8_t motorId);        // Command 3: 0x9C
    void sendReadMotorStatus3(uint8_t motorId);        // Command 4: 0x9D
    void sendShutdown(uint8_t motorId);                // Command 5: 0x80
    void sendRun(uint8_t motorId);                     // Command 6: 0x88
    void sendStop(uint8_t motorId);                    // Command 7: 0x81
    void sendBrakeControl(uint8_t motorId, uint8_t brakeCmd); // Command 8: 0x8C
    void sendOpenLoopControl(uint8_t motorId, int16_t powerControl); // Command 9: 0xA0
    void sendTorqueControl(uint8_t motorId, int16_t iqControl);      // Command 10: 0xA1
    void sendSpeedControl(uint8_t motorId, int32_t speedControl);    // Command 11: 0xA2
    void sendMultiTurnPositionControl1(uint8_t motorId, int32_t angleControl); // Command 12: 0xA3
    void sendMultiTurnPositionControl2(uint8_t motorId, int32_t angleControl, uint16_t maxSpeed); // Command 13: 0xA4
    void sendSingleTurnPositionControl1(uint8_t motorId, uint8_t spinDirection, uint32_t angleControl); // Command 14: 0xA5
    void sendSingleTurnPositionControl2(uint8_t motorId, uint8_t spinDirection, uint32_t angleControl, uint16_t maxSpeed); // Command 15: 0xA6
    void sendIncrementalPositionControl1(uint8_t motorId, int32_t angleIncrement); // Command 16: 0xA7
    void sendIncrementalPositionControl2(uint8_t motorId, int32_t angleIncrement, uint16_t maxSpeed); // Command 17: 0xA8
    void sendReadPIDParams(uint8_t motorId);           // Command 18: 0x30
    void sendWritePIDParamsRAM(uint8_t motorId, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi); // Command 19: 0x31
    void sendWritePIDParamsROM(uint8_t motorId, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi); // Command 20: 0x32
    void sendReadAcceleration(uint8_t motorId);        // Command 21: 0x33
    void sendWriteAccelerationRAM(uint8_t motorId, int32_t accel); // Command 22: 0x34
    void sendReadEncoderData(uint8_t motorId);         // Command 23: 0x90
    void sendWriteEncoderZeroROM(uint8_t motorId, uint16_t encoderOffset); // Command 24: 0x91
    void sendWriteCurrentPositionZeroROM(uint8_t motorId);  // Command 25: 0x19
    void sendReadMultiTurnAngle(uint8_t motorId);      // Command 26: 0x92
    void sendReadSingleTurnAngle(uint8_t motorId);     // Command 27: 0x94
    void sendClearMotorAngle(uint8_t motorId);         // Command 28: 0x95
    void printCAN(unsigned long id, byte* buff, byte len);
    void sendCommand(uint8_t motorId, const uint8_t* data);
    INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U *buf);
    void sendSingleTurnPositionSecure(uint8_t  motorId, uint32_t angleControl, uint32_t currentPositions, uint16_t maxSpeed);
    uint32_t getCalibratedAngle(uint32_t hash);
    void update(uint8_t hex);
    uint32_t getCurrentAngle() { return currentAngle; }
    int convertScaledToRaw(int scale);
    int convertRawToScaled(int raw);

    MCP_CAN can;  // MCP_CAN object for CAN bus communication
  private:

    // Helper function: compute the CAN ID from the motor id (per protocol)
    uint16_t getCANId(uint8_t motorId);
    volatile uint32_t currentAngle = -1;
};

#endif