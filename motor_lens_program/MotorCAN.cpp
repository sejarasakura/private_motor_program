#include "MotorCAN.h"

MotorCAN::MotorCAN(uint8_t csPin) : can(csPin) {
}

bool MotorCAN::begin(uint8_t speed, uint8_t clock) {
  // Initialize the CAN bus; returns true if initialization is successful.
  return (can.begin(MCP_NORMAL, speed, clock) == CAN_OK);
}

uint8_t MotorCAN::getError() {
  return can.getError();
}

uint16_t MotorCAN::getCANId(uint8_t motorId) {
  // As per protocol: Identifier = 0x140 + motorId (motorId 1..32)
  return 0x140 + motorId;
}

void MotorCAN::sendCommand(uint8_t motorId, const uint8_t* data) {
  uint16_t canId = getCANId(motorId);
  byte sndStat = can.sendMsgBuf(canId, 8, (uint8_t*)data);
  if(sndStat == CAN_OK){
    Serial.print("Sent Successful!!");
  } else {
    Serial.print("Send Error!!");
  }
  printCAN(canId, (byte*)data, 8);
}

// --- Command implementations ---

void MotorCAN::sendReadMotorStatus1(uint8_t motorId) {
  uint8_t data[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendClearError(uint8_t motorId) {
  uint8_t data[8] = {0x9B, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendReadMotorStatus2(uint8_t motorId) {
  uint8_t data[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendReadMotorStatus3(uint8_t motorId) {
  uint8_t data[8] = {0x9D, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendShutdown(uint8_t motorId) {
  uint8_t data[8] = {0x80, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendRun(uint8_t motorId) {
  uint8_t data[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendStop(uint8_t motorId) {
  uint8_t data[8] = {0x81, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendBrakeControl(uint8_t motorId, uint8_t brakeCmd) {
  // brakeCmd options: 0x00 (brake engaged), 0x01 (brake released), 0x10 (read status)
  uint8_t data[8] = {0x8C, brakeCmd, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendOpenLoopControl(uint8_t motorId, int16_t powerControl) {
  uint8_t data[8] = {0xA0, 0, 0, 0, 0, 0, 0, 0};
  data[4] = powerControl & 0xFF;
  data[5] = (powerControl >> 8) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendTorqueControl(uint8_t motorId, int16_t iqControl) {
  uint8_t data[8] = {0xA1, 0, 0, 0, 0, 0, 0, 0};
  data[4] = iqControl & 0xFF;
  data[5] = (iqControl >> 8) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendSpeedControl(uint8_t motorId, int32_t speedControl) {
  uint8_t data[8] = {0xA2, 0, 0, 0, 0, 0, 0, 0};
  data[4] = speedControl & 0xFF;
  data[5] = (speedControl >> 8) & 0xFF;
  data[6] = (speedControl >> 16) & 0xFF;
  data[7] = (speedControl >> 24) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendMultiTurnPositionControl1(uint8_t motorId, int32_t angleControl) {
  uint8_t data[8] = {0xA3, 0, 0, 0, 0, 0, 0, 0};
  data[4] = angleControl & 0xFF;
  data[5] = (angleControl >> 8) & 0xFF;
  data[6] = (angleControl >> 16) & 0xFF;
  data[7] = (angleControl >> 24) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendMultiTurnPositionControl2(uint8_t motorId, int32_t angleControl, uint16_t maxSpeed) {
  uint8_t data[8] = {0xA4, 0, 0, 0, 0, 0, 0, 0};
  data[2] = maxSpeed & 0xFF;
  data[3] = (maxSpeed >> 8) & 0xFF;
  data[4] = angleControl & 0xFF;
  data[5] = (angleControl >> 8) & 0xFF;
  data[6] = (angleControl >> 16) & 0xFF;
  data[7] = (angleControl >> 24) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendSingleTurnPositionControl1(uint8_t motorId, uint8_t spinDirection, uint32_t angleControl) {
  uint8_t data[8] = {0xA5, spinDirection, 0, 0, 0, 0, 0, 0};
  data[4] = angleControl & 0xFF;
  data[5] = (angleControl >> 8) & 0xFF;
  data[6] = (angleControl >> 16) & 0xFF;
  data[7] = (angleControl >> 24) & 0xFF;
  sendCommand(motorId, data);
}

// Maps a hash value to a calibrated angle as follows:
//   - For hash values in [0, 2960):
//       calibrated angle = hash + 10000
//       (Note: 12960° is equivalent to 0°)
//   - For hash values in [2960, 6960]:
//       calibrated angle = hash - 2960
//
// Examples:
//   getCalibratedAngle(0)    -> 10000°
//   getCalibratedAngle(1000) -> 11000°
//   getCalibratedAngle(2960) -> 0° (or 12960° equivalently)
//   getCalibratedAngle(4960) -> 2000°
//   getCalibratedAngle(6960) -> 4000°
uint32_t MotorCAN::getCalibratedAngle(uint32_t hash) {
    if (hash < 2960) {
        uint32_t angle = hash + 10000;
        // Wrap 12960° to 0° since they are equivalent.
        return (angle == 12960 ? 0 : angle);
    } else {
        return hash - 2960;
    }
}

void MotorCAN::sendSingleTurnPositionControl2(
  uint8_t   motorId, 
  uint8_t   spinDirection, 
  uint32_t  angleControl, 
  uint16_t  maxSpeed) 
{
  uint8_t data[8] = {0xA6, spinDirection, 0, 0, 0, 0, 0, 0};
  data[2] = maxSpeed & 0xFF;
  data[3] = (maxSpeed >> 8) & 0xFF;
  data[4] = angleControl & 0xFF;
  data[5] = (angleControl >> 8) & 0xFF;
  data[6] = (angleControl >> 16) & 0xFF;
  data[7] = (angleControl >> 24) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendSingleTurnPositionSecure(
    uint8_t  motorId, 
    uint32_t angleControl, 
    uint32_t currentPositions,
    uint16_t maxSpeed)
{
    // Assume currentPosition holds the current encoder value (0 to 12960)
    uint32_t currentPos = currentPositions;  // this variable must be maintained in the MotorCAN class
    bool lowerZone = false; // flag: true if in lower safe zone ([0,4000]), false if in upper ([10000,12960])
    
    // Determine current safe zone.
    if(currentPos <= 4000) {
        lowerZone = true;
    } else if(currentPos >= 10000) {
        lowerZone = false;
    } else {
        // If current position is in the disabled area, adjust to the nearest safe boundary.
        // (For example, if closer to 4000 then force lower safe zone; otherwise upper.)
        if ((currentPos - 4000) < (10000 - currentPos)) {
            currentPos = 4000;
            lowerZone = true;
        } else {
            currentPos = 10000;
            lowerZone = false;
        }
    }
    
    // Adjust the target angle to ensure it lies within the safe zone.
    // If the motor is in the lower safe zone, allowed range is [0,4000].
    // Otherwise, allowed range is [10000,12960].
    if(lowerZone) {
        if(angleControl > 4000) {
            // The requested angle lies in the disabled area.
            // Adjust to the nearest boundary (in this case, the upper limit of lower safe zone).
            angleControl = 4000;
        }
    } else {
        if(angleControl < 10000) {
            // The requested angle is below the allowed range in the upper safe zone.
            angleControl = 10000;
        }
    }
    
    // Choose the spin direction based on the safe zone:
    // In the lower zone, use clockwise (0x00), in the upper zone, anticlockwise (0x01).
    uint8_t spinDirection = lowerZone ? 0x00 : 0x01;
    
    // Now call the underlying function that sends the command.
    sendSingleTurnPositionControl2(motorId, spinDirection, angleControl, maxSpeed);
}

// void MotorCAN::sendSingleTurnPositionSecure(uint8_t motorId, uint32_t angleControl , uint32_t currentAngle, uint16_t maxSpeed) {
//     // Ensure the hash is within the allowed range [0, 6960].
//     // If angleControl exceeds the maximum allowed, re-correct (i.e. clamp it) to 6960.
//     if (angleControl > 6960) {
//         angleControl = 6960;
//     }
    
//     // Determine the spin direction based on the hash:
//     // - Hash values in [0, 2960) yield a calibrated angle = hash + 10000 (anticlockwise zone, spinDirection 0x01)
//     // - Hash values in [2960, 6960] yield a calibrated angle = hash - 2960 (clockwise zone, spinDirection 0x00)
//     uint8_t spinDirection = (angleControl < 2960) ? 0x01 : 0x00;
    
//     // Calculate the calibrated angle using the provided mapping function.
//     uint32_t calibratedAngle = getCalibratedAngle(angleControl);
    
//     // Call the lower-level command function with the auto-determined direction and calibrated angle.
//     sendSingleTurnPositionControl2(motorId, spinDirection, calibratedAngle, maxSpeed);
// }

void MotorCAN::sendIncrementalPositionControl1(uint8_t motorId, int32_t angleIncrement) {
  uint8_t data[8] = {0xA7, 0, 0, 0, 0, 0, 0, 0};
  data[4] = angleIncrement & 0xFF;
  data[5] = (angleIncrement >> 8) & 0xFF;
  data[6] = (angleIncrement >> 16) & 0xFF;
  data[7] = (angleIncrement >> 24) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendIncrementalPositionControl2(uint8_t motorId, int32_t angleIncrement, uint16_t maxSpeed) {
  uint8_t data[8] = {0xA8, 0, 0, 0, 0, 0, 0, 0};
  data[2] = maxSpeed & 0xFF;
  data[3] = (maxSpeed >> 8) & 0xFF;
  data[4] = angleIncrement & 0xFF;
  data[5] = (angleIncrement >> 8) & 0xFF;
  data[6] = (angleIncrement >> 16) & 0xFF;
  data[7] = (angleIncrement >> 24) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendReadPIDParams(uint8_t motorId) {
  uint8_t data[8] = {0x30, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendWritePIDParamsRAM(uint8_t motorId, uint8_t anglePidKp, uint8_t anglePidKi,
                                      uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
  uint8_t data[8] = {0x31, 0, anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi};
  sendCommand(motorId, data);
}

void MotorCAN::sendWritePIDParamsROM(uint8_t motorId, uint8_t anglePidKp, uint8_t anglePidKi,
                                      uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
  uint8_t data[8] = {0x32, 0, anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi};
  sendCommand(motorId, data);
}

void MotorCAN::sendReadAcceleration(uint8_t motorId) {
  uint8_t data[8] = {0x33, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendWriteAccelerationRAM(uint8_t motorId, int32_t accel) {
  uint8_t data[8] = {0x34, 0, 0, 0, 0, 0, 0, 0};
  data[4] = accel & 0xFF;
  data[5] = (accel >> 8) & 0xFF;
  data[6] = (accel >> 16) & 0xFF;
  data[7] = (accel >> 24) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendReadEncoderData(uint8_t motorId) {
  uint8_t data[8] = {0x90, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendWriteEncoderZeroROM(uint8_t motorId, uint16_t encoderOffset) {
  uint8_t data[8] = {0x91, 0, 0, 0, 0, 0, 0, 0};
  data[6] = encoderOffset & 0xFF;
  data[7] = (encoderOffset >> 8) & 0xFF;
  sendCommand(motorId, data);
}

void MotorCAN::sendWriteCurrentPositionZeroROM(uint8_t motorId) {
  uint8_t data[8] = {0x19, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendReadMultiTurnAngle(uint8_t motorId) {
  uint8_t data[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendReadSingleTurnAngle(uint8_t motorId) {
  uint8_t data[8] = {0x94, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::sendClearMotorAngle(uint8_t motorId) {
  uint8_t data[8] = {0x95, 0, 0, 0, 0, 0, 0, 0};
  sendCommand(motorId, data);
}

void MotorCAN::printCAN(unsigned long id, byte* buff, byte len){
    // Print the message ID in hexadecimal format
    Serial.print("ID: 0x");
    Serial.print(id, HEX);
    Serial.print(" Data: ");
    
    // Loop through each data byte and print it in hex
    for (int i = 0; i < len; i++) {
      // Add a leading zero for values less than 0x10 for consistent formatting
      if (buff[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(buff[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
}

INT8U MotorCAN::readMsgBuf(INT32U *id, INT8U *len, INT8U *buf){
  return can.readMsgBuf(id, len, buf); 
}

void MotorCAN::update(uint8_t hex) {
  INT32U id;
  INT8U len;
  INT8U buf[8];
  readMsgBuf(&id, &len, buf);
  if(id == 0x141){
    printCAN(id, buf, len);
    if(0x94 == buf[0]){
      uint32_t circleAngle = ((uint32_t)buf[4]) | 
                            (((uint32_t)buf[5]) << 8) | 
                            (((uint32_t)buf[6]) << 16) | 
                            (((uint32_t)buf[7]) << 24);
      
      // Output the extracted angle. This assumes you have a Serial interface.
      Serial.print("Circle Angle: ");
      Serial.println(circleAngle);
      currentAngle = circleAngle;
    }
  }
}
int MotorCAN::convertRawToScaled(int raw) {
    if (raw >= 0 && raw <= 1000) {
        return raw + 3760;
    } else if (raw >= 9200 && raw <= 12960) {
        // Reverse direction:
        // When adjusted = 8200 (i.e. raw = 9200), we want the output to be 4760.
        return raw - 9200;
    }
    return -1;
}

// Inverse for raw values from 0 to 1000:
// If scaled is in [3760, 4760] then raw = scaled - 3760.
int MotorCAN::convertScaledToRaw(int scale) {
    if (scale >= 3760 && scale <= 4760) {
        return scale - 3760;
    }else if (scale >= 0 && scale < 3760) {
        return scale + 9200;
    }
    return -1;
}
