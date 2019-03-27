/*
 * @Description: msg implementation
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-26 10:53:55
 * @LastEditTime: 2019-03-26 19:43:45
 */

#include "Msg.h"
#include <cmath>
#include <cstdio>
#include <sstream>

#ifndef MSG_CONST_H
// reveive msg protocol
#define SHIFT_LEVEL_OFFSET 0
#define SHIFT_LEVEL_LENGTH 2
#define WHEEL_DIRECTION_OFFSET 7
#define WHEEL_ANGLE_OFFSET 8
#define WHEEL_ANGLE_LEGNTH 12
#define SPEED_OFFSET 20
#define SPEED_LENGTH 9
#define CCU_LEFT_TURN 56
#define CCU_RIGHT_TURN 57
#define CCU_HAZARD 58
#define CCU_POSITION 59
#define CCU_LOW_BEAM 60
#define CCU_HIGH_BEAM 61
#define CCU_REAR_FOG 62
#define CCU_HORN 63
#define CCU_TOTAL_ODOMETER_OFFSET 36
#define CCU_TOTAL_ODOMETER_LENGTH 20

// send msg protocol
#define SCU_SHIFT_LEVEL_OFFSET 0
#define SCU_SHIFT_LEVEL_LENGTH 2
#define SCU_ACC_MODE_OFFSET 2
#define SCU_ACC_MODE_LENGTH 2
#define SCU_BRAKE_MODE_OFFSET 4
#define SCU_BRAKE_MODE_LENGTH 2
#define SCU_DRIVE_MODE_REQ_OFFSET 6
#define SCU_DRIVE_MODE_REQ_LENGTH 2
#define SCU_STEERING_WHEEL_ANGLE_OFFSET 8
#define SCU_STEERING_WHEEL_ANGLE_LENGTH 16
#define SCU_TARGET_SPEED_OFFSET 24
#define SCU_TARGET_SPEED_LENGTH 9
#define SCU_EBRAKE_OFFSET 33
#define SCU_EBRAKE_LENGTH 1
#define SCU_GRADIENT_OFFSET 34
#define SCU_GRADIENT_LENGTH 6

#define SCU_LEFT_TURN_LIGHT_OFFSET 40
#define SCU_RIGHT_TURN_LIGHT_OFFSET 42
#define SCU_HAZARD_LIGHT_OFFSET 44
#define SCU_POSITION_LIGHT_OFFSET 46
#define SCU_LOWBEAM_OFFSET 48
#define SCU_HIGHBEAM_OFFSET 50
#define SCU_REARFOGLIGHT_OFFSET 52
#define SCU_HORN_OFFSET 54
#define LIGHT_LENGTH 2
#define SCU_RESERVED_OFFSET 56
#define SCU_RESERVED_LENGTH 8

#define PAYLOAD_LENGTH 8

#endif // !MSG_CONST_H

namespace CanBridge {

void SendMsg::setDriveMode(DriveMode driveMode)
{
    writeInt(this->packet, SCU_DRIVE_MODE_REQ_OFFSET, SCU_DRIVE_MODE_REQ_LENGTH, driveMode);
}

void SendMsg::setAccMode(int v)
{
    writeInt(this->packet, SCU_ACC_MODE_OFFSET, SCU_ACC_MODE_LENGTH, v);
}

void SendMsg::setBrakeMode(int v)
{
    writeInt(this->packet, SCU_BRAKE_MODE_OFFSET, SCU_BRAKE_MODE_LENGTH, v);
}

void SendMsg::setSpeed(double v)
{
    int speed = (int)lround(v * 10);
    writeInt(this->packet, SCU_TARGET_SPEED_OFFSET, SCU_TARGET_SPEED_LENGTH, speed);
}

void SendMsg::setWheelAngle(double angle)
{
    short v = angle * 10;
    writeInt(this->packet, SCU_STEERING_WHEEL_ANGLE_OFFSET, SCU_STEERING_WHEEL_ANGLE_LENGTH, v);
}

void SendMsg::setShiftLevel(ShiftLevel shift_level)
{
    writeInt(this->packet, SCU_SHIFT_LEVEL_OFFSET, SCU_SHIFT_LEVEL_LENGTH, shift_level);
}

void SendMsg::setGradient(double v)
{
    int iv = v * 2;
    writeInt(this->packet, SCU_GRADIENT_OFFSET, SCU_GRADIENT_LENGTH, iv);
}

void SendMsg::setEBrake(bool need)
{
    int v = 0;
    if (need) {
        v = 1;
    }
    writeInt(this->packet, SCU_EBRAKE_OFFSET, SCU_EBRAKE_LENGTH, v);
}

void SendMsg::print()
{
    fprintf(stdout, "print scumsg: \n");
    for (size_t i = 0; i < 8; i++) {
        fprintf(stdout, "0x%X", this->packet[i]);
    }
    fprintf(stdout, "......\n");
}

void SendMsg::serialize(struct can_frame& frame)
{
    bzero(&frame, sizeof(struct can_frame));
    frame.can_dlc = 8;
    for (size_t i = 0; i < PAYLOAD_LENGTH; i++) {
        frame.data[i] = packet[i];
    }
    frame.can_id = 0x120;
}

int RevMsg::shiftLevel()
{
    return readAsInt(this->packet, SHIFT_LEVEL_OFFSET, SHIFT_LEVEL_LENGTH);
}

double RevMsg::Speed()
{
    double speed = readAsInt(this->packet, SPEED_OFFSET, SPEED_LENGTH) / 10.0;
    return speed;
}

double RevMsg::wheelAngle()
{
    double angle = readAsInt(this->packet, WHEEL_ANGLE_OFFSET, WHEEL_ANGLE_LEGNTH);
    return angle;
}

std::string RevMsg::toString()
{
    std::string msg;
    std::stringstream ss;
    ss << "RevMsg speed: " << this->Speed() << ", shiftLevel: " << this->shiftLevel() << ", wheelAngle: " << this->wheelAngle();
    ss >> msg;
    ss.str();
    return msg;
}
}