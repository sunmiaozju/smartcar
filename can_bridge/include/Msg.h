/*
 * @Description: some import msgs declaration
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:48:52
 * @LastEditTime: 2019-03-26 19:41:14
 */

#ifndef CANLIB_MSG_H
#define CANLIB_MSG_H

#include "utils.h"
#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint-gcc.h>
#include <string.h>
#include <string>

#define BUFFER_SIZE 8

namespace CanBridge {

class Serializable {
public:
    virtual void serialize(struct can_frame& frame) = 0;
};

enum ShiftLevel {
    NOT_DETECTED = 0,
    D = 1,
    N = 2,
    R = 3
};

enum DriveMode {
    AUTO_MODE = 1,
    PAD_MODE = 2,
    WHEEL_MODE = 3
};

class SendMsg : public Serializable {
private:
    uint8_t packet[BUFFER_SIZE];

public:
    SendMsg()
    {
        bzero(packet, sizeof(packet));
        this->setGradient(0);
        this->setAccMode(1);
        this->setDriveMode(AUTO_MODE);
        this->setBrakeMode(1);
    }

    SendMsg(uint8_t* data)
    {
        bzero(packet, sizeof(packet));
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            this->packet[i] = data[i];
        }
    }

    void print();

    void setDriveMode(DriveMode driveMode);

    // DriveMode getDriveMode();

    void setBrakeMode(int v);

    // int getBrakeMode();

    void setShiftLevel(ShiftLevel shiftLevel);

    // ShiftLevel getShiftLevel();

    void setSpeed(double v);

    // double getSpeed();

    void setGradient(double v);

    // double getGradient();

    void setEBrake(bool need = false);

    // int getEBrake();

    void setWheelAngle(double angle);

    // double getWheelAngle();

    void setAccMode(int v);

    // int getAccMode();

    void serialize(struct can_frame& frame);

    std::string toString();
};

class RevMsg {
    uint8_t packet[BUFFER_SIZE];

public:
    RevMsg()
    {
        bzero(packet, sizeof(packet));
    }

    RevMsg(uint8_t* data)
    {
        bzero(packet, sizeof(packet));
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            this->packet[i] = data[i];
        }
    }

    void print();

    int shiftLevel();

    double Speed();

    double wheelAngle();

    std::string toString();
};
}
#endif // !CANLIB_MSG_H