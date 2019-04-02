/*
 * @Description: some import msgs declaration
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:48:52
 * @LastEditTime: 2019-04-02 16:32:57
 */

#ifndef CANLIB_MSG_H
#define CANLIB_MSG_H

#include "utils.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sstream>
#include <stdint-gcc.h>
#include <string.h>
#include <string>

// reveive vehicle status msg protocol
#define CCU_SHIFT_LEVEL_OFFSET 0
#define CCU_SHIFT_LEVEL_LENGTH 2 // 档位  1：D  2：N  3：R
#define CCU_WHEEL_DIRECTION_OFFSET 7 // 实际方向盘角度方向 0左边 1右边
#define CCU_WHEEL_ANGLE_OFFSET 8
#define CCU_WHEEL_ANGLE_LEGNTH 12 // 实际方向盘角度 单位 0.1度  最大值 1200 代表 120度
#define CCU_SPEED_OFFSET 20
#define CCU_SPEED_LENGTH 9 // 当前车速 单位 0.1m/s 最大值 510 代表 51m/s
#define CCU_DRIVE_MODE_OFFSET 29
#define CCU_DRIVE_MODE_LENGTH 3 // 当前驾驶模式 0不影响  1自动驾驶模式  2驾驶员PAD模式  3驾驶员方向盘模式\
                                   自动驾驶模式的优先级低于 方向盘模式 也就是 在自动驾驶模式下，调整控制版，车辆\
                                   优先采取控制版的命令
#define CCU_ACCELERATE_LEVEL_OFFSET 32
#define CCU_ACCELERATE_LEVEL_LENGTH 2 // 当前加速档位
#define CCU_BRAKE_LEVEL_OFFSET 34
#define CCU_BRAKE_LEVEL_LENGTH 2 // 当前减速档位
#define CCU_TOTAL_ODOMETER_OFFSET 36
#define CCU_TOTAL_ODOMETER_LENGTH 20 // 车辆累计里程数 单位：km

// reveive vehicle battery msg protocol
#define BMU_VOLTAGE_OFFSET 0
#define BMU_VOLTAGE_LENGTH 16 // 动力电池 电压 单位：V Physical voltage= BSU_Battery_Out_Voltage * 0.1
#define BMU_AMPERE_OFFSET 16
#define BMU_AMPERE_LENGTH 16 // 动力电池 电流 单位：A Physical current= (BSU_Battery_Out_Current - 4000) * 0.1
#define BMU_CAPACITY_OFFSET 32
#define BMU_CAPACITY_LENGTH 8 // 动力电池 电量 单位：百分比 Physical current= BSU_SysSOC  *  0.4%
#define BMU_SYS_STATUS_OFFSET 40
#define BMU_SYS_STATUS_LENGTH 2 // BSU 系统状态：  0 正常 \
                                                 1 不影响车辆正常行驶的故障\
                                                 2 影响车辆正常行使，需要驾驶员限制驾驶的故障\
                                                 3 驾驶员需要立即停车，并请求救援的故障

#define BMU_CHARGE_STATUS_OFFSET 42 // BSU 充电状态： 0未充电  1正在充电

// send vehicle control msg protocol
#define SCU_SHIFT_LEVEL_OFFSET 0
#define SCU_SHIFT_LEVEL_LENGTH 2 // 车辆档位请求： 0 为检测到或初始状态  1：D 2：N 3：R
#define SCU_ACC_MODE_OFFSET 2
#define SCU_ACC_MODE_LENGTH 2 // 加速模式  0不影响 1：加速1 2：加速2 3：加速3
#define SCU_BRAKE_MODE_OFFSET 4
#define SCU_BRAKE_MODE_LENGTH 2 // 制动模式 0：不影响 1：制动1 2：制动2 3：制动3
#define SCU_DRIVE_MODE_REQ_OFFSET 6
#define SCU_DRIVE_MODE_REQ_LENGTH 2 // 驾驶模式请求： 0：不影响 1：自动驾驶模式请求  2：驾驶员PAD模式请求  3：驾驶员方向盘模式请求
#define SCU_STEERING_WHEEL_ANGLE_OFFSET 8
#define SCU_STEERING_WHEEL_ANGLE_LENGTH 16 // 请求方向盘转角信号
#define SCU_TARGET_SPEED_OFFSET 24
#define SCU_TARGET_SPEED_LENGTH 9 // 目标车速请求
#define SCU_EBRAKE_OFFSET 33
#define SCU_EBRAKE_LENGTH 1 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define SCU_GRADIENT_OFFSET 34
#define SCU_GRADIENT_LENGTH 6 // 道路倾斜坡度设置  -15度～+15度 单位 0.5度

#define PAYLOAD_LENGTH 8

#define BUFFER_SIZE 8

namespace CanBridge {

class Serializable {
public:
    virtual void serialize(struct can_frame& frame) = 0;
};

enum ShiftLevel {
    NOT_DETECTED = 0,
    D = 1, // 前进
    N = 2, // 停车
    R = 3 // 倒车
};

enum DriveMode {
    AUTO_MODE = 1,
    PAD_MODE = 2,
    WHEEL_MODE = 3
};

enum SteerDirection {
    left = 0,
    right = 1
};

class SendMsg : public Serializable {
private:
    uint8_t packet[BUFFER_SIZE];

public:
    SendMsg()
    {
        bzero(packet, sizeof(packet));
        this->setGradient(0); // 设置路面坡度
        this->setAccMode(1); // 设置加速档位
        this->setDriveMode(AUTO_MODE); // 设置驾驶模式 ： 自动驾驶
        this->setBrakeMode(1); // 设置减速档位
    }

    SendMsg(uint8_t* data)
    {
        bzero(packet, sizeof(packet));
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            this->packet[i] = data[i];
        }
    }

    void setDriveMode(DriveMode driveMode);

    void setBrakeMode(int v);

    void setAccMode(int v);

    void setShiftLevel(ShiftLevel shiftLevel);

    void setSpeed(double v);

    void setWheelAngle(double angle);

    void setGradient(double v);

    void setEBrake(bool need = false);

    void serialize(struct can_frame& frame);
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

    double speed();

    double wheelAngle();

    SteerDirection wheelDirection();

    int driveMode();

    int accLevel();

    int brakeLevel();

    double totalOdometer();

    std::string toString();
};

class BatteryMsg {
private:
    uint8_t packet[BUFFER_SIZE];

public:
    BatteryMsg()
    {
        bzero(packet, sizeof(packet));
    }

    BatteryMsg(uint8_t* data)
    {
        bzero(packet, sizeof(packet));
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            packet[i] = data[i];
        }
    }

    double getVoltage();

    double getAmpere();

    double getBatteryCapacity();

    int getBsuSysStatus();

    int getChargeStatus();
};
}
#endif // !CANLIB_MSG_H