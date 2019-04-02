/*
 * @Description: msg implementation
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-26 10:53:55
 * @LastEditTime: 2019-04-02 15:43:59
 */

#include "Msg.h"

namespace CanBridge {
/**
 * @description: 设置驾驶模式 
 */
void SendMsg::setDriveMode(DriveMode driveMode)
{
    writeInt(this->packet, SCU_DRIVE_MODE_REQ_OFFSET, SCU_DRIVE_MODE_REQ_LENGTH, driveMode);
}
/**
 * @description: 设置加速档位 
 */
void SendMsg::setAccMode(int v)
{
    writeInt(this->packet, SCU_ACC_MODE_OFFSET, SCU_ACC_MODE_LENGTH, v);
}
/**
 * @description: 设置减速档位 
 */
void SendMsg::setBrakeMode(int v)
{
    writeInt(this->packet, SCU_BRAKE_MODE_OFFSET, SCU_BRAKE_MODE_LENGTH, v);
}
/**
 * @description: 设置目标速度 
 */
void SendMsg::setSpeed(double v)
{
    int speed = (int)lround(v * 10);
    writeInt(this->packet, SCU_TARGET_SPEED_OFFSET, SCU_TARGET_SPEED_LENGTH, speed);
}
/**
 * @description: 设置转向角度 
 */
void SendMsg::setWheelAngle(double angle)
{
    short v = angle * 10;
    writeInt(this->packet, SCU_STEERING_WHEEL_ANGLE_OFFSET, SCU_STEERING_WHEEL_ANGLE_LENGTH, v);
}
/**
 * @description: 设置车辆档位 
 */
void SendMsg::setShiftLevel(ShiftLevel shift_level)
{
    writeInt(this->packet, SCU_SHIFT_LEVEL_OFFSET, SCU_SHIFT_LEVEL_LENGTH, shift_level);
}
/**
 * @description:设置路面坡度 
 */
void SendMsg::setGradient(double v)
{
    int iv = v * 2; // 因为can里面gradient的单位是0.5度
    writeInt(this->packet, SCU_GRADIENT_OFFSET, SCU_GRADIENT_LENGTH, iv);
}
/**
 * @description: 设置紧急制动 
 */
void SendMsg::setEBrake(bool need)
{
    int v = 0;
    if (need) {
        v = 1;
    }
    writeInt(this->packet, SCU_EBRAKE_OFFSET, SCU_EBRAKE_LENGTH, v);
}

void SendMsg::serialize(struct can_frame& frame)
{
    bzero(&frame, sizeof(struct can_frame));
    frame.can_dlc = 8;
    for (size_t i = 0; i < PAYLOAD_LENGTH; i++) {
        frame.data[i] = packet[i];
    }
    frame.can_id = 0x120; // 这是can协议里面的 scu_auto_cantrol 的 can_id ,具体可参考docs下的can协议文件说明
}
/**
 * @description: 获取当前档位
 */
int RevMsg::shiftLevel()
{
    return readAsInt(this->packet, CCU_SHIFT_LEVEL_OFFSET, CCU_SHIFT_LEVEL_LENGTH);
}
/**
 * @description: 获取当前速度 
 */
double RevMsg::speed()
{
    double speed = readAsInt(this->packet, CCU_SPEED_OFFSET, CCU_SPEED_LENGTH) / 10.0; // 单位 m/s
    return speed;
}
/**
 * @description: 获取当前转向方向 
 */
SteerDirection RevMsg::wheelDirection()
{
    int dir = readBit(this->packet, CCU_WHEEL_DIRECTION_OFFSET);
    if (dir == 0)
        return left;
    return right;
}
/**
 * @description: 获取当前转向角度 
 */
double RevMsg::wheelAngle()
{
    double angle = readAsInt(this->packet, CCU_WHEEL_ANGLE_OFFSET, CCU_WHEEL_ANGLE_LEGNTH) / 10.0; // 单位： 度
    return angle;
}
/**
 * @description: 获取当前驾驶模式 
 */
int RevMsg::driveMode()
{
    return readAsInt(this->packet, CCU_DRIVE_MODE_OFFSET, CCU_DRIVE_MODE_LENGTH);
}
/**
 * @description: 获取当前加速档位 
 */
int RevMsg::accLevel()
{
    return readAsInt(this->packet, CCU_ACCELERATE_LEVEL_OFFSET, CCU_ACCELERATE_LEVEL_LENGTH);
}
/**
 * @description: 获取当前减速档位 
 */
int RevMsg::brakeLevel()
{
    return readAsInt(this->packet, CCU_BRAKE_LEVEL_OFFSET, CCU_BRAKE_LEVEL_LENGTH);
}
/**
 * @description: 获取当前车辆累计里程数 
 */
double RevMsg::totalOdometer()
{
    double total_odometer = readAsInt(this->packet, CCU_TOTAL_ODOMETER_OFFSET, CCU_TOTAL_ODOMETER_LENGTH);
    return total_odometer;
}
/**
 * @description:  输出为字符串 
 */
std::string RevMsg::toString()
{
    std::string msg;
    std::stringstream ss;
    ss << "RevMsg speed: " << this->speed() << ", shiftLevel: " << this->shiftLevel() << ", wheelAngle: " << this->wheelAngle()
       << ", driveMode: " << this->driveMode() << ", accLevel: " << this->accLevel() << ", brakeLevel: " << this->brakeLevel()
       << ", totalOdometer: " << this->totalOdometer();
    ss >> msg;
    ss.str();
    return msg;
}
/**
 * @description: 获取动力电池电压 
 */
double BatteryMsg::getVoltage()
{
    return readAsInt(this->packet, BMU_VOLTAGE_OFFSET, BMU_VOLTAGE_LENGTH) * 0.1;
}
/**
 * @description: 获取动力电池电流
 */
double BatteryMsg::getAmpere()
{
    return (readAsInt(this->packet, BMU_AMPERE_OFFSET, BMU_AMPERE_LENGTH) - 4000) * 0.1;
}
/**
 * @description: 获取动力电池点量 
 */
double BatteryMsg::getBatteryCapacity()
{
    return readAsInt(this->packet, BMU_CAPACITY_OFFSET, BMU_CAPACITY_LENGTH) * 0.4;
}
/**
 * @description: 获取电池管理模块状态
 */
int BatteryMsg::getBsuSysStatus()
{
    return readAsInt(this->packet, BMU_SYS_STATUS_OFFSET, BMU_SYS_STATUS_LENGTH);
}
/**
 * @description: 获取充电状态 
 */
int BatteryMsg::getChargeStatus()
{
    return readBit(this->packet, BMU_CHARGE_STATUS_OFFSET);
}
}