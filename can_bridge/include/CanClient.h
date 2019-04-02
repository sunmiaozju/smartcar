/*
 * @Description: basic class CANListener and CANClient
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:25:24
 * @LastEditTime: 2019-04-02 16:37:39
 */

#include "Msg.h"
#include <bits/time.h>
#include <set>
#include <string>

namespace CanBridge {

class CANListener {
public:
    virtual void onReceiveMsg(RevMsg* msg) = 0;

    virtual void onReceiveMsg(BatteryMsg* msg) = 0;

    virtual ~CANListener() = default;
};

class CANClient {
public:
    static CANClient* creatInstance(std::string devname);

    static void disposal(CANClient* pClient);

    virtual ~CANClient() = default;

    /**
     * @description: support for many listener 
     */
    virtual int registerListener(CANListener* pListener) = 0;

    virtual int removeListener(CANListener* listener) = 0;

    /**
     * @description: start client 
     */
    virtual void start() = 0;

    virtual void shutdown() = 0;

    virtual int writeSendMsg(SendMsg* pMsg) = 0;

    // virtual int writeSendMsg(SendMsg* pMsg, timeval* timeout) = 0;
};
}