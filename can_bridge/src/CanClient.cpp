/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:21:23
 * @LastEditTime: 2019-03-25 20:51:40
 */

#include "DefaultCanClient.h"

namespace CanBridge {
CANClient* CANClient::creatInstance(std::string devname)
{
    DefaultCanClient* pClient = new DefaultCanClient();
    pClient->setDevname(devname);
    int ret = pClient->init();
    if (ret == SUCCESS) {
        return pClient;
    }
    delete pClient;
    return nullptr;
}

void CANClient::disposal(CANClient* pClient)
{
    if (pClient != NULL) {
        delete pClient;
    }
}
}