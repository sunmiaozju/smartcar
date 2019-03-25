/*
 * @Description: default can client implement 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 19:41:40
 * @LastEditTime: 2019-03-25 21:47:53
 */
#include "DefaultCanClient.h"
#include <cassert>
#include <cstdio>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>

namespace CanBridge {
DefaultCanClient::DefaultCanClient()
{
    this->interrupted = 0;
    this->canFd = -1;
    this->running = 0;
}

void DefaultCanClient::setDevname(std::string devname)
{
    this->devname = devname;
}

int DefaultCanClient::init()
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    this->canFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, this->devname.c_str());
    if (ioctl(this->canFd, SIOCGIFINDEX, &ifr) == -1) {
        return DEVICE_NOT_FOUND;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(this->canFd, (struct sockaddr*)&addr, sizeof(addr));
    return SUCCESS;
}

void DefaultCanClient::shutdown()
{
    this->interrupted = 1;
    void* ptr;
    pthread_join(this->mThread, &ptr);
}

DefaultCanClient::~DefaultCanClient()
{
    if (this->canFd > 0) {
        close(this->canFd);
    }
    if (this->running == 1) {
        shutdown();
        pthread_join(this->mThread, NULL);
    }
}
}