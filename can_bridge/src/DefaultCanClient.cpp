/*
 * @Description: default can client implement 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 19:41:40
 * @LastEditTime: 2019-04-02 16:43:40
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

int DefaultCanClient::registerListener(CANListener* listener)
{
    if (listener == NULL) {
        return NULL_VALUE;
    }
    auto ret = this->listeners.insert(listener);
    if (!ret.second) {
        return REPEATED_KEY;
    }
    return SUCCESS;
}

int DefaultCanClient::removeListener(CANListener* listener)
{
    if (listener == NULL) {
        return NULL_VALUE;
    }
    this->listeners.erase(listener);
    return SUCCESS;
}

void DefaultCanClient::start()
{
    pthread_t threadID;
    bzero(&threadID, sizeof(pthread_t));
    pthread_create(&threadID, NULL, DefaultCanClient::start_client_thread, (void*)this);
    if (threadID == 0) {
        fprintf(stderr, "start thread error!");
        return;
    }
    this->mThread = threadID;
}

void* DefaultCanClient::start_client_thread(void* args)
{
    assert(args != NULL);
    DefaultCanClient* pClient = (DefaultCanClient*)args;
    pClient->run();
}

void DefaultCanClient::run()
{
    fd_set readSet;
    fd_set writeSet;
    struct can_filter rfilter[3];
    // filter info, accept 0x51 only.
    rfilter[0].can_id = 0x51; // 车辆状态信息
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 1000; // 毫米波雷达信息 暂时保留
    rfilter[1].can_mask = CAN_SFF_MASK;
    rfilter[2].can_id = 0x02AA; // 电池状态信息
    rfilter[2].can_mask = CAN_SFF_MASK;
    setsockopt(this->canFd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    this->running = 1;
    while (true) {
        if (interrupted) {
            break;
        }
        FD_ZERO(&readSet);
        FD_ZERO(&writeSet);
        FD_SET(this->canFd, &readSet);
        if (!this->queue.empty()) {
            FD_SET(this->canFd, &writeSet);
        }
        timeval timeout;
        bzero(&timeout, sizeof(timeval));
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;
        int ret = select(this->canFd + 1, &readSet, &writeSet, NULL, &timeout);
        if (ret == 0) {
            continue;
        }

        if (FD_ISSET(this->canFd, &readSet)) {
            struct can_frame canFrame;
            int nbytes = read(this->canFd, &canFrame, sizeof(can_frame));
            if (nbytes < 0) {
                fprintf(stderr, "read can error\n");
                break;
            }
            if (nbytes < sizeof(can_frame)) {
                fprintf(stderr, "incomplete can_frame\n");
                continue;
            }

            //车辆状态信息监听
            if (canFrame.can_id == 0x51) {
                RevMsg msg(canFrame.data);
                for (auto e = listeners.begin(); e != listeners.end(); ++e) {
                    CANListener* canListener = (*e);
                    canListener->onReceiveMsg(&msg);
                }
            }

            //电池状态信息监听
            if (canFrame.can_id == 0x02AA) {
                BatteryMsg msg(canFrame.data);
                for (auto e = listeners.begin(); e != listeners.end(); ++e) {
                    CANListener* canListener = (*e);
                    canListener->onReceiveMsg(&msg);
                }
            }
            // if (canFrame.can_id == 1000) {
            //     MillimeterWaveRadar msg(canFrame.data);
            //     for (auto e = listeners.begin(); e != listeners.end(); ++e) {
            //         CANListener* canListener = (*e);
            //         canListener->onReceiveMsg(&msg);
            //     }
            // }
        }
        if (FD_ISSET(this->canFd, &writeSet)) {
            void* ptr = NULL;
            if (this->queue.pop(&ptr)) {
                struct can_frame* pFrame = (struct can_frame*)ptr;
                int nbytes = write(this->canFd, pFrame, sizeof(can_frame));
                if (nbytes != sizeof(can_frame)) {
                    fprintf(stderr, "send error frame ID = 0x%X DLC=%d data = 0x%X\n", pFrame->can_id, pFrame->can_dlc, pFrame->data);
                    break;
                }
                free(pFrame);
            }
        }
    }
    this->running = -1;
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

int DefaultCanClient::writeSendMsg(SendMsg* pMsg)
{
    if (pMsg == NULL) {
        fprintf(stderr, "parameter error pMsg must not be null");
        return NULL_VALUE;
    }
    offer(pMsg);
    return 0;
}

int DefaultCanClient::offer(Serializable* pMsg)
{
    if (pMsg == NULL) {
        fprintf(stderr, "paramter error msg must not be null");
        return NULL_VALUE;
    }

    struct can_frame* pFrame = (struct can_frame*)malloc(sizeof(struct can_frame));
    if (pFrame == NULL) {
        fprintf(stderr, "malloc can_frame error");
        return ALLOCATE_ERROR;
    }
    pMsg->serialize(*pFrame);
    int cnt = 0;
    while (cnt < 10) {
        if (this->queue.push(pFrame)) {
            return SUCCESS;
        }
        cnt++;
    }
    return FAILED;
}
}