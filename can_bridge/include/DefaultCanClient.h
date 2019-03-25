/*
 * @Description:  can client specific implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 17:14:07
 * @LastEditTime: 2019-03-25 22:23:01
 */
#include "CanClient.h"
#include "ErrorCode.h"
#include "Msg.h"
#include <boost/lockfree/spsc_queue.hpp>
#include <set>

namespace CanBridge {
class DefaultCanClient : public CANClient {
private:
    pthread_t mThread;
    volatile int interrupted;
    boost::lockfree::spsc_queue<void*, boost::lockfree::capacity<100>> queue;
    std::string devname;
    int canFd;
    int running;

    std::set<CANListener*> listeners;

    int offer(Serializable* ptr);

    int put(Serializable* ptr, timeval* timeout);

    static void* start_client_thread(void* args);

public:
    DefaultCanClient();

    ~DefaultCanClient();

    int init();

    void setDevname(std::string devname);

    int registerListener(CANListener* listener) override;

    void start() override;

    int removeListener(CANListener* listener) override;

    void shutdown() override;

    int writeSendMsg(SendMsg* pMsg) override;

    int writeSendMsg(SendMsg* pMsg, timeval* timeout) override;

protected:
    void run();
};
}
