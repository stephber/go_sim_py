#include <vifware_vehicle_interface_common/timeoutwatchdog.hpp>

using namespace vifware_vehicle_interfaces;

void TimeoutWatchdog::init(unsigned int timeout,
                           std::function<void()> timeout_cb,
                           std::function<void()> cleared_cb)
{
    timeout_ = timeout;
    timeout_cb_ = timeout_cb;
    cleared_cb_ = cleared_cb;
}

void TimeoutWatchdog::update(unsigned int recv)
{
    last_recv_ = recv;
}

void TimeoutWatchdog::poll(unsigned int now)
{
    // call timeout callback if timeouted at every poll
    if ((now - last_recv_) > timeout_) {
        timeouted_ = true;
        timeout_cb_();
    }
    // call cleared callback once,
    else if (timeouted_) {
        timeouted_ = false;
        cleared_cb_();
    }
}
