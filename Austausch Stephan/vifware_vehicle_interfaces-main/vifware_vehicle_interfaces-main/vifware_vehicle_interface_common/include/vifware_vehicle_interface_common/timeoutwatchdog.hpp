#pragma once

#include<bitset>
#include<functional>

namespace vifware_vehicle_interfaces {
    
    class TimeoutWatchdog
    {   
    public:
        void init(unsigned int timeout,
                  std::function<void()> timout_cb,
                  std::function<void()> cleared_cb = []{});

        void update(unsigned int recv);
        void poll(unsigned int now);

    private:
        unsigned int timeout_{ 0 };
        unsigned int last_recv_{ 0 };
        bool timeouted_ { false };
        std::function<void()> timeout_cb_{ []{} };
        std::function<void()> cleared_cb_{ []{} };
    };

} // namespace vifware_vehicle_interfaces