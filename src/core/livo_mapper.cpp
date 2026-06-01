#include <thread>
#include <chrono>

#include "core/livo_mapper.hpp"

using namespace hsm;

void livo_mapper::run()
{
    while (true)
    {
        if (! this->sync_packages())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        this->handle_first_frame();
        processImu();
        stateEstimationAndMapping();
    }
    savePCD();
}

void livo_mapper::handle_first_frame()
{
    static bool is_first_frame = false;
    if (! is_first_frame)
    {
        this->_first_lidar_time = this->lidar_measures.last_lio_update_time;
        p_imu->first_lidar_time = _first_lidar_time;
        is_first_frame          = true;
    }
}