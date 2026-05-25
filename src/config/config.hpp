#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

namespace hsm
{
    struct camera_config
    {
        int  device_id;
        int  width;
        int  height;
        int  offset_x;
        int  offset_y;
        int  ADC_bit_depth;
        int  exposure;
        int  gain;
        int  balck_level;
        bool Reverse_X;
        bool Reverse_Y;
    };

    
} // namespace hsm

#endif