#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

class AP_Portenta
{

public:
        void start();

private:

        bool init();
        uint8_t _register_read(uint8_t reg);
        bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
};
