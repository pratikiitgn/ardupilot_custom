#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h>
#include "mycontroller_usercode.h"
#include <AP_GPS/AP_GPS.h>

bool ModeMyController::init(bool)
{
        return true;

}

void ModeMyController::run()
{
    // apply simple mode transform to pilot input

}