#include "Copter.h"
#include "mycontroller_usercode.h"
#include <AP_HAL/HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <AP_GPS/AP_GPS.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

float encoder_roll_feedback     = 0.0;
float encoder_pitch_feedback    = 0.0;

float encoder_roll_dot_feedback     = 0.0;
float encoder_pitch_dot_feedback    = 0.0;

float imu_roll_log      = 0.0;
float imu_pitch_log     = 0.0;
float imu_yaw_log       = 0.0;

////////////////
float H_roll_channel = 0.0;
float H_roll_channel_two_point_avg = 0.0;
float H_roll_channel_three_point_avg = 0.0;
float H_roll_channel_dot = 0.0;
float H_roll_channel_prev = 0.0;
float H_roll_channel_prev_2 = 0.0;


// const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  setup one UART at 57600
 */
// static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
// {
//     if (uart == nullptr) {
//         // that UART doesn't exist on this platform
//         return;
//     }
//     uart->begin(115200);
// }


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{

    // put your initialisation code here
    // this will be called once at start-up
    // setup_uart(hal.serial(4), "SERIAL1");  // telemetry 1
    hal.serial(1)->begin(115200);
    hal.serial(2)->begin(115200);

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    // Log_Write_position();
    // Log_Write_velocity();
    log_attitude_tracking();
    // log_sys_ID_ph_func();

    // log_DS_quad_Trans_pos();

    // hal.console->printf("Pf %d PWM1 %d PWM2 %d PWM3 %d PWM4 %d Roll %f time %f \n",Pf,PWM1,PWM2,PWM3,PWM4,imu_roll,t_ph_sys_ID);
    imu_roll_log        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch_log       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw_log         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 

    // int LED_status = light_on_off;

    // hal.serial(1)->printf("%d\n",LED_status);
    // hal.serial(2)->printf("%d\n",LED_status);

    // hal.serial(2)->printf(",");
    // hal.serial(2)->printf("%f",quad_y);
    // hal.serial(2)->printf(",");
    // hal.serial(2)->printf("%f\n",quad_z );

    // hal.serial(2)->printf("%f",z_des);
    // hal.serial(2)->printf(",");
    // hal.serial(2)->printf("%f\n",quad_z );

    //////////// For RPI 3 ////////////
    //// Data logging for system identification
    //arm_disarm_flag,Pf,Pm1,Pm2,Pm3,Pm4,PWM1,PWM2,PWM3,PWM4,ph,th,ps,z
    // hal.serial(2)->printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f\n",arm_disarm_flag,Pf,Pm1,Pm2,Pm3,Pm4,PWM1,PWM2,PWM3,PWM4,imu_roll,imu_pitch,imu_yaw,quad_z);

    //// Data logging outdoor////
    // hal.serial(2)->printf("%d",arm_disarm_flag);
    // hal.serial(2)->printf("%f,%f,%f,%f,%f,%f",quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw);
    // hal.serial(2)->printf("%f,%f,%f,%f",H_roll,H_pitch,H_yaw_rate,H_yaw);
    // hal.serial(2)->printf("%f,%f,%f,%f",F,Mb1,Mb2,Mb3);
    // hal.serial(2)->printf("\n");

    // hal.serial(2)->printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);
    // hal.console->printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);

    //////// To debug the human inputs
    Human_Joystick_data_Analysis();

}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    // //////// To debug the human inputs
    // Human_Joystick_data_Analysis();

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::log_DS_quad_Trans_pos(){

        struct log_DS_POS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DS_POS_MSG),
        time_us  : AP_HAL::micros64(),
        X     : quad_x,
        Y     : quad_y,
        Z     : quad_z,
        X_des : x_des,
        Y_des : y_des,
        Z_des : z_des,
        light : light_on_off,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));

}

void Copter::Human_Joystick_data_Analysis()
{
    float dt = 0.01;
    H_roll_channel     = (double)(channel_pitch->get_control_in())/100.0;

    // First time derivative
    H_roll_channel_dot = (H_roll_channel - H_roll_channel_prev)/dt;

    // Two point average
    H_roll_channel_two_point_avg = (H_roll_channel + H_roll_channel_prev)/2;

    // Three point average
    H_roll_channel_three_point_avg = (H_roll_channel + H_roll_channel_prev+H_roll_channel_prev_2)/3;

    hal.console->printf("%5.3f,%5.3f,%5.3f\n",H_roll_channel,H_roll_channel_two_point_avg,H_roll_channel_three_point_avg);

    H_roll_channel_prev = H_roll_channel;
    H_roll_channel_prev_2 = H_roll_channel_prev;
}

void Copter::Log_Write_position()
{
    struct log_position pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POSI_MSG),
        time_us  : AP_HAL::micros64(),
        x        : quad_x,
        y        : quad_y,
        z        : quad_z,
        phi      : imu_roll_log,
        theta    : imu_pitch_log,
        psi      : imu_yaw_log,
        phi_p    : encoder_roll_feedback,
        theta_p  : encoder_pitch_feedback,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
    // hal.console->printf("From log write position \n");

}

void Copter::Log_Write_velocity()
{
    struct log_velocity pkt = {
        LOG_PACKET_HEADER_INIT(LOG_VELO_MSG),
        time_us  : AP_HAL::micros64(),
        x_dot        : quad_x_dot,
        y_dot        : quad_y_dot,
        z_dot        : quad_z_dot,
        phi_dot      : imu_roll_dot,
        theta_dot    : imu_pitch_dot,
        psi_dot      : imu_yaw_dot,
        phi_p_dot    : encoder_roll_dot_feedback,
        theta_p_dot  : encoder_pitch_dot_feedback,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_attitude_tracking()
{
    struct log_att_trac pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATT_TRA_MSG),
        time_us  : AP_HAL::micros64(),
        phi      : imu_roll_log,
        theta    : imu_pitch_log,
        psi      : imu_yaw_log,
        phi_h    : H_roll,
        theta_h  : H_pitch,
        psi_h    : H_yaw,
        psi_h_dot: H_yaw_rate,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_sys_ID_ph_func()
{
    struct log_sys_ID_ph pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SID_PH_MSG),
        time_us  : AP_HAL::micros64(),
        PWM1     : PWM1,
        PWM2     : PWM2,
        PWM3     : PWM3,
        PWM4     : PWM4,
        Pf       : Pf,
        phi      : imu_roll_log,
        theta    : imu_pitch_log,
        psi      : imu_yaw_log,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}


void Copter::getEncoderData()
{
    bool receiving_data = false;
    int index = 0;
    char startChar = ',';
    char endChar = '/';
    bool new_data = false;

    char attitude[] = "50000_50000";
    while (hal.serial(2)->available()>0 && new_data == false)
        {
            char temp = hal.serial(2)->read();
            if (receiving_data == true)
            {
                if (temp != endChar)
                {   
                    attitude[index] = temp;
                    index++;
                }
                else
                {
                    attitude[index] = '\0';
                    receiving_data = false;
                    new_data = false;
                    index = 0;
                }
            }
            else if (temp == startChar)
            {
                receiving_data = true;
                index = 0; 
            }
        }
        // hal.uartE->printf("%s\n",attitude);

        char roll_char[]        = "11111";
        char pitch_char[]       = "11111";

        for (int i = 0; i < 11; i++)
        {
            if (i < 5)  
            {
                roll_char[i]                = attitude[i];
            } else if (i >= 6 && i < 11 )
            {
                pitch_char[i - 6]           = attitude[i];
            }
        }

        int encoder_roll_int      = atoi(roll_char);
        int encoder_pitch_int     = atoi(pitch_char);

        encoder_roll_feedback  = (float)((encoder_roll_int  - 50000.0) / 100.0);
        encoder_pitch_feedback = (float)((encoder_pitch_int - 50000.0) / 100.0);

        if (encoder_roll_feedback > 60.0){
            encoder_roll_feedback = 60.0;
        }
        if (encoder_roll_feedback < -60.0){
            encoder_roll_feedback = -60.0;
        }

        if (encoder_pitch_feedback > 60.0){
            encoder_pitch_feedback = 60.0;
        }
        if (encoder_pitch_feedback < -60.0){
            encoder_pitch_feedback = -60.0;
        }

        // hal.uartE->printf("%0.3f,", encoder_roll_feedback);
        // hal.uartE->printf("%0.3f\n", encoder_pitch_feedback);
}
