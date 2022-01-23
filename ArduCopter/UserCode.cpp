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

static AP_HAL::OwnPtr<AP_HAL::Device> spi_dev;

#include <utility>
#include <stdio.h>

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

char sz[20];

char attitude[] = "50001_50000";
char human_handle_IMU_data[] = "500000,500000,500000";
char human_handle_encoder_data[] = "0,500000,500000";

float HH_on_off_feedback    = 0.0;
float HH_yaw_feedback       = 0.0;
float HH_pitch_feedback     = 0.0;

float HH_IMU_roll_feedback  = 0.0;
float HH_IMU_pitch_feedback = 0.0;
float HH_IMU_yaw_feedback   = 0.0;

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
    hal.serial(2)->begin(2000000);
    hal.serial(4)->begin(115200);
    hal.serial(5)->begin(115200);


}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{

    // To access the CAM device data
    getEncoderData();

    getHumanIMUdata();
    getHumanEncoderdata();

    // Portenta_data();
    // hal.serial(2)->printf("%1d,%6.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%4d,%4d,%4d,%4d_",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw,H_yaw_rate,H_throttle,PWM1,PWM2,PWM3,PWM4);

    //// uncomment following code for the data sending for quadcopter+payload system outdoor
    // hal.serial(2)->printf("%1d,%5.2f,%5.2f,%5.2f,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%4d,%4d,%4d,%4d,%6.2f,%6.2f_",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw,H_yaw_rate,H_throttle,PWM1,PWM2,PWM3,PWM4,encoder_roll_feedback,encoder_pitch_feedback);

    //// [start] uncomment following code for CAM device validations
    // int check = 0;
    // if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
    //     check = 1;
    // }

    // hal.serial(2)->printf("%1d,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f_",check,imu_roll,imu_pitch,imu_yaw,encoder_roll_feedback,encoder_pitch_feedback);

    //// [IROS'22] uncomment following code for CAM device validations
    int check = 0;
    if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
        check = 1;
    }

    hal.serial(1)->printf("%1d,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%1.0f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f_",check,imu_roll,imu_pitch,imu_yaw,encoder_roll_feedback,encoder_pitch_feedback,HH_on_off_feedback,HH_yaw_feedback,HH_pitch_feedback,HH_IMU_roll_feedback,HH_IMU_pitch_feedback,HH_IMU_yaw_feedback);
    // 1+7+7+7+6+6+1+7+7+7+7+7+11

    // hal.console->printf("Roll %f, pith %f\n", encoder_roll_feedback, encoder_pitch_feedback);

    ////////////////////// For SPI communicatio
    // spi_dev->read_registers(reg, buf, size);

    // spi_dev->read_registers(MPUREG_WHOAMI, &whoami, 1);
    // printf("20789 SPI whoami: 0x%02x\n", whoami);


    // char temp = hal.serial(2)->read();
    // hal.console->printf("%s\n",temp);


    // gains_data_from_Rpi();

    // put your 100Hz code here
    // Log_Write_position();
    // Log_Write_velocity();
    // log_attitude_tracking();
    // log_sys_ID_ph_func();

    // hal.console->printf("Pf %d PWM1 %d PWM2 %d PWM3 %d PWM4 %d Roll %f time %f \n",Pf,PWM1,PWM2,PWM3,PWM4,imu_roll,t_ph_sys_ID);
    
    
    // imu_roll_log        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    // imu_pitch_log       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    // imu_yaw_log         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
 

    // hal.console->printf("From usercode \n");
    // getEncoderData();
    // hal.console->printf("ph_p %f, th_p - %f\n",encoder_roll_feedback,encoder_pitch_feedback); 

    // char H_roll_[5]    = "";
    // char H_pitch_[5]   = "";
    // char imu_roll_[5]  = "";
    // char imu_pitch_[5] = "";

    // int imu_roll_         = imu_roll*100;
    // int H_roll_           = H_roll*100;
    // int H_pitch_        = H_pitch*100;
    // int imu_roll_dot_       = imu_roll_dot*100;
    // int imu_pitch_      = imu_pitch*100;

// if (H_roll_ > 4500){
//     H_roll_ = 4500;
// }

// if (H_roll_ < -4500){
//     H_roll_ = -4500;
// }

// if (imu_roll_ > 4500){
//     imu_roll_ = 4500;
// }

// if (imu_roll_ < -4500){
//     imu_roll_ = -4500;
// }


    // hal.serial(2)->printf("%f",quad_x);
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
    // hal.serial(2)->printf("%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",arm_disarm_flag,quad_x,quad_y,quad_z,x_des,y_des,z_des,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);

    //// Geometric controller ////

    //// For rotation matrix
    // hal.serial(2)->printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",arm_disarm_flag,R_log[0][0],R_log[0][1],R_log[0][2],R_log[1][0],R_log[1][1],R_log[1][2],R_log[2][0],R_log[2][1],R_log[2][2]);

    //// For e_R_log
    // hal.serial(2)->printf("%f,%f,%f\n",e_R_log[0],e_R_log[1],e_R_log[2]);

    // For e_R_log and e_Omega
    // hal.serial(2)->printf("%d,%f,%f,%f,%f,%f,%f\n",arm_disarm_flag,e_R_log[0],e_R_log[1],e_R_log[2],e_Omega_log[0],e_Omega_log[1],e_Omega_log[2]);

    // For R and Rd
    // hal.serial(2)->printf("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",arm_disarm_flag,R_log[0][0],R_log[0][1],R_log[0][2],R_log[1][0],R_log[1][1],R_log[1][2],R_log[2][0],R_log[2][1],R_log[2][2],Rd_log[0][0],Rd_log[0][1],Rd_log[0][2],Rd_log[1][0],Rd_log[1][1],Rd_log[1][2],Rd_log[2][0],Rd_log[2][1],Rd_log[2][2]);

    // For desired euler angles and actual euler angles
    // hal.serial(2)->printf("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",arm_disarm_flag,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw);
    // hal.serial(2)->printf("Pratik\n");

    // For portenta 
    // hal.spi->get_device
    // _dev
    
    //// Mav proxy debugging
    // hal.console->printf("roll-> %f, pitch-> %f, yaw-> %f, roll_c-> %f, pitch_c-> %f \n",imu_roll,imu_pitch,imu_yaw,encoder_roll_feedback,encoder_pitch_feedback);


}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    // hal.serial(2)->printf("%1d,%6.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%4d,%4d,%4d,%4d_",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw,H_yaw_rate,H_throttle,PWM1,PWM2,PWM3,PWM4);








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

void Copter::getHumanEncoderdata()
{
    bool receiving_data = false;
    int index = 0;
    char startChar = '/';
    char endChar = '_';
    bool new_data = false;

    while (hal.serial(5)->available()>0 && new_data == false)
        {
            char temp = hal.serial(5)->read();
            if (receiving_data == true)
            {
                if (temp != endChar)
                {   
                    human_handle_encoder_data[index] = temp;
                    index++;
                }
                else
                {
                    human_handle_encoder_data[index] = '\0';
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

        char on_off[] = "0";
        char encoder_yaw[]      = "500000";
        char encoder_pitch[]    = "500000";

        for (int i = 0; i < 15; i++)
        {
            if (i < 1)  
            {
                on_off[i]                = human_handle_encoder_data[i];
            } else if (i >= 2 && i < 8 )
            {
                encoder_yaw[i - 2]       = human_handle_encoder_data[i];
            } else if (i >=9 && i < 15)
            {
                encoder_pitch[i-9]       = human_handle_encoder_data[i];
            }
        }

        int HH_on_off_int          = atoi(on_off);
        int HH_yaw_int     = atoi(encoder_yaw);
        int HH_pitch_int   = atoi(encoder_pitch);

        HH_on_off_feedback  =  (float)(HH_on_off_int);
        HH_yaw_feedback     =  (float)((HH_yaw_int - 500000.0) / 100.0);
        HH_pitch_feedback   = -(float)((HH_pitch_int - 500000.0) / 100.0);

        float max_lim = 180;

        if (HH_on_off_feedback > 1){HH_on_off_feedback = 1;}
        if (HH_on_off_feedback < 0){HH_on_off_feedback = 0;}

        if (HH_yaw_feedback > max_lim){HH_yaw_feedback = max_lim;}
        if (HH_yaw_feedback < -max_lim){HH_yaw_feedback = -max_lim;}

        if (HH_pitch_feedback > max_lim){HH_pitch_feedback = max_lim;}
        if (HH_pitch_feedback < -max_lim){HH_pitch_feedback = -max_lim;}

    hal.console->printf("Encoder data --> %s",human_handle_encoder_data);
    hal.console->printf(" OF -> %1.1f, Yaw -> %4.2f, Pitch-> %4.2f\n",HH_on_off_feedback,HH_yaw_feedback,HH_pitch_feedback);

}

void Copter::getHumanIMUdata()
{
    bool receiving_data = false;
    int index = 0;
    char startChar = '/';
    char endChar = '_';
    bool new_data = false;

    while (hal.serial(2)->available()>0 && new_data == false)
        {
            char temp = hal.serial(2)->read();
            if (receiving_data == true)
            {
                if (temp != endChar)
                {   
                    human_handle_IMU_data[index] = temp;
                    index++;
                }
                else
                {
                    human_handle_IMU_data[index] = '\0';
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

        char HH_IMU_roll_char[]     = "500000";
        char HH_IMU_pitch_char[]    = "500000";
        char HH_IMU_yaw_char[]      = "500000";

        for (int i = 0; i < 20; i++)
        {
            if (i < 6)  
            {
                HH_IMU_roll_char[i]         = human_handle_IMU_data[i];
            } else if (i >= 7 && i < 13 )
            {
                HH_IMU_pitch_char[i - 7]    = human_handle_IMU_data[i];
            } else if (i >=14 && i < 20)
            {
                HH_IMU_yaw_char[i-14]       = human_handle_IMU_data[i];
            }
        }

        int HH_IMU_roll_int  = atoi(HH_IMU_roll_char);
        int HH_IMU_pitch_int = atoi(HH_IMU_pitch_char);
        int HH_IMU_yaw_int   = atoi(HH_IMU_yaw_char);

        HH_IMU_roll_feedback  = (float)((HH_IMU_roll_int - 500000.0) / 100.0);
        HH_IMU_pitch_feedback = (float)((HH_IMU_pitch_int - 500000.0) / 100.0);
        HH_IMU_yaw_feedback   = (float)((HH_IMU_yaw_int - 500000.0) / 100.0);

        float max_lim = 90;

        if (HH_IMU_roll_feedback > max_lim){HH_IMU_roll_feedback = max_lim;}
        if (HH_IMU_roll_feedback < -max_lim){HH_IMU_roll_feedback = -max_lim;}

        if (HH_IMU_pitch_feedback > max_lim){HH_IMU_pitch_feedback = max_lim;}
        if (HH_IMU_pitch_feedback < -max_lim){HH_IMU_pitch_feedback = -max_lim;}

        // if (HH_pitch_feedback > max_lim){HH_pitch_feedback = max_lim;}
        // if (HH_pitch_feedback < -max_lim){HH_pitch_feedback = -max_lim;}

    // hal.console->printf("Encoder data --> %s",human_handle_IMU_data);
    // hal.console->printf(" R->  %4.2f, P-> %4.2f, Y-> %4.2f\n",HH_IMU_roll_feedback,HH_IMU_pitch_feedback,HH_IMU_yaw_feedback);



    // hal.console->printf("IMU_data --> %s\n",human_handle_IMU_data);
}

void Copter::getEncoderData()
{
    bool receiving_data = false;
    int index = 0;
    char startChar = ',';
    char endChar = '/';
    bool new_data = false;

    
    while (hal.serial(4)->available()>0 && new_data == false)
        {
            char temp = hal.serial(4)->read();
            
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

        // hal.console->printf("%s\n",attitude);

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


void Copter::gains_data_from_Rpi(){

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
