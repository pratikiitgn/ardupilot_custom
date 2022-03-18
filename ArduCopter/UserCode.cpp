#include "Copter.h"
#include "mycontroller_usercode.h"
#include <AP_HAL/HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <AP_GPS/AP_GPS.h>
#define PI 3.14
// static AP_HAL::OwnPtr<AP_HAL::Device> spi_dev;

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
char human_handle_IMU_data[] = "500000,500000,500000,500000,500000,500000";
char human_handle_encoder_data[] = "0,500000,500000";
char gain_data_portenta[] = "/00000,00000,00000\n";

float HH_on_off_feedback    = 0.0;
float HH_yaw_feedback       = 0.0;
float HH_pitch_feedback     = 0.0;

float HH_IMU_roll_feedback  = 0.0;
float HH_IMU_pitch_feedback = 0.0;
float HH_IMU_yaw_feedback   = 0.0;

Vector3f qp(0.0,0.0,0.0);
Vector3f qc(0.0,0.0,0.0);
Vector3f qp_prev(0.0,0.0,0.0);
Vector3f qc_prev(0.0,0.0,0.0);
Vector3f Omega_p(0.0,0.0,0.0);
Vector3f Omega_c(0.0,0.0,0.0);
Vector3f qpd(1.0,0.0,0.0);
// Vector3f qp_in_HH_frame();
// For filtering qp_dot and qc_dot

// int fil_iter_qp_dot = 20;
// float fil_qp1_dot_array[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
// float fil_qp2_dot_array[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
// float fil_qp3_dot_array[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float qp3_dot_fil1 = 0.0;
float qp3_dot_fil2 = 0.0;
float qp3_dot_fil3 = 0.0;
float qp3_dot_fil4 = 0.0;
float qp3_dot_fil5 = 0.0;
float qp3_dot_fil6 = 0.0;
float qp3_dot_fil7 = 0.0;
float qp3_dot_fil8 = 0.0;
float qp3_dot_fil9 = 0.0;
float qp3_dot_fil10 = 0.0;

float qp2_dot_fil1 = 0.0;
float qp2_dot_fil2 = 0.0;
float qp2_dot_fil3 = 0.0;
float qp2_dot_fil4 = 0.0;
float qp2_dot_fil5 = 0.0;
float qp2_dot_fil6 = 0.0;
float qp2_dot_fil7 = 0.0;
float qp2_dot_fil8 = 0.0;
float qp2_dot_fil9 = 0.0;
float qp2_dot_fil10 = 0.0;

float qp1_dot_fil1 = 0.0;
float qp1_dot_fil2 = 0.0;
float qp1_dot_fil3 = 0.0;
float qp1_dot_fil4 = 0.0;
float qp1_dot_fil5 = 0.0;
float qp1_dot_fil6 = 0.0;
float qp1_dot_fil7 = 0.0;
float qp1_dot_fil8 = 0.0;
float qp1_dot_fil9 = 0.0;
float qp1_dot_fil10 = 0.0;

float qp3_dot_fil_final = 0.0;
float qp2_dot_fil_final = 0.0;
float qp1_dot_fil_final = 0.0;

float qc1_dot_fil1 = 0.0;
float qc1_dot_fil2 = 0.0;
float qc1_dot_fil3 = 0.0;
float qc1_dot_fil4 = 0.0;
float qc1_dot_fil5 = 0.0;
float qc1_dot_fil6 = 0.0;
float qc1_dot_fil7 = 0.0;
float qc1_dot_fil8 = 0.0;
float qc1_dot_fil9 = 0.0;
float qc1_dot_fil10 = 0.0;

float qc2_dot_fil1 = 0.0;
float qc2_dot_fil2 = 0.0;
float qc2_dot_fil3 = 0.0;
float qc2_dot_fil4 = 0.0;
float qc2_dot_fil5 = 0.0;
float qc2_dot_fil6 = 0.0;
float qc2_dot_fil7 = 0.0;
float qc2_dot_fil8 = 0.0;
float qc2_dot_fil9 = 0.0;
float qc2_dot_fil10 = 0.0;

float qc1_dot_fil_final = 0.0;
float qc2_dot_fil_final = 0.0;

uint16_t PWM1 = 1000;
uint16_t PWM2 = 1000;
uint16_t PWM3 = 1000;
uint16_t PWM4 = 1000;

float F         = 0.0;
float Mb1       = 0.0;
float Mb2       = 0.0;
float Mb3       = 0.0;

float Ax = 0.0;
float Ay = 0.0;
float Az = 0.0;

// For gains receiving from portenta

// float MG = 0.0;
// float KP_gain = 25.0;
// float KD_gain = 20.0;

// float fil_H_pitch_array[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


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
    hal.serial(2)->begin(2000000);  // For human IMU data
    hal.serial(4)->begin(115200);   // For CAM device data
    hal.serial(5)->begin(115200);   // For human encoder data
    // hal.serial(1)->begin(115200);   // For getting gains values from portenta
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{

    // To access data 
    getEncoderData();
    getHumanIMUdata();
    getHumanEncoderdata();
    get_IROS_data();

    // get_Gain_data_from_portenta();
    // put your 100Hz code here
    Log_Write_position();
    Log_Write_velocity();
    log_attitude_tracking();
    log_IROS_data();
    log_IROS_raw_data();
    log_IROS_HH_acc();
    log_IROS_ATT_Track();

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
    // int check = 0;
    // if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
    //     check = 1;
    // }
    // hal.serial(2)->printf("%1d,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%1.0f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f\n",check,imu_roll,imu_pitch,imu_yaw,encoder_roll_feedback,encoder_pitch_feedback,HH_on_off_feedback,HH_yaw_feedback,HH_pitch_feedback,HH_IMU_roll_feedback,HH_IMU_pitch_feedback,HH_IMU_yaw_feedback);
    // 1+7+7+7+6+6+1+7+7+7+7+7+11

    // hal.console->printf("%10.3f, %10.3f\n",imu_yaw, 180+HH_IMU_yaw_feedback);
    // hal.console->printf("%1d,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%1.0f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f\n",check,imu_roll,imu_pitch,imu_yaw,encoder_roll_feedback,encoder_pitch_feedback,HH_on_off_feedback,HH_yaw_feedback,HH_pitch_feedback,HH_IMU_roll_feedback,HH_IMU_pitch_feedback,HH_IMU_yaw_feedback);

    // hal.console->printf("Pf %d PWM1 %d PWM2 %d PWM3 %d PWM4 %d Roll %f time %f \n",Pf,PWM1,PWM2,PWM3,PWM4,imu_roll,t_ph_sys_ID);

    imu_roll_log        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch_log       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw_log         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
 

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
    // hal.serial(1)->printf("Pratik\n");

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

void Copter::get_Gain_data_from_portenta()
{
    // bool receiving_data = false;
    // int index = 0;
    // char startChar = '/';
    // char endChar = '\n';
    // bool new_data = false;

    // while (hal.serial(1)->available()>0 && new_data == false)
    //     {
    //         char temp = hal.serial(1)->read();
    //         // hal.console->printf("%c\n",temp);
    //         if (receiving_data == true)
    //         {
    //             if (temp != endChar)
    //             {   
    //                 gain_data_portenta[index] = temp;
    //                 index++;
    //             }
    //             else
    //             {
    //                 gain_data_portenta[index] = '\0';
    //                 receiving_data = false;
    //                 new_data = false;
    //                 index = 0;
    //             }
    //         }
    //         else if (temp == startChar)
    //         {
    //             receiving_data = true;
    //             index = 0; 
    //         }

    //     }

    //     // hal.console->printf("%s\n",gain_data_portenta);

    //     char MG_array[]         = "101275";
    //     char KP_gain_array[]    = "101500";
    //     char KD_gain_array[]    = "109000";

    //     for (int i = 0; i < 20; i++)
    //     {
    //         if (i < 6)  
    //         {
    //             MG_array[i]             = gain_data_portenta[i];
    //         } else if (i >= 7 && i < 13 )
    //         {
    //             KP_gain_array[i - 7]    = gain_data_portenta[i];
    //         } else if (i >=14 && i < 20)
    //         {
    //             KD_gain_array[i-14]     = gain_data_portenta[i];
    //         }
    //     }

        // int MG_int = atoi(MG_array);
        // int KP_int = atoi(KP_gain_array);
        // int KD_int = atoi(KD_gain_array);

        // Pratik if you want to use these code please make a global variables

        // float MG        = (float) ((MG_int - 100000)/100.0);
        // float KP_gain   = (float) ((KP_int - 100000)/100.0);
        // float KD_gain   = (float) ((KD_int - 100000)/100.0);

        // hal.console->printf("MG--> %f, KP--> %f, KD -> %f\n",MG,KP_gain,KD_gain);

        // float max_lim_MG = 180;

        // if (HH_on_off_feedback > 1){HH_on_off_feedback = 1;}
        // if (HH_on_off_feedback < 0){HH_on_off_feedback = 0;}

        // if (HH_yaw_feedback > max_lim){HH_yaw_feedback = max_lim;}
        // if (HH_yaw_feedback < -max_lim){HH_yaw_feedback = -max_lim;}

        // if (HH_pitch_feedback > max_lim){HH_pitch_feedback = max_lim;}
        // if (HH_pitch_feedback < -max_lim){HH_pitch_feedback = -max_lim;}

}

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

        int HH_on_off_int   = atoi(on_off);
        int HH_yaw_int      = atoi(encoder_yaw);
        int HH_pitch_int    = atoi(encoder_pitch);

        HH_on_off_feedback  =  (float)(HH_on_off_int);
        HH_yaw_feedback     =  (float)((HH_yaw_int - 500000.0) / 100.0);
        HH_pitch_feedback   =  (float)((HH_pitch_int - 500000.0) / 100.0);

        float max_lim = 180;

        if (HH_on_off_feedback > 1){HH_on_off_feedback = 1;}
        if (HH_on_off_feedback < 0){HH_on_off_feedback = 0;}

        if (HH_yaw_feedback > max_lim){HH_yaw_feedback = max_lim;}
        if (HH_yaw_feedback < -max_lim){HH_yaw_feedback = -max_lim;}

        if (HH_pitch_feedback > max_lim){HH_pitch_feedback = max_lim;}
        if (HH_pitch_feedback < -max_lim){HH_pitch_feedback = -max_lim;}


    // hal.console->printf("Encoder data --> %s",human_handle_encoder_data);
    // hal.console->printf(" OF -> %1.1f, Yaw -> %4.2f, Pitch-> %4.2f\n",HH_on_off_feedback,HH_yaw_feedback,HH_pitch_feedback);

}

void Copter::get_IROS_data(){

    Vector3f HH_rpy(HH_IMU_roll_feedback*PI/180.0,HH_IMU_pitch_feedback*PI/180.0,HH_IMU_yaw_feedback*PI/180.0);
    Vector3f e_1(1,0,0);

    Matrix3f R_H(eulerAnglesToRotationMatrix(HH_rpy));
    
    // hal.console->printf("%5.2f,%5.2f,%5.2f\n",HH_IMU_vector[0],HH_IMU_vector[1],HH_IMU_vector[2]);

    // Calculate rotation about pitch axis of human handle
    // HH_pitch_feedback = 0.0;
    // HH_yaw_feedback   = 0.0;
    Matrix3f HH_R_y (
               cosf(HH_pitch_feedback*PI/180.0),    0,      sinf(HH_pitch_feedback*PI/180.0),
               0,               1,      0,
               -sinf(HH_pitch_feedback*PI/180.0),   0,      cosf(HH_pitch_feedback*PI/180.0)
               );

    Matrix3f HH_R_y_IMU_pitch (
               cosf((HH_pitch_feedback + HH_IMU_pitch_feedback)*PI/180.0),    0,      sinf((HH_pitch_feedback + HH_IMU_pitch_feedback)*PI/180.0),
               0,               1,      0,
               -sinf((HH_pitch_feedback + HH_IMU_pitch_feedback)*PI/180.0),   0,      cosf((HH_pitch_feedback + HH_IMU_pitch_feedback)*PI/180.0)
               );

    // Calculate rotation about yaw axis of human handle
    Matrix3f HH_R_z (
              cosf(HH_yaw_feedback*PI/180),    -sinf(HH_yaw_feedback*PI/180),      0,
               sinf(HH_yaw_feedback*PI/180),    cosf(HH_yaw_feedback*PI/180),      0,
               0,               0,                  1);

    Matrix3f HH_IMU_R_z (
              cosf(HH_IMU_yaw_feedback*PI/180.0),    -sinf(HH_IMU_yaw_feedback*PI/180.0),      0,
               sinf(HH_IMU_yaw_feedback*PI/180.0),    cosf(HH_IMU_yaw_feedback*PI/180.0),      0,
               0,               0,                  1);

    Matrix3f HH_IMU_R_y (
               cosf(HH_IMU_pitch_feedback*PI/180.0),    0,      sinf(HH_IMU_pitch_feedback*PI/180.0),
               0,               1,      0,
               -sinf(HH_IMU_pitch_feedback*PI/180.0),   0,      cosf(HH_IMU_pitch_feedback*PI/180.0)
               );

    // Vector3f HH_IMU_vector(Matrix_vector_mul(HH_IMU_R_z,Matrix_vector_mul(HH_IMU_R_y,Matrix_vector_mul(HH_R_z,Matrix_vector_mul(HH_R_y,e_1)))));
    Vector3f HH_CAM_device_vec(Matrix_vector_mul(HH_R_z,Matrix_vector_mul(HH_R_y,e_1)));
    Vector3f HH_IMU_vector(Matrix_vector_mul(R_H,HH_CAM_device_vec));
    // qp_in_HH_frame

    // hal.console->printf("%5.2f,%5.2f,%5.2f   ",HH_IMU_vector[0],HH_IMU_vector[1],HH_IMU_vector[2]);
    // hal.console->printf("%5.2f,%5.2f\n",HH_IMU_pitch_feedback,HH_pitch_feedback);
    // hal.console->printf("%f\n", cosf(PI));
    // qp = Matrix_vector_mul(R_H,(Matrix_vector_mul(HH_R_z,Matrix_vector_mul(HH_R_y,e_1))));
    // qp =  Matrix_vector_mul(HH_R_y,Matrix_vector_mul(HH_R_z,Matrix_vector_mul(R_H,e_1)));
    // qp =  Matrix_vector_mul(R_H,e_1);
    // qp =  Matrix_vector_mul(HH_R_z,Matrix_vector_mul(HH_R_y,Matrix_vector_mul(R_H,e_1)));

    qp = Matrix_vector_mul(HH_R_z,Matrix_vector_mul(HH_R_y,e_1));

    float qpd1 = cosf(HH_IMU_yaw_feedback*PI/180);
    float qpd2 = sinf(HH_IMU_yaw_feedback*PI/180);

    qpd[0] = qpd1;
    qpd[1] = qpd2;
    qpd[2] = 0.0;

    // Vector3f qpd_HH_frame(1.0,0.0,0.0);
    // Vector3f delta_qp = Matrix_vector_mul(hatmap(qpd_HH_frame),qp);
    // HH_pitch_feedback = HH_pitch_feedback;
    // if (HH_IMU_pitch_feedback > 0) {
    //     HH_IMU_pitch_feedback = 0.98*HH_IMU_pitch_feedback;
    // }
    // HH_IMU_pitch_feedback = HH_IMU_pitch_feedback + 7.2;
    // Debugging for IMU pitch angle and human handle pitch angle
    // hal.console->printf("%6.2f,%6.2f,%6.2f\n",HH_pitch_feedback, HH_IMU_pitch_feedback ,HH_pitch_feedback + HH_IMU_pitch_feedback);

    // hal.console->printf("%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n",qp[0],qpd[0],qp[1],qpd[1],qp[2],qpd[2]);
    // hal.console->printf("%5.2f,%5.2f,%5.2f\n",delta_qp[0],delta_qp[1],delta_qp[2]);
    // hal.console->printf("%5.2f,%5.2f,%5.2f \n",qp[0],qp[1],qp[2]);
    // hal.console->printf("%5.2f,%5.2f,%5.2f,   ",qpd[0],qpd[1],qpd[2]);
    // hal.console->printf("%5.2f,%5.2f,%5.2f,   ",delta_qp[0],delta_qp[1],delta_qp[2]);
    // hal.console->printf("%5.2f,%5.2f,   ",HH_pitch_feedback,HH_yaw_feedback);
    // hal.console->printf("%5.2f,%5.2f,%5.2f\n",HH_IMU_roll_feedback,HH_IMU_pitch_feedback,HH_IMU_yaw_feedback);

    Vector3f qp_dot = (qp - qp_prev);

    qp3_dot_fil10 = qp3_dot_fil9;
    qp3_dot_fil9 = qp3_dot_fil8;
    qp3_dot_fil8 = qp3_dot_fil7;    
    qp3_dot_fil7 = qp3_dot_fil6;
    qp3_dot_fil6 = qp3_dot_fil5;
    qp3_dot_fil5 = qp3_dot_fil4;
    qp3_dot_fil4 = qp3_dot_fil3;
    qp3_dot_fil3 = qp3_dot_fil2;
    qp3_dot_fil2 = qp3_dot_fil1;
    qp3_dot_fil1 = qp_dot[2];

    qp2_dot_fil10 = qp2_dot_fil9;
    qp2_dot_fil9 = qp2_dot_fil8;
    qp2_dot_fil8 = qp2_dot_fil7;    
    qp2_dot_fil7 = qp2_dot_fil6;
    qp2_dot_fil6 = qp2_dot_fil5;
    qp2_dot_fil5 = qp2_dot_fil4;
    qp2_dot_fil4 = qp2_dot_fil3;
    qp2_dot_fil3 = qp2_dot_fil2;
    qp2_dot_fil2 = qp2_dot_fil1;
    qp2_dot_fil1 = qp_dot[1];

    qp1_dot_fil10 = qp1_dot_fil9;
    qp1_dot_fil9 = qp1_dot_fil8;
    qp1_dot_fil8 = qp1_dot_fil7;    
    qp1_dot_fil7 = qp1_dot_fil6;
    qp1_dot_fil6 = qp1_dot_fil5;
    qp1_dot_fil5 = qp1_dot_fil4;
    qp1_dot_fil4 = qp1_dot_fil3;
    qp1_dot_fil3 = qp1_dot_fil2;
    qp1_dot_fil2 = qp1_dot_fil1;
    qp1_dot_fil1 = qp_dot[0];

    qp3_dot_fil_final = (qp3_dot_fil1 + qp3_dot_fil2 + qp3_dot_fil3 + qp3_dot_fil4 + qp3_dot_fil5 + qp3_dot_fil6 + qp3_dot_fil7 + qp3_dot_fil8 + qp3_dot_fil9 + qp3_dot_fil10)/10.0;
    qp2_dot_fil_final = (qp2_dot_fil1 + qp2_dot_fil2 + qp2_dot_fil3 + qp2_dot_fil4 + qp2_dot_fil5 + qp2_dot_fil6 + qp2_dot_fil7 + qp2_dot_fil8 + qp2_dot_fil9 + qp2_dot_fil10)/10.0;
    qp1_dot_fil_final = (qp1_dot_fil1 + qp1_dot_fil2 + qp1_dot_fil3 + qp1_dot_fil4 + qp1_dot_fil5 + qp1_dot_fil6 + qp1_dot_fil7 + qp1_dot_fil8 + qp1_dot_fil9 + qp1_dot_fil10)/10.0;
    // hal.console->printf("%10.5f,%10.5f,%10.5f,%10.5f\n",qp[2],100*qp3_dot_fil_final,qp[1],100*qp2_dot_fil_final);
    // hal.console->printf("%10.5f,%10.5f\n",qp[2],100*qp3_dot_fil_final);

    // qp3_dot_fil1 = qp_dot[2];
    // qp3_dot_fil2 = qp3_dot_fil1;
    // qp3_dot_fil3 = qp3_dot_fil2;
    // qp3_dot_fil4 = qp3_dot_fil3;
    // qp3_dot_fil5 = qp3_dot_fil4;

/////////// To filter qp_dot (short code)

    // for (int k = (fil_iter_qp_dot); k < 0 ; --k){
    //     if (k == (fil_iter_qp_dot)){
    //         fil_qp1_dot_array[k] =  qp_dot[0];
    //         fil_qp2_dot_array[k] =  qp_dot[1];
    //         fil_qp3_dot_array[k] =  qp_dot[2];
    //     }else{
    //         fil_qp1_dot_array[k] =  fil_qp1_dot_array[k-1];
    //         fil_qp2_dot_array[k] =  fil_qp2_dot_array[k-1];
    //         fil_qp3_dot_array[k] =  fil_qp3_dot_array[k-1];
    //     }
    // }

    // float fil_qp1_dot_sum = 0.0;
    // float fil_qp2_dot_sum = 0.0;
    // float fil_qp3_dot_sum = 0.0;

    // for (int ii = 0; ii < (fil_iter_qp_dot) ; ++ii){
    //     fil_qp1_dot_sum = fil_qp1_dot_array[ii] + fil_qp1_dot_sum;
    //     fil_qp2_dot_sum = fil_qp2_dot_array[ii] + fil_qp2_dot_sum; 
    //     fil_qp3_dot_sum = fil_qp3_dot_array[ii] + fil_qp3_dot_sum; 
    //     // hal.console->printf("%10.5f\n",fil_qp1_dot_sum);
    // }

    // // // qp_dot(fil_qp1_dot_sum/fil_iter_qp_dot,fil_qp2_dot_sum/fil_iter_qp_dot,fil_qp3_dot_sum/fil_iter_qp_dot);
    // // // qp_dot(fil_qp1_dot_sum,fil_qp2_dot_sum,fil_qp3_dot_sum);

    // // // float qp_dot1_fil = fil_qp1_dot_sum / fil_iter_qp_dot;
    // // // float qp_dot2_fil = fil_qp2_dot_sum / fil_iter_qp_dot;
    // float qp_dot3_fil = fil_qp3_dot_sum / fil_iter_qp_dot;

    // hal.console->printf("%10.5f,%10.5f,%10.5f\n",qp[2],100*qp_dot[2],100*qp_dot3_fil);

//////////////////// To filter short code  ////////////////////

    Omega_p = Matrix_vector_mul(hatmap(qp),qp_dot);
    qp_prev  = qp;

    // hal.console->printf("%10.5f,%10.5f\n",qp[2],fil_qp3_dot_array[0]);

// hal.console->printf("(%3.2f,%3.2f,%3.2f), (%3.2f,%3.2f), (%3.2f,%3.2f,%3.2f) \n",qp[0],qp[1],qp[2],HH_yaw_feedback,HH_pitch_feedback,HH_IMU_roll_feedback,HH_IMU_pitch_feedback,HH_IMU_yaw_feedback);

    Vector3f rpy(imu_roll*PI/180.0,imu_pitch*PI/180.0,0*PI/180.0);
    Vector3f e_3_neg(0,0,-1);
    Matrix3f R(eulerAnglesToRotationMatrix(rpy));

    // Calculate rotation about pitch axis of CAM device
    Matrix3f CAM_R_y (
               cosf(encoder_pitch_feedback*PI/180),    0,      sinf(encoder_pitch_feedback*PI/180),
               0,               1,      0,
               -sinf(encoder_pitch_feedback*PI/180),   0,      cosf(encoder_pitch_feedback*PI/180)
               );

    // Calculate rotation about roll axis of CAM device
    Matrix3f CAM_R_x (
               1,       0,              0,
               0,       cosf(encoder_roll_feedback*PI/180),   -sinf(encoder_roll_feedback*PI/180),
               0,       sinf(encoder_roll_feedback*PI/180),   cosf(encoder_roll_feedback*PI/180)
               );

    qc = Matrix_vector_mul(R,Matrix_vector_mul(CAM_R_x,Matrix_vector_mul(CAM_R_y,e_3_neg)));
    Vector3f qc_dot = (qc - qc_prev);

    qc1_dot_fil10 = qc1_dot_fil9;
    qc1_dot_fil9 = qc1_dot_fil8;
    qc1_dot_fil8 = qc1_dot_fil7;    
    qc1_dot_fil7 = qc1_dot_fil6;
    qc1_dot_fil6 = qc1_dot_fil5;
    qc1_dot_fil5 = qc1_dot_fil4;
    qc1_dot_fil4 = qc1_dot_fil3;
    qc1_dot_fil3 = qc1_dot_fil2;
    qc1_dot_fil2 = qc1_dot_fil1;
    qc1_dot_fil1 = qc_dot[0];

    qc2_dot_fil10 = qc2_dot_fil9;
    qc2_dot_fil9 = qc2_dot_fil8;
    qc2_dot_fil8 = qc2_dot_fil7;    
    qc2_dot_fil7 = qc2_dot_fil6;
    qc2_dot_fil6 = qc2_dot_fil5;
    qc2_dot_fil5 = qc2_dot_fil4;
    qc2_dot_fil4 = qc2_dot_fil3;
    qc2_dot_fil3 = qc2_dot_fil2;
    qc2_dot_fil2 = qc2_dot_fil1;
    qc2_dot_fil1 = qc_dot[1];

    qc1_dot_fil_final = (qc1_dot_fil1 + qc1_dot_fil2 + qc1_dot_fil3 + qc1_dot_fil4 + qc1_dot_fil5 + qc1_dot_fil6 + qc1_dot_fil7 + qc1_dot_fil8 + qc1_dot_fil9 + qc1_dot_fil10)/10.0;
    qc2_dot_fil_final = (qc2_dot_fil1 + qc2_dot_fil2 + qc2_dot_fil3 + qc2_dot_fil4 + qc2_dot_fil5 + qc2_dot_fil6 + qc2_dot_fil7 + qc2_dot_fil8 + qc2_dot_fil9 + qc2_dot_fil10)/10.0;

    // hal.console->printf("%10.5f,%10.5f\n",qc[0],100*qc1_dot_fil_final);
    // hal.console->printf("%10.5f,%10.5f\n",qc[1],100*qc2_dot_fil_final);

    Omega_c = Matrix_vector_mul(hatmap(qc),qc_dot);
    qc_prev  = qc;
}

void Copter::log_IROS_data(){

    struct log_iros_data pkt = {
    LOG_PACKET_HEADER_INIT(LOG_IROS_MSG),
    time_us  : AP_HAL::micros64(),
    qp1      : qp[0],
    qp2      : qp[1],
    qp3      : qp[2],
    qc1      : qc[0],
    qc2      : qc[1],
    qc3      : qc[2],
    OO       : constant_mg_IROS,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_IROS_raw_data(){

    struct log_iros_raw_data_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_IROS_RAW_MSG),
    time_us  : AP_HAL::micros64(),
    HH_ph    : HH_IMU_roll_feedback,
    HH_th    : HH_IMU_pitch_feedback,
    HH_ps    : HH_IMU_yaw_feedback,
    HH_E_th  : HH_pitch_feedback,
    HH_E_ps  : HH_yaw_feedback,
    On_Off   : HH_on_off_feedback,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_IROS_HH_acc(){

    struct log_iros_HH_acc_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_IROS_HH_ACC_MSG),
    time_us  : AP_HAL::micros64(),
    Axx      : Ax,
    Ayy      : Ay,
    Azz      : Az,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_IROS_ATT_Track(){

    struct log_iros_att_track_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_IROS_ATT_TRACK_MSG),
    time_us  : AP_HAL::micros64(),
    qp1d      : qpd[0],
    qp2d      : qpd[1],
    qp3d      : qpd[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

Vector3f Copter::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}

Matrix3f Copter::eulerAnglesToRotationMatrix(Vector3f rpy){
     // Calculate rotation about x axis
    Matrix3f R_x (
               1,       0,              0,
               0,       cosf(rpy[0]),   -sinf(rpy[0]),
               0,       sinf(rpy[0]),   cosf(rpy[0])
               );

    // Calculate rotation about y axis
    Matrix3f R_y (
               cosf(rpy[1]),    0,      sinf(rpy[1]),
               0,               1,      0,
               -sinf(rpy[1]),   0,      cosf(rpy[1])
               );

    // Calculate rotation about z axis
    Matrix3f R_z (
               cosf(rpy[2]),    -sinf(rpy[2]),      0,
               sinf(rpy[2]),    cosf(rpy[2]),       0,
               0,               0,                  1);

    // Combined rotation matrix
    Matrix3f R = R_z * R_y * R_x;
    return R;
}


Matrix3f Copter::hatmap(Vector3f v){
    Matrix3f R (
               0,         -v[2],      v[1],
               v[2],         0 ,     -v[0],
               -v[1],      v[0],       0);
    return R;
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

        // hal.console->printf("%s\n",human_handle_IMU_data);

        char HH_IMU_roll_char[]     = "500000";
        char HH_IMU_pitch_char[]    = "500000";
        char HH_IMU_yaw_char[]      = "500000";

        char HH_AX_char[]      = "500000";
        char HH_AY_char[]      = "500000";
        char HH_AZ_char[]      = "500000";

        for (int i = 0; i < 41; i++)
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
            } else if (i >=21 && i < 27)
            {
                HH_AX_char[i-21]       = human_handle_IMU_data[i];
            } else if (i >=28 && i < 34)
            {
                HH_AY_char[i-28]       = human_handle_IMU_data[i];
            } else if (i >=35 && i < 41)
            {
                HH_AZ_char[i-35]       = human_handle_IMU_data[i];
            }
        }

        int HH_IMU_roll_int  = atoi(HH_IMU_roll_char);
        int HH_IMU_pitch_int = atoi(HH_IMU_pitch_char);
        int HH_IMU_yaw_int   = atoi(HH_IMU_yaw_char);

        int HH_AX_int        = atoi(HH_AX_char);
        int HH_AY_int        = atoi(HH_AY_char);
        int HH_AZ_int        = atoi(HH_AZ_char);

        HH_IMU_roll_feedback  = (float)((HH_IMU_roll_int - 500000.0) / 100.0);
        HH_IMU_pitch_feedback = (float)((HH_IMU_pitch_int - 500000.0) / 100.0);
        HH_IMU_yaw_feedback   = 180.0 + (float)((HH_IMU_yaw_int - 500000.0) / 100.0);
        Ax = (float) ((HH_AX_int - 500000.0) /100.0);
        Ay = (float) ((HH_AY_int - 500000.0) /100.0);
        Az = (float) ((HH_AZ_int - 500000.0) /100.0);

        // hal.console->printf("%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n",HH_IMU_roll_feedback,HH_IMU_pitch_feedback,HH_IMU_yaw_feedback,Ax,Ay,Az);

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

        encoder_roll_feedback  =  (float)((encoder_roll_int  - 50000.0) / 100.0);
        encoder_pitch_feedback =  (float)((encoder_pitch_int - 50000.0) / 100.0);

        encoder_pitch_feedback = encoder_pitch_feedback + 28.13;
        encoder_roll_feedback  = encoder_roll_feedback  + 62.93;

        // hal.console->printf("%5.2f,%5.2f\n",encoder_roll_feedback,encoder_pitch_feedback);

        if (encoder_roll_feedback > 65.0){
            encoder_roll_feedback = 65.0;
        }
        if (encoder_roll_feedback < -65.0){
            encoder_roll_feedback = -65.0;
        }

        if (encoder_pitch_feedback > 65.0){
            encoder_pitch_feedback = 65.0;
        }
        if (encoder_pitch_feedback < -65.0){
            encoder_pitch_feedback = -65.0;
        }

        // hal.console->printf("%0.3f,", encoder_roll_feedback);
        // hal.console->printf("%0.3f\n", encoder_pitch_feedback);
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