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
#define PI 3.14


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

Vector3f qc(0.0,0.0,0.0);
Vector3f Omega_c(0.0,0.0,0.0);
Vector3f qc_prev(0.0,0.0,0.0);
Vector3f qc_dot(0.0,0.0,0.0);

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
    hal.serial(2)->begin(115200);
    hal.serial(4)->begin(115200);

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    Log_Write_position();
    Log_Write_velocity();
    log_attitude_tracking();
    log_sys_ID_ph_func();

    // hal.console->printf("Pf %d PWM1 %d PWM2 %d PWM3 %d PWM4 %d Roll %f time %f \n",Pf,PWM1,PWM2,PWM3,PWM4,imu_roll,t_ph_sys_ID);
    
    
    imu_roll_log        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch_log       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw_log         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 

    // hal.console->printf("From usercode \n");
    getEncoderData();
    Drone_geometric_data();
    // hal.console->printf("ph_p %f, th_p - %f\n",encoder_roll_feedback,encoder_pitch_feedback); 

    // char H_roll_[5]    = "";
    // char H_pitch_[5]   = "";
    // char imu_roll_[5]  = "";
    // char imu_pitch_[5] = "";

    // int imu_roll_         = imu_roll*100;
    // int H_roll_           = H_roll*100;
    // int H_pitch_          = H_pitch*100;
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
    // hal.serial(2)->printf("%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);
    // hal.serial(2)->printf("%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",arm_disarm_flag,quad_x,quad_y,quad_z,x_des,y_des,z_des,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);

    // hal.console->printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);

}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here








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
        // hal.uartE->printf("%s\n",attitude);
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

        // hal.console->printf("phi_p -> %f, theta_p -> %f\n",encoder_roll_feedback,encoder_pitch_feedback);

}

void Copter::Drone_geometric_data(){

    // hal.console->printf("%5.2f,%5.2f,%5.2f   ",HH_IMU_vector[0],HH_IMU_vector[1],HH_IMU_vector[2]);
    // hal.console->printf("%5.2f,%5.2f\n",HH_IMU_pitch_feedback,HH_pitch_feedback);
    // hal.console->printf("%f\n", cosf(PI));
    Vector3f rpy_quad(imu_roll_log*PI/180.0,imu_pitch_log*PI/180.0,imu_yaw_log*PI/180.0);
    Matrix3f R_quad(eulerAnglesToRotationMatrix(rpy_quad));
    
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
    Vector3f e_3_neg(0,0,-1);

    qc = -Matrix_vector_mul(R_quad,Matrix_vector_mul(CAM_R_x,Matrix_vector_mul(CAM_R_y,e_3_neg)));
    // qc   = Matrix_vector_mul(CAM_R_x,Matrix_vector_mul(CAM_R_y,e_3_neg));
    
    Vector3f b1_quad(Matrix_vector_mul(R_quad,e1_earth));
    qc_dot = (qc - qc_prev)*10.0;

    // hal.console->printf("%10.5f, %10.5f, %10.5f, %10.5f \n",qc[0],qc[1],qc[2],imu_yaw_log);

    // hal.console->printf("%10.5f, %10.5f\n",qc[1],qc_dot[1]);

    // hal.console->printf("%10.5f, %10.5f, %10.5f, %10.5f \n",b1_quad[0],b1_quad[1],b1_quad[2],imu_yaw_log);
    // hal.console->printf("%10.5f,%10.5f,%10.5f\n",imu_roll,imu_pitch,imu_yaw);
    // hal.console->printf("%10.5f,%10.5f\n",qc[1],100*qc2_dot_fil_final);

    Omega_c = Matrix_vector_mul(hatmap(qc),qc_dot);
    // hal.console->printf("%10.5f, %10.5f\n",qc[0],Omega_c[1]);
    qc_prev  = qc;
}

Vector3f Copter::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}


Matrix3f Copter::hatmap(Vector3f v){
    Matrix3f R (
               0,         -v[2],      v[1],
               v[2],         0 ,     -v[0],
               -v[1],      v[0],       0);
    return R;
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