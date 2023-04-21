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

float encoder_roll_feedback = 0.0;
float encoder_pitch_feedback = 0.0;

float encoder_roll_dot_feedback = 0.0;
float encoder_pitch_dot_feedback = 0.0;

float imu_roll_log = 0.0;
float imu_pitch_log = 0.0;
float imu_yaw_log = 0.0;

char sz[20];

char attitude[] = "50001_50000";

uint16_t PWM1 = 1000;
uint16_t PWM2 = 1000;
uint16_t PWM3 = 1000;
uint16_t PWM4 = 1000;

float F = 0.0;
float Mb1 = 0.0;
float Mb2 = 0.0;
float Mb3 = 0.0;

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
    hal.serial(4)->begin(115200); // For CAM device data
    // hal.serial(1)->begin(115200);   // For getting gains values from portenta
    hal.console->printf("Hi\n");
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{

    // To access data
    getEncoderData();
    Log_Write_position();
    Log_Write_velocity();
    log_attitude_tracking();

    imu_roll_log = (ahrs.roll_sensor) / 100.0;       // degrees
    imu_pitch_log = -(ahrs.pitch_sensor) / 100.0;    // degrees
    imu_yaw_log = 360.0 - (ahrs.yaw_sensor) / 100.0; // degrees

    // hal.console->printf("Hi\n");

    //////////// For RPI 3 ////////////
    //// Data logging for system identification
    // arm_disarm_flag,Pf,Pm1,Pm2,Pm3,Pm4,PWM1,PWM2,PWM3,PWM4,ph,th,ps,z
    //  hal.serial(2)->printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f\n",arm_disarm_flag,Pf,Pm1,Pm2,Pm3,Pm4,PWM1,PWM2,PWM3,PWM4,imu_roll,imu_pitch,imu_yaw,quad_z);

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



Vector3f Copter::Matrix_vector_mul(Matrix3f R_quad, Vector3f v_quad)
{
    Vector3f mul_vector(
        R_quad[0][0] * v_quad[0] + R_quad[0][1] * v_quad[1] + R_quad[0][2] * v_quad[2],
        R_quad[1][0] * v_quad[0] + R_quad[1][1] * v_quad[1] + R_quad[1][2] * v_quad[2],
        R_quad[2][0] * v_quad[0] + R_quad[2][1] * v_quad[1] + R_quad[2][2] * v_quad[2]);
    return mul_vector;
}

Matrix3f Copter::eulerAnglesToRotationMatrix(Vector3f rpy_func)
{
    // Calculate rotation about x axis
    Matrix3f R_x(
        1, 0, 0,
        0, cosf(rpy_func[0]), -sinf(rpy_func[0]),
        0, sinf(rpy_func[0]), cosf(rpy_func[0]));

    // Calculate rotation about y axis
    Matrix3f R_y(
        cosf(rpy_func[1]), 0, sinf(rpy_func[1]),
        0, 1, 0,
        -sinf(rpy_func[1]), 0, cosf(rpy_func[1]));

    // Calculate rotation about z axis
    Matrix3f R_z(
        cosf(rpy_func[2]), -sinf(rpy_func[2]), 0,
        sinf(rpy_func[2]), cosf(rpy_func[2]), 0,
        0, 0, 1);

    // Combined rotation matrix
    Matrix3f R_func = R_z * R_y * R_x;

    return R_func;
}

Matrix3f Copter::hatmap(Vector3f v)
{
    Matrix3f R_from_hatmap(
        0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0);
    return R_from_hatmap;
}

void Copter::getEncoderData()
{
    bool receiving_data = false;
    int index = 0;
    char startChar = ',';
    char endChar = '/';
    bool new_data = false;

    while (hal.serial(4)->available() > 0 && new_data == false)
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

    char roll_char[] = "11111";
    char pitch_char[] = "11111";

    for (int i = 0; i < 11; i++)
    {
        if (i < 5)
        {
            roll_char[i] = attitude[i];
        }
        else if (i >= 6 && i < 11)
        {
            pitch_char[i - 6] = attitude[i];
        }
    }

    int encoder_roll_int = atoi(roll_char);
    int encoder_pitch_int = atoi(pitch_char);

    encoder_roll_feedback = (float)((encoder_roll_int - 50000.0) / 100.0);
    encoder_pitch_feedback = (float)((encoder_pitch_int - 50000.0) / 100.0);

    encoder_pitch_feedback = encoder_pitch_feedback + 2.64;
    encoder_roll_feedback = encoder_roll_feedback - 2.0;

    // hal.console->printf("%5.2f,%5.2f\n",encoder_roll_feedback,encoder_pitch_feedback);

    if (encoder_roll_feedback > 65.0)
    {
        encoder_roll_feedback = 65.0;
    }
    if (encoder_roll_feedback < -65.0)
    {
        encoder_roll_feedback = -65.0;
    }

    if (encoder_pitch_feedback > 65.0)
    {
        encoder_pitch_feedback = 65.0;
    }
    if (encoder_pitch_feedback < -65.0)
    {
        encoder_pitch_feedback = -65.0;
    }

    imu_roll_log = (ahrs.roll_sensor) / 100.0;       // degrees
    imu_pitch_log = -(ahrs.pitch_sensor) / 100.0;    // degrees
    // imu_yaw_log = 360.0 - (ahrs.yaw_sensor) / 100.0; // degrees

    // hal.console->printf("%0.3f,", imu_roll_log);
    hal.console->printf("%0.3f,", encoder_roll_feedback);
    // hal.console->printf("%0.3f,", imu_pitch_log);
    hal.console->printf("%0.3f\n", encoder_pitch_feedback);
}

void Copter::gains_data_from_Rpi()
{

}

void Copter::Log_Write_position()
{
    struct log_position pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POSI_MSG),
        time_us : AP_HAL::micros64(),
        x : quad_x,
        y : quad_y,
        z : quad_z,
        phi : rpy[0],
        theta : rpy[1],
        psi : rpy[2],
        phi_p : encoder_roll_feedback,
        theta_p : encoder_pitch_feedback,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_Write_velocity()
{
    struct log_velocity pkt = {
        LOG_PACKET_HEADER_INIT(LOG_VELO_MSG),
        time_us : AP_HAL::micros64(),
        x_dot : quad_x_dot,
        y_dot : quad_y_dot,
        z_dot : quad_z_dot,
        phi_dot : Omega[0],
        theta_dot : Omega[1],
        psi_dot : Omega[2],
        phi_p_dot : encoder_roll_dot_feedback,
        theta_p_dot : encoder_pitch_dot_feedback,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_attitude_tracking()
{
    struct log_att_trac pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATT_TRA_MSG),
        time_us : AP_HAL::micros64(),
        phi : rpy[0],
        theta : rpy[1],
        psi : rpy[2],
        phi_h : H_roll,
        theta_h : H_pitch,
        psi_h : H_yaw,
        psi_h_dot : H_yaw_rate,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}