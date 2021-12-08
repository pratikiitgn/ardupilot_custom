
#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h>
#include "mycontroller_usercode.h"
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  


#define ESC_HZ 490
// Hello 

int code_starting_flag = 0;

int yaw_flag_start  = 0;

float current_time  = 0.0;
float H_roll        = 0.0;
float H_pitch       = 0.0;
float H_yaw_rate    = 0.0;
float H_throttle    = 0.0;
float H_yaw         = 0.0;


float H_roll_dot    = 0.0;
float H_pitch_dot   = 0.0;

float H_pitch_prev  = 0.0;
float H_roll_prev   = 0.0;

float imu_roll      = 0.0;
float imu_pitch     = 0.0;
float imu_yaw       = 0.0;

float imu_roll_dot  = 0.0;
float imu_pitch_dot = 0.0;
float imu_yaw_dot   = 0.0;
float battvolt      = 0.0;

float quad_x = 0.0;
float quad_y = 0.0;
float quad_z = 0.0;

float quad_x_ini = 0.0;
float quad_y_ini = 0.0;
float quad_z_ini = 0.0;

float quad_x_dot = 0.0;
float quad_y_dot = 0.0;
float quad_z_dot = 0.0;

float latitude  = 0.0;
float longitude = 0.0;

int pwm__thrust_measurement = 1000;
int flag_thrust_measurement = 0;

float fil_z     = 0.0;
float fil_z_1   = 0.0;
float fil_z_2   = 0.0;
float fil_z_3   = 0.0;
float fil_z_4   = 0.0;
float fil_z_5   = 0.0;
float fil_z_6   = 0.0;
float fil_z_7   = 0.0;
float fil_z_8   = 0.0;
float fil_z_9   = 0.0;
float fil_z_10  = 0.0;
float fil_z_11  = 0.0;

float fil_ph    = 0.0;
float fil_ph_1  = 0.0;
float fil_ph_2  = 0.0;
float fil_ph_3  = 0.0;
float fil_ph_4  = 0.0;
float fil_ph_5  = 0.0;

float fil_th    = 0.0;
float fil_th_1  = 0.0;
float fil_th_2  = 0.0;
float fil_th_3  = 0.0;
float fil_th_4  = 0.0;
float fil_th_5  = 0.0;

float e_phi_prev    = 0.0;
float e_theta_prev  = 0.0;
float e_psi_prev    = 0.0;

float tstart = 0.0;
float time_thrust_mea = 0.0;

float x_des  = 0.0;
float y_des  = 0.0;
float z_des  = 0.0;

float x_des_dot  = 0.0;
float y_des_dot  = 0.0;
float z_des_dot  = 0.0;

float yaw_initially = 0.0;

uint16_t PWM1 = 1000;
uint16_t PWM2 = 1000;
uint16_t PWM3 = 1000;
uint16_t PWM4 = 1000;

uint16_t PWM1_last_sysID = 1000;
uint16_t PWM2_last_sysID = 1000;
uint16_t PWM3_last_sysID = 1000;
uint16_t PWM4_last_sysID = 1000;

uint16_t Pf  = 0;
uint16_t Pm1 = 0;
uint16_t Pm2 = 0;
uint16_t Pm3 = 0;
uint16_t Pm4 = 0;

float t_ph_sys_ID = 0.0;
float t_th_sys_ID = 0.0;
float t_ps_sys_ID = 0.0;

float t_start_ph_sys_ID = 0.0;
float t_start_th_sys_ID = 0.0;
float t_start_ps_sys_ID = 0.0;

int flag_ph_sys_ID = 0.0;
int flag_th_sys_ID = 0.0;
int flag_ps_sys_ID = 0.0;

int arm_disarm_flag = 0;

float F         = 0.0;
float Mb1       = 0.0;
float Mb2       = 0.0;
float Mb3       = 0.0;

float landing_timer = 0.0;
int landing_timer_flag = 0;
float landing_timer_start = 0.0;

float des_theta_x   = 0.0;
float des_phi_y     = 0.0;

// global variables for pos controller
int y_des_flag = 0;
float timer_y_des = 0.0;
int timer_y_des_flag = 0;
float timer_y_des_start = 0.0;

int x_des_flag          = 0;
float timer_x_des       = 0.0;
int timer_x_des_flag    = 0;
float timer_x_des_start = 0.0;

void ModeStabilize::run()
{
        // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);

    if (code_starting_flag == 0){

        // To intialize the code
        copter.init_rc_out();
        hal.rcout->set_freq( 15, ESC_HZ); //0xFF  0x0F->b'00001111'
        hal.rcout->enable_ch(0);
        hal.rcout->enable_ch(1);
        hal.rcout->enable_ch(2);
        hal.rcout->enable_ch(3);

        // hal.serial(2)->begin(115200);

        code_starting_flag = 1;

    }else{

    ///////////// Arming checks  /////////////
        if (copter.motors->armed()){
            arm_disarm_flag = 1;
        }else{
            arm_disarm_flag = 0;
        }
    
    ///////////// Checking batter voltage  /////////////
        battery_check();
    ///////////// Taking pilot inputs  /////////////
        pilot_input();
    ///////////// getting states of quadcopter /////////////
        quad_states();

    ///////////// For system iedntification  /////////////
        // system_identification_main();

    ///////////// For attitude controller controller  /////////////
        attitude_altitude_controller();

    }

        // hal.console->printf("Hello captain from general");
    // hal.console->printf("landing_timer -> %f \n",landing_timer);
    hal.console->printf("z -> %f, zd -> %f, F -> %f \n",quad_z,z_des,F);


///////////// OLD CODE  /////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////



// // convert pilot input to lean angles
//     float target_roll, target_pitch;
//     get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max); // from mode.cpp file angle in centi-degree

//     // get pilot's desired yaw rate
//     // float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz()); // from mode.cpp file angle in centi-degree per sec

//     if (!motors->armed()) {
//         // Motors should be Stopped
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries
//     } else if (copter.ap.throttle_zero) {
//         // Attempting to Land
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
//     } else {
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//     }

//     switch (motors->get_spool_state()) {
//     case AP_Motors::SpoolState::SHUT_DOWN:
//         // Motors Stopped
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms();
//         break;

//     case AP_Motors::SpoolState::GROUND_IDLE:
//         // Landed
//         attitude_control->reset_yaw_target_and_rate();
//         attitude_control->reset_rate_controller_I_terms_smoothly();
//         break;

//     case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
//         // clear landing flag above zero throttle
//         if (!motors->limit.throttle_lower) {
//             set_land_complete(false);
//             // copter.motors->rc_write(1, 2000);
//         }
//         break;

//     case AP_Motors::SpoolState::SPOOLING_UP:
//     case AP_Motors::SpoolState::SPOOLING_DOWN:
//         // do nothing
//         break;
//     }

}

void ModeStabilize::battery_check(){
    battvolt=copter.battery_volt();
}

void ModeStabilize::attitude_altitude_controller(){
    if (RC_Channels::get_radio_in(CH_6) < 1200){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

        //// Initializing the states of the quadcopters
        quad_z_ini =  inertial_nav.get_position().z / 100.0;
        // yaw_initially = (360.0-(ahrs.yaw_sensor)/ 100.0)*3.141/180.0;     // rad;
        yaw_initially = 0.0;
        H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

        float quad_x_ini_inertial =  inertial_nav.get_position().x / 100.0;
        float quad_y_ini_inertial =  inertial_nav.get_position().y / 100.0;

        quad_x_ini =  cosf(yaw_initially)*quad_x_ini_inertial + sinf(yaw_initially)*quad_y_ini_inertial;
        quad_y_ini = -sinf(yaw_initially)*quad_x_ini_inertial + cosf(yaw_initially)*quad_y_ini_inertial;

        }
        else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
            if (copter.motors->armed()){
                custom_PID_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, z_des ,0.0);
                custom_pwm_code();
            }
        }else if (RC_Channels::get_radio_in(CH_6) > 1600){
            if(copter.motors->armed()){
                // custom_PID_position_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, z_des ,0.0);
                // custom_pwm_code();
            }
        }
}


void ModeStabilize::custom_PID_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

    // float mass = 1.236;
    float arm_length = 0.161;
    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24;
    }else{
         FM_devided_FF = 0.31;
    }

    float Kp_phi      = 0.012;   // 0.012 (best) 
    float Kp_theta    = 0.015;   // 0.015 (best) 
    float Kp_psi      = 0.1;     // 0.1 (best)

    float Kd_phi      = 0.2;     // 0.4 (best) // 0.25 (lab 1) // 2.0 (out 1) 1.6 (testbed)
    float Kd_theta    = 0.15;    // 0.4 (best) // 0.5 (lab 1) // // 3.0 (out 1) 2.5 (testbed)
    float Kd_psi      = 0.2;     // // 6.0 (testbed)

    float Ki_phi      = 0.00;    // 0.0005 (lab 1)
    float Ki_theta    = 0.00;    // 0.0005 (lab 1)
    float Ki_psi      = 0.00;    // 0.0005 (lab 1)

    float des_phi_offset = 10.0;
    float des_theta_offset = 0.0;

    des_phi = des_phi + des_phi_offset;
    des_theta = des_theta + des_theta_offset;

    float e_phi       = des_phi   - imu_roll;
    float e_theta     = des_theta - imu_pitch;

    float e_phi_sum   = e_phi + e_phi_prev;
    Mb1 = Kp_phi    * saturation_for_roll_pitch_angle_error(e_phi)    + Kd_phi    * (des_phi_dot - imu_roll_dot) + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
    e_phi_prev = e_phi;

    float e_theta_sum   = e_theta + e_theta_prev;
    Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta)  + Kd_theta  * (des_theta_dot - imu_pitch_dot) + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
    e_theta_prev = e_theta;

    ////// Yaw controller customized //////
    float e_psi     = des_psi   - imu_yaw;

    if (e_psi > 0.0){

        if ( e_psi > 180.0 ){
            e_psi = -(360.0 - e_psi);
        }

    }else if ( e_psi < 0.0 ) {
        if ( -e_psi < 180.0 ){
            e_psi = e_psi;
        }else{
            e_psi = 360.0 + e_psi;
        }
    }

    // hal.console->printf("Yaw__ %f H_yaw ->  %f e_psi %f \n",imu_yaw,H_yaw, e_psi);
    float e_psi_sum     = e_psi + e_psi_prev;
    Mb3                 = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
    e_psi_prev          = e_psi;
    // Mb3 = 0.0;
    ////// Yaw controller customized //////

///////////////////// Altitude controller /////////////////////

    float e_z   = z_des - quad_z;

    float Kp_z        = 2.0;    // 2.0 (best)
    float Kd_z        = 1.0;    // 1.0 (best)
    // F     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    // F     =  10.0 + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    F     =  10.0 + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    // F = 5.0;
    if (landing_timer > 2.0){
        F = 5.0;
    }

    // F = 10.0;
    if (F > 20.0){
        F = 20.0;
    }

    if (F < 0.0){
        F =  0.0;
    }

    float function_F1 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

}


void ModeStabilize::custom_PID_position_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

    float mass = 1.236;
    float arm_length = 0.161;
    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24;
    }else{
         FM_devided_FF = 0.31;
    }

    float Kp_phi      = 0.012;   // 0.012 (best) 
    float Kp_theta    = 0.015;   // 0.015 (best) 
    float Kp_psi      = 0.1;     // 0.1 (best)

    float Kd_phi      = 0.2;     // 0.4 (best) // 0.25 (lab 1) // 2.0 (out 1) 1.6 (testbed)
    float Kd_theta    = 0.15;    // 0.4 (best) // 0.5 (lab 1) // // 3.0 (out 1) 2.5 (testbed)
    float Kd_psi      = 0.2;     // // 6.0 (testbed)

    float Ki_phi      = 0.00;    // 0.0005 (lab 1)
    float Ki_theta    = 0.00;    // 0.0005 (lab 1)
    float Ki_psi      = 0.00;    // 0.0005 (lab 1)

    // Translational Position controller 

    float Kp_x = 0.0;
    float Kp_y = 0.0;

    float Kd_x = 0.0;
    float Kd_y = 0.0;

    float e_x  = x_des - quad_x ;
    float e_y  = y_des - quad_y ;

    float e_x_dot = x_des_dot - quad_x_dot;
    float e_y_dot = y_des_dot - quad_y_dot;

    des_theta_x   = Kp_x * (e_x) + Kd_x * (e_x_dot);
    des_phi_y     = Kp_y * (e_y) + Kd_y * (e_y_dot);

    float des_phi_offset    = 10.0;
    float des_theta_offset  = 00.0;

    des_phi     = des_phi   + des_phi_y     + des_phi_offset;
    des_theta   = des_theta + des_theta_x   + des_theta_offset;

    float e_phi       = des_phi   - imu_roll;
    float e_theta     = des_theta - imu_pitch;

    float e_phi_sum   = e_phi + e_phi_prev;
    Mb1 = Kp_phi    * saturation_for_roll_pitch_angle_error(e_phi)    + Kd_phi    * (des_phi_dot - imu_roll_dot) + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
    e_phi_prev = e_phi;

    float e_theta_sum   = e_theta + e_theta_prev;
    Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta)  + Kd_theta  * (des_theta_dot - imu_pitch_dot) + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
    e_theta_prev = e_theta;

    ////// Yaw controller customized //////
    float e_psi     = des_psi   - imu_yaw;

    if (e_psi > 0.0){

        if ( e_psi > 180.0 ){
            e_psi = -(360.0 - e_psi);
        }

    }else if ( e_psi < 0.0 ) {
        if ( -e_psi < 180.0 ){
            e_psi = e_psi;
        }else{
            e_psi = 360.0 + e_psi;
        }
    }

    // hal.console->printf("Yaw__ %f H_yaw ->  %f e_psi %f \n",imu_yaw,H_yaw, e_psi);
    float e_psi_sum     = e_psi + e_psi_prev;
    Mb3                 = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
    e_psi_prev          = e_psi;
    // Mb3 = 0.0;
    ////// Yaw controller customized //////

///////////////////// Altitude controller /////////////////////

    float e_z   = z_des - quad_z;

    float Kp_z        = 2.0;    // 2.0 (best)
    float Kd_z        = 1.0;    // 1.0 (best)
    F     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    F     =  10.0 + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);

    if (landing_timer > 2.0){
        F = 5.0;
    }

    // F = 10.0;
    if (F > 20.0){
        F = 20.0;
    }

    if (F < 0.0){
        F =  0.0;
    }

    float function_F1 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

}

void ModeStabilize::system_identification_x_axis(){
    Pf  = (double)channel_throttle->get_control_in() + 1100;

    if (t_ph_sys_ID > 5.0 && t_ph_sys_ID < 6.0){
        Pm1 = 50;
        PWM1 = Pf + Pm1;
        PWM2 = Pf - Pm1;
        PWM3 = Pf + Pm1;
        PWM4 = Pf - Pm1;
    }else{
        PWM1 = Pf;
        PWM2 = Pf;
        PWM3 = Pf;
        PWM4 = Pf;
    }
}

void ModeStabilize::system_identification_y_axis(){
    Pf  = (double)channel_throttle->get_control_in() + 1100;

    if (t_ph_sys_ID > 5.0 && t_ph_sys_ID < 6.0){
        Pm2 = 50;
        PWM1 = Pf - Pm2;
        PWM2 = Pf - Pm2;
        PWM3 = Pf + Pm2;
        PWM4 = Pf + Pm2;
    }else{
        PWM1 = Pf;
        PWM2 = Pf;
        PWM3 = Pf;
        PWM4 = Pf;
    }
}


void ModeStabilize::custom_pwm_code(){

    // if (RC_Channels::get_radio_in(CH_6) > 1600){

        copter.init_rc_out();
        SRV_Channels::cork();
        hal.rcout->write(0,PWM1);
        hal.rcout->write(1,PWM2);
        hal.rcout->write(2,PWM3);
        hal.rcout->write(3,PWM4);
        SRV_Channels::push();

        // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);
}

void ModeStabilize::system_identification_main(){
    if (RC_Channels::get_radio_in(CH_6) < 1200){
            // copter.motors->armed(false);
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries
            Pf = 1100;
            PWM1 = Pf;
            PWM2 = Pf;
            PWM3 = Pf;
            PWM4 = Pf;
            t_ph_sys_ID = 0.0;
            flag_ph_sys_ID = 0;
        }
        else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
            if (copter.motors->armed()){
                Pf  = (double)channel_throttle->get_control_in() + 1100;
                PWM1 = Pf;
                PWM2 = Pf;
                PWM3 = Pf;
                PWM4 = Pf;
                t_ph_sys_ID = 0.0;
                flag_ph_sys_ID = 0;
                custom_pwm_code();
            }
        }else if (RC_Channels::get_radio_in(CH_6) > 1600){
            if(copter.motors->armed()){
                if ( flag_ph_sys_ID == 0){
                t_start_ph_sys_ID = AP_HAL::millis();
                flag_ph_sys_ID = 1;
            }
                t_ph_sys_ID = (AP_HAL::millis() - t_start_ph_sys_ID)/1000.0;
                // system_identification_x_axis();
                system_identification_y_axis();
                custom_pwm_code();
            }
        }
}

void ModeStabilize::quad_states(){
    // Position in inertial reference frame
    float quad_x_inertial =  inertial_nav.get_position().x / 100.0;
    float quad_y_inertial =  inertial_nav.get_position().y / 100.0;

    // position in body reference frame
    quad_x =  (cosf(yaw_initially)*quad_x_inertial + sinf(yaw_initially)*quad_y_inertial) - quad_x_ini;
    quad_y = (-sinf(yaw_initially)*quad_x_inertial + cosf(yaw_initially)*quad_y_inertial) - quad_y_ini;

    quad_y = -quad_y;

    quad_z =  (inertial_nav.get_position().z / 100.0) - quad_z_ini;

    if (quad_z < 0){quad_z = 0;}
    if (quad_z > 5){quad_z = 5;}

    // linear velocity in inertial frame of reference
    float quad_x_dot_inertial =  inertial_nav.get_velocity().x /100.0;
    float quad_y_dot_inertial =  inertial_nav.get_velocity().y /100.0;
    quad_z_dot                =  inertial_nav.get_velocity().z /100.0;

    // linear velocity in body reference frame
    quad_x_dot =  (cosf(yaw_initially)*quad_x_dot_inertial + sinf(yaw_initially)*quad_y_dot_inertial);
    quad_y_dot = (-sinf(yaw_initially)*quad_x_dot_inertial + cosf(yaw_initially)*quad_y_dot_inertial);

    imu_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    imu_roll_dot    =  (ahrs.get_gyro().x);             // degrees/second
    imu_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
    imu_yaw_dot     = -(ahrs.get_gyro().z);             // degrees/second

}

void ModeStabilize::custom_PID_controller_sysID(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){
    
    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24;
    }else{
         FM_devided_FF = 0.31;
    }

    float arm_length = 0.161;

    float Kp_phi      = 0.012;   // 0.012 (best) 
    float Kp_theta    = 0.015;   // 0.015 (best) 
    float Kp_psi      = 0.1;     // 0.1 (best)

    float Kd_phi      = 0.2;     // 0.4 (best) // 0.25 (lab 1) // 2.0 (out 1) 1.6 (testbed)
    float Kd_theta    = 0.15;    // 0.4 (best) // 0.5 (lab 1) // // 3.0 (out 1) 2.5 (testbed)
    float Kd_psi      = 0.2;     // // 6.0 (testbed)

    float e_phi       = des_phi   - imu_roll;
    float e_theta     = des_theta - imu_pitch;

    Mb1 = Kp_phi    * saturation_for_roll_pitch_angle_error(e_phi)    + Kd_phi    * (des_phi_dot - imu_roll_dot);

    Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta)  + Kd_theta  * (des_theta_dot - imu_pitch_dot);

    ////// Yaw controller customized //////
    float e_psi     = des_psi   - imu_yaw;

    if (e_psi > 0.0){

        if ( e_psi > 180.0 ){
            e_psi = -(360.0 - e_psi);
        }

    }else if ( e_psi < 0.0 ) {
        if ( -e_psi < 180.0 ){
            e_psi = e_psi;
        }else{
            e_psi = 360.0 + e_psi;
        }
    }

    Mb3           = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) ;    
    Mb3 = 0.0;
    ////// Yaw controller customized //////

    float function_F1 =  Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = -Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 =  Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = -Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1) + Pf;
    PWM2 = Inverse_thrust_function(function_F2) + Pf;
    PWM3 = Inverse_thrust_function(function_F3) + Pf;
    PWM4 = Inverse_thrust_function(function_F4) + Pf;

    // PWM1 = Inverse_thrust_function(function_F1);
    // PWM2 = Inverse_thrust_function(function_F2);
    // PWM3 = Inverse_thrust_function(function_F3);
    // PWM4 = Inverse_thrust_function(function_F4);

}

void ModeStabilize::pilot_input(){

    H_roll      = (double)(channel_roll->get_control_in())/100.0;
    H_roll_dot  = (H_roll - H_roll_prev)/400.0;
    H_roll_prev = H_roll;

    H_pitch     = (double)(channel_pitch->get_control_in())/100.0;
    H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
    H_pitch_prev= H_pitch;

    H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;
    H_throttle  =  (double)channel_throttle->get_control_in()-500.0;

    float dt_yaw = 1.0/100.0;
    H_yaw = wrap_360(H_yaw + H_yaw_rate*dt_yaw);

    if (H_throttle > -20 && H_throttle < 20){
        H_throttle = 0.0;
    }

    float dt_z = 1.0/15000.0;
    z_des       =  z_des + H_throttle * dt_z;
    if (z_des > 5.0){
        z_des = 5.0;
    }
    if (z_des < 0.0){
        z_des = 0.0;
    }

    //// To inactivate XY controller
    // For y direction
    if (H_roll < -5.0 || H_roll > 5.0){
        des_phi_y = 0.0;
        y_des_flag = 0;
        timer_y_des = 0;
        timer_y_des_flag = 0;}
    else{
        if (timer_y_des_flag == 0){
            timer_y_des_start = AP_HAL::millis()/1000.0;
            timer_y_des_flag = 1;}
        else{
            timer_y_des = AP_HAL::millis()/1000.0 - timer_y_des_start;
        }

        if (timer_y_des > 1.0){
            if (y_des_flag == 0 ){
                    y_des = quad_y;
                    y_des_flag = 1;}
            }
        else{
            des_phi_y = 0.0;
        }
    }

    // For x direction
    if (H_pitch < -5.0 || H_pitch > 5.0){
        des_theta_x = 0.0;
        x_des_flag = 0;
        timer_x_des = 0;
        timer_x_des_flag = 0;}
    else{
        if (timer_x_des_flag == 0){
            timer_x_des_start = AP_HAL::millis()/1000.0;
            timer_x_des_flag = 1;}
        else{
            timer_x_des = AP_HAL::millis()/1000.0 - timer_x_des_start;
        }

        if (timer_x_des > 1.0){
            if (x_des_flag == 0 ){
                    x_des = quad_x;
                    x_des_flag = 1;}
            }
        else{
            des_theta_x = 0.0;
        }
    }
////////

    ////  for safe landing
    if (H_throttle < -450.0){
        if (landing_timer_flag == 0){
            landing_timer_flag = 1;
            landing_timer_start = AP_HAL::millis()/1000.0;
        }
        landing_timer = AP_HAL::millis()/1000.0 - landing_timer_start;
    }
    else{
        landing_timer = 0.0;  
        landing_timer_flag = 0;
        landing_timer_start = 0.0;
    }
    ////////

}


float ModeStabilize::saturation_for_roll_pitch_angle_error(float error){

    float lim = 30.0;

    if (error > lim){
        error = lim;
    }else if (error < -lim){
        error = -lim;
    }else {
        error = error;
    }
    return error;
}


float ModeStabilize::sat_I_gain_ph_th(float sum){

    float lim = 10.0;

    if (sum > lim){
        sum = lim;
    }else if (sum < -lim){
        sum = -lim;
    }else {
        sum = sum;
    }
    return sum;
}

float ModeStabilize::sat_I_gain_psi(float sum){

    float lim = 20.0;

    if (sum > lim){
        sum = lim;
    }else if (sum < -lim){
        sum = -lim;
    }else {
        sum = sum;
    }
    return sum;
}

int ModeStabilize::Inverse_thrust_function(float Force){
    int PWM = 1200;

/////////////////////////// From the quadcopter motors  ///////////////////////////

    if (battvolt >= 11.5 ){PWM = 1000 * (0.9206 + (sqrtf(12.8953 + 30.3264*Force)/(15.1632)));
    }else{PWM = 1000 * (0.6021 + (sqrtf(33.2341 + 19.418*Force)/(9.5740)));}
    if (PWM > 2000){PWM = 2000;}
    if (PWM < 1000){PWM = 1000;}

    return PWM;
}


float ModeStabilize::saturation_for_yaw_angle_error(float error){

    float lim = 30.0;

    if (error > lim){
        error = lim;
    }else if (error < -lim){
        error = -lim;
    }else {
        error = error;
    }
    return error;
}

