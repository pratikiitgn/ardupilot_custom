#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  
#include "mycontroller_usercode.h"

#define ESC_HZ 490

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

float x_des  = 0.0;
float y_des  = 0.0;
float z_des  = 0.0;

float x_des_dot  = 0.0;
float y_des_dot  = 0.0;
float z_des_dot  = 0.0;

float yaw_initially = 0.0;

int arm_disarm_flag = 0;

float landing_timer = 0.0;
int landing_timer_flag = 0;
float landing_timer_start = 0.0;

// global variables for pos controller
int y_des_flag = 0;
float timer_y_des = 0.0;
int timer_y_des_flag = 0;
float timer_y_des_start = 0.0;

int x_des_flag          = 0;
float timer_x_des       = 0.0;
int timer_x_des_flag    = 0;
float timer_x_des_start = 0.0;

// to get the cable states
Vector3f qc(0.0,0.0,0.0);
Vector3f qc_dot(0.0,0.0,0.0);
Vector3f qc_old(0.0,0.0,0.0);

//  
float Final_roll_angle_TRO_single_quad  = 0.0;  // rarnge [-3500 3500]
float Final_pitch_angle_TRO_single_quad = 0.0;  // rarnge [-3500 3500]
float Final_yaw_rate_TRO_single_quad    = 0.0;  // rarnge [-20250 20250]
float Final_throttle_TRO_single_quad    = 0.0;  // rarnge [ 0 1] - offset = 0.219

void ModeStabilize::run()
{

        //////////////////////////////////////////////////////////
        /// Before turning into the TRO code lets initialize and reset the states of the system
        //////////////////////////////////////////////////////////
        if (code_starting_flag == 1){
            yaw_initially = 0.0;
            H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

            float quad_x_ini_inertial =  inertial_nav.get_position().x / 100.0;
            float quad_y_ini_inertial =  inertial_nav.get_position().y / 100.0;

            quad_x_ini =  cosf(yaw_initially)*quad_x_ini_inertial + sinf(yaw_initially)*quad_y_ini_inertial;
            quad_y_ini = -sinf(yaw_initially)*quad_x_ini_inertial + cosf(yaw_initially)*quad_y_ini_inertial;
            quad_z_ini =  inertial_nav.get_position().z / 100.0;
            
            ///////////// Checking batter voltage  /////////////
                battery_check();
            ///////////// getting states of quadcopter /////////////
                quad_states();
                x_des      =  quad_x;
                y_des      =  quad_y;
                z_des      =  quad_z;
            ///////////// getting states of quadcopter /////////////
                cable_states();
            ///////////// Taking pilot inputs  /////////////
                pilot_input();
                code_starting_flag = 0;
        }
        
    /////////////////////////////////////
    /// Let's initialize the necessary functions/variables
    ///////////////////////////////////

    ///////////// Checking batter voltage  /////////////
        battery_check();
    ///////////// getting states of quadcopter /////////////
        quad_states();
    ///////////// getting states of quadcopter /////////////
        cable_states();
    ///////////// Taking pilot inputs  /////////////
        pilot_input();

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Main logic starts here //////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    if (RC_Channels::get_radio_in(CH_7) < 1200){
        /////////////////////////////////////
        /// This is default stabilize controller
        /////////////////////////////////////

        update_simple_mode();

        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        // get pilot's desired yaw rate
        float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        if (!motors->armed()) {
            // Motors should be Stopped
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        } else if (copter.ap.throttle_zero
                || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
            // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
            // in order to facilitate the spoolup block

            // Attempting to Land
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        float pilot_desired_throttle = get_pilot_desired_throttle();

        switch (motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
            // Motors Stopped
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->reset_rate_controller_I_terms();
            pilot_desired_throttle = 0.0f;
            break;

        case AP_Motors::SpoolState::GROUND_IDLE:
            // Landed
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->reset_rate_controller_I_terms_smoothly();
            pilot_desired_throttle = 0.0f;
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            // clear landing flag above zero throttle
            if (!motors->limit.throttle_lower) {
                set_land_complete(false);
            }
            break;

        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // do nothing
            break;
        }

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // output pilot's throttle
        attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);

        // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f\n", target_roll, target_pitch, target_yaw_rate,pilot_desired_throttle);

        //////////////////////////////////////////////////////////
        ////// The default Mode_stabilize code ends here    //////
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        /// Before turning into the TRO code lets initialize and reset the states of the system
        //////////////////////////////////////////////////////////

        yaw_initially = 0.0;
        H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

        float quad_x_ini_inertial =  inertial_nav.get_position().x / 100.0;
        float quad_y_ini_inertial =  inertial_nav.get_position().y / 100.0;

        quad_x_ini =  cosf(yaw_initially)*quad_x_ini_inertial + sinf(yaw_initially)*quad_y_ini_inertial;
        quad_y_ini = -sinf(yaw_initially)*quad_x_ini_inertial + cosf(yaw_initially)*quad_y_ini_inertial;
        quad_z_ini =  inertial_nav.get_position().z / 100.0;
        x_des      =  quad_x;
        y_des      =  quad_y;
        z_des      =  quad_z;
        // hal.console->printf("%3.3f,%3.3f\n", quad_z,z_des);
        // hal.console->printf("%3.3f,%3.3f\n", quad_x,x_des);
    }
    else if (RC_Channels::get_radio_in(CH_7) > 1400 ){
        // hal.console->printf("%3.3f,%3.3f\n", quad_z,z_des);

        ////////////////////////////////////
        // call default motor starter code from stabilize code
        ////////////////////////////////////

        update_simple_mode();

        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        // get pilot's desired yaw rate
        // float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        if (!motors->armed()) {
            // Motors should be Stopped
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        } else if (copter.ap.throttle_zero
                || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
            // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
            // in order to facilitate the spoolup block

            // Attempting to Land
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        /////////////////////////////////////
        /// TRO 23 controller for single quad starts here
        ////////////////////////////////////

        // if (copter.motors->armed()){
            Non_linear_controller_single_quad();
        // }
        
        /////////////////////////////////////
        /// TRO 23 controller for single quad ends here
        ////////////////////////////////////
    }
}

void ModeStabilize::battery_check(){
    battvolt=copter.battery_volt();
}

void ModeStabilize::Non_linear_controller_single_quad(){
    // if (copter.motors->armed()){

        // X direction desired position
        if (H_pitch > -2.0 && H_pitch < 2.0){H_pitch = 0.0;}
        float dt_x = 1.0/5000.0;
        x_des       =  x_des + (-H_pitch) * dt_x;
        if (x_des > 5.0){x_des = 5.0;}
        if (x_des < -5.0){x_des = -5.0;}
        x_des_dot = 0.0;

        // Y direction desired position
        if (H_roll > -2.0 && H_roll < 2.0){H_roll = 0.0;}
        float dt_y = 1.0/5000.0;
        y_des       =  y_des + (-H_roll) * dt_y;
        if (y_des > 5.0){y_des = 5.0;}
        if (y_des < -5.0){y_des = -5.0;}
        y_des_dot = 0.0;

        // Position controller
        if (H_throttle > -20.0 && H_throttle < 20.0){H_throttle = 0.0;}
        float dt_z = 1.0/20000.0;
        z_des       =  z_des + H_throttle * dt_z;
        if (z_des > 5.0){z_des = 5.0;}
        if (z_des < -5.0){z_des = -5.0;}
        z_des_dot = 0.0;

        float e_z       = z_des - quad_z;
        float e_z_dot   = z_des_dot - quad_z_dot;
        float Kp_z      = 0.5;    // 0.5 (best)
        float Kd_z      = 0.2;    // 0.2 (best)

        // final_thrust - should be friom -1 to 1
        float final_thrust = Kp_z * e_z + Kd_z * e_z_dot;

        // final_thrust = 0.0;
        // hal.console->printf("Current -> %3.3f,%3.3f,%3.3f || desired -> %3.3f,%3.3f,%3.3f \n", quad_x,quad_y,quad_z,x_des,y_des,z_des);

        /// X position controller
        float e_x       = x_des - quad_x;
        float e_x_dot   = x_des_dot - quad_x_dot;
        float Kp_x      = 10.0;    // 2.0 (best)
        float Kd_x      = 5.0;     // 1.0 (best)
        float Trust_1         = Kp_x * e_x + Kd_x * e_x_dot;    // -45 to 45

        H_roll      = H_roll;        // -45 to 45
        H_pitch     = H_pitch;       // -45 to 45
        H_yaw_rate  = H_yaw_rate;    // -45 to 45
        H_throttle  = H_throttle;    // -500 to 500

        // Simple_Linear_mapping_code()

        Final_roll_angle_TRO_single_quad    = H_roll* 100.0;
        Final_pitch_angle_TRO_single_quad   = Trust_1* 100.0;
        Final_yaw_rate_TRO_single_quad      = -H_yaw_rate* 500.0;
        float throttle_offset               = 0.395;
        Final_throttle_TRO_single_quad      = throttle_offset + final_thrust;

        // To make sure the inputs are within limits
        // [1] Roll angle -> [-3500 3500]
        // [2] Roll pitch -> [-3500 3500]
        // [3] Roll angle -> [-20250 20250]
        // [4] Throttle   -> [ 0 1] - offset = 0.219

        Satuation_func_Final_roll_angle(Final_roll_angle_TRO_single_quad);
        Satuation_func_Final_pitch_angle(Final_pitch_angle_TRO_single_quad);
        Satuation_func_Final_yaw_rate(Final_yaw_rate_TRO_single_quad);
        Satuation_func_Final_throttle(Final_throttle_TRO_single_quad);

        // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f\n", Final_roll_angle_TRO_single_quad, Final_pitch_angle_TRO_single_quad, Final_yaw_rate_TRO_single_quad,Final_throttle_TRO_single_quad);

        // call default attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(Final_roll_angle_TRO_single_quad,Final_pitch_angle_TRO_single_quad,Final_yaw_rate_TRO_single_quad);
        // call default altitude controller
        attitude_control->set_throttle_out(Final_throttle_TRO_single_quad, true, g.throttle_filt);
    // }
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

    // if (quad_z < 0){quad_z = 0;}
    if (quad_z > 5.0){quad_z = 5.0;}

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

    // hal.console->printf("%3.3f,%3.3f\n", imu_yaw,imu_yaw_dot);

}

void ModeStabilize::cable_states(){
    
    // float rate_of_Mode_stabilize = 400.0;
    float rate_of_Mode_stabilize = 1.0;
    qc_dot = (qc - qc_old)/rate_of_Mode_stabilize;
    qc_old = qc;

    // hal.console->printf("%3.3f,%3.3f,%3.3f\n", qc[0],qc[1],qc[2]);
    // hal.console->printf("%3.3f,%3.3f\n", qc[0],qc_dot[0]);
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



    // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f\n", H_roll,H_pitch, H_yaw,H_throttle);

    //// To inactivate XY controller
    // For y direction
    // if (H_roll < -5.0 || H_roll > 5.0){
    //     des_phi_y = 0.0;
    //     y_des_flag = 0;
    //     timer_y_des = 0;
    //     timer_y_des_flag = 0;}
    // else{
    //     if (timer_y_des_flag == 0){
    //         timer_y_des_start = AP_HAL::millis()/1000.0;
    //         timer_y_des_flag = 1;}
    //     else{
    //         timer_y_des = AP_HAL::millis()/1000.0 - timer_y_des_start;
    //     }

    //     if (timer_y_des > 1.0){
    //         if (y_des_flag == 0 ){
    //                 y_des = quad_y;
    //                 y_des_flag = 1;}
    //         }
    //     else{
    //         des_phi_y = 0.0;
    //     }
    // }

    // // For x direction
    // if (H_pitch < -5.0 || H_pitch > 5.0){
    //     des_theta_x = 0.0;
    //     x_des_flag = 0;
    //     timer_x_des = 0;
    //     timer_x_des_flag = 0;}
    // else{
    //     if (timer_x_des_flag == 0){
    //         timer_x_des_start = AP_HAL::millis()/1000.0;
    //         timer_x_des_flag = 1;}
    //     else{
    //         timer_x_des = AP_HAL::millis()/1000.0 - timer_x_des_start;
    //     }

    //     if (timer_x_des > 1.0){
    //         if (x_des_flag == 0 ){
    //                 x_des = quad_x;
    //                 x_des_flag = 1;}
    //         }
    //     else{
    //         des_theta_x = 0.0;
    //     }
    // }
////////

    ////  for safe landing
    // if (H_throttle < -450.0){
    //     if (landing_timer_flag == 0){
    //         landing_timer_flag = 1;
    //         landing_timer_start = AP_HAL::millis()/1000.0;
    //     }
    //     landing_timer = AP_HAL::millis()/1000.0 - landing_timer_start;
    // }
    // else{
    //     landing_timer = 0.0;  
    //     landing_timer_flag = 0;
    //     landing_timer_start = 0.0;
    // }
    ////////

}
void ModeStabilize::Satuation_func_Final_roll_angle(float angle){
    float max_angle_allow = 3500.0;
    if (angle > max_angle_allow)
    {
        Final_roll_angle_TRO_single_quad = max_angle_allow;
    }
    if (angle < -max_angle_allow)
    {
        Final_roll_angle_TRO_single_quad = -max_angle_allow;
    }
}

void ModeStabilize::Satuation_func_Final_pitch_angle(float angle){
    float max_angle_allow = 3500.0;
    if (angle > max_angle_allow)
    {
        Final_pitch_angle_TRO_single_quad = max_angle_allow;
    }
    if (angle < -max_angle_allow)
    {
        Final_pitch_angle_TRO_single_quad = -max_angle_allow;
    }
}

void ModeStabilize::Satuation_func_Final_yaw_rate(float angle_rate){
    float max_angle_rate_allow = 20000;
    if (angle_rate > max_angle_rate_allow)
    {
        Final_yaw_rate_TRO_single_quad = max_angle_rate_allow;
    }
    if (angle_rate < -max_angle_rate_allow)
    {
        Final_yaw_rate_TRO_single_quad = -max_angle_rate_allow;
    }
}

void ModeStabilize::Satuation_func_Final_throttle(float throttle_value){
    float max_throttle_value = 1;
    float min_throttle_value = 0;
    if (throttle_value > max_throttle_value)
    {
        Final_throttle_TRO_single_quad = max_throttle_value;
    }
    if (throttle_value < min_throttle_value)
    {
        Final_throttle_TRO_single_quad = min_throttle_value;
    }
}