#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h>
#include "mycontroller_usercode.h"
#include <AP_GPS/AP_GPS.h>




// // For yaw commands
// int yaw_flag_start = 0;

// float current_time  = 0.0;
// float H_roll        = 0.0;
// float H_pitch       = 0.0;
// float H_yaw_rate    = 0.0;
// // float H_throttle    = 0.0;
// float H_yaw         = 0.0;

// float H_roll_dot    = 0.0;
// float H_pitch_dot   = 0.0;

// float H_pitch_prev  = 0.0;
// float H_roll_prev   = 0.0;

// float imu_roll      = 0.0;
// float imu_pitch     = 0.0;
// float imu_yaw       = 0.0;

// float imu_roll_dot  = 0.0;
// float imu_pitch_dot = 0.0;
// float imu_yaw_dot   = 0.0;
// float battvolt      = 0.0;

// float quad_x = 0.0;
// float quad_y = 0.0;
// float quad_z = 0.0;

// float quad_x_ini = 0.0;
// float quad_y_ini = 0.0;
// float quad_z_ini = 0.0;

// float quad_x_dot = 0.0;
// float quad_y_dot = 0.0;
// float quad_z_dot = 0.0;

// float latitude  = 0.0;
// float longitude = 0.0;

// int pwm__thrust_measurement = 1000;
// int flag_thrust_measurement = 0;

// float fil_z     = 0.0;
// float fil_z_1   = 0.0;
// float fil_z_2   = 0.0;
// float fil_z_3   = 0.0;
// float fil_z_4   = 0.0;
// float fil_z_5   = 0.0;
// float fil_z_6   = 0.0;
// float fil_z_7   = 0.0;
// float fil_z_8   = 0.0;
// float fil_z_9   = 0.0;
// float fil_z_10  = 0.0;
// float fil_z_11  = 0.0;

// float fil_ph    = 0.0;
// float fil_ph_1  = 0.0;
// float fil_ph_2  = 0.0;
// float fil_ph_3  = 0.0;
// float fil_ph_4  = 0.0;
// float fil_ph_5  = 0.0;

// float fil_th    = 0.0;
// float fil_th_1  = 0.0;
// float fil_th_2  = 0.0;
// float fil_th_3  = 0.0;
// float fil_th_4  = 0.0;
// float fil_th_5  = 0.0;

// float e_phi_prev    = 0.0;
// float e_theta_prev  = 0.0;
// float e_psi_prev    = 0.0;

// float tstart = 0.0;
// float time_thrust_mea = 0.0;

// float x_des  = 0.0;
// float y_des  = 0.0;
// float z_des  = 0.0;

// float x_des_dot  = 0.0;
// float y_des_dot  = 0.0;
// float z_des_dot  = 0.0;

// float z_des_init = 0.0;
// float yaw_initially = 0.0;

bool ModeMyController::init(bool)
{
//         hal.rcout->set_freq( 15, 490); //0xFF  0x0F->b'00001111'
//         hal.rcout->enable_ch(0);
//         hal.rcout->enable_ch(1);
//         hal.rcout->enable_ch(2);
//         hal.rcout->enable_ch(3);

//         H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;
        
//         tstart  = AP_HAL::millis();
//         z_des_init     = inertial_nav.get_altitude()/100.0;
//         hal.serial(2)->begin(115200);


//         yaw_initially = (360.0-(ahrs.yaw_sensor)/ 100.0)*3.141/180.0;     // rad;

//         quad_x_ini =  inertial_nav.get_position().x / 100.0;
//         quad_y_ini =  inertial_nav.get_position().y / 100.0;
//         quad_z_ini =  inertial_nav.get_position().z / 100.0;

//         quad_x_ini =  cosf(yaw_initially)*quad_x_ini + sinf(yaw_initially)*quad_y_ini;
//         quad_y_ini = -sinf(yaw_initially)*quad_x_ini + cosf(yaw_initially)*quad_y_ini;

        return true;

}

// void ModeMyController::custom_PID_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

//     float mass = 1.236;
//     float arm_length = 0.161;
//     float FM_devided_FF ;
//     if (battvolt >= 11.5 ){
//          FM_devided_FF = 0.24;
//     }else{
//          FM_devided_FF = 0.31;
//     }

//     float Kp_phi      = 0.012;   // 0.012 (best) 
//     float Kp_theta    = 0.015;   // 0.015 (best) 
//     float Kp_psi      = 0.1;     // 0.1 (best)

//     float Kd_phi      = 0.2;     // 0.4 (best) // 0.25 (lab 1) // 2.0 (out 1) 1.6 (testbed)
//     float Kd_theta    = 0.15;    // 0.4 (best) // 0.5 (lab 1) // // 3.0 (out 1) 2.5 (testbed)
//     float Kd_psi      = 0.2;     // // 6.0 (testbed)

//     float Ki_phi      = 0.00;    // 0.0005 (lab 1)
//     float Ki_theta    = 0.00;    // 0.0005 (lab 1)
//     float Ki_psi      = 0.00;    // 0.0005 (lab 1)

//     float e_phi       = des_phi   - imu_roll;
//     float e_theta     = des_theta - imu_pitch;

//     float e_phi_sum   = e_phi + e_phi_prev;
//     float Mb1 = Kp_phi    * saturation_for_roll_pitch_angle_error(e_phi)    + Kd_phi    * (des_phi_dot - imu_roll_dot) + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
//     e_phi_prev = e_phi;

//     float e_theta_sum   = e_theta + e_theta_prev;
//     float Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta)  + Kd_theta  * (des_theta_dot - imu_pitch_dot) + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
//     e_theta_prev = e_theta;


//     ////// Yaw controller customized //////
//     float e_psi     = des_psi   - imu_yaw;

//     if (e_psi > 0.0){

//         if ( e_psi > 180.0 ){
//             e_psi = -(360.0 - e_psi);
//         }

//     }else if ( e_psi < 0.0 ) {
//         if ( -e_psi < 180.0 ){
//             e_psi = e_psi;
//         }else{
//             e_psi = 360.0 + e_psi;
//         }
//     }

//     // hal.console->printf("Yaw__ %f H_yaw ->  %f e_psi %f \n",imu_yaw,H_yaw, e_psi);
//     float e_psi_sum     = e_psi + e_psi_prev;
//     float Mb3           = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
//     e_psi_prev          = e_psi;
//     // Mb3 = 0.0;
//     ////// Yaw controller customized //////


// ///////////////////// Altitude controller /////////////////////


//     // fil_z_4 = fil_z_3;
//     // fil_z_3 = fil_z_2;
//     // fil_z_2 = fil_z_1;
//     // fil_z_1 = copter.barometer.get_altitude() * 100.0f;
//     // float baro_alt = 0.5*fil_z_1 + 0.2*fil_z_2 + 0.2*fil_z_3 + 0.1*fil_z_4;
    
//     fil_z_11    = fil_z_10;
//     fil_z_10    = fil_z_9;
//     fil_z_9     = fil_z_8;
//     fil_z_8     = fil_z_7;
//     fil_z_7     = fil_z_6;
//     fil_z_6     = fil_z_5;
//     fil_z_5     = fil_z_4;
//     fil_z_4     = fil_z_3;
//     fil_z_3     = fil_z_2;
//     fil_z_2     = fil_z_1;
//     fil_z_1     = quad_z;
//     fil_z       = (fil_z_1+ fil_z_2+ fil_z_3+ fil_z_4+ fil_z_5+ fil_z_6+ fil_z_7+ fil_z_8+ fil_z_9+ fil_z_10+ fil_z_11)/11.0;

//     float e_z   = z_des - quad_z;

//     float Kp_z        = 4.0;    // 2.0 (best)
//     float Kd_z        = 1.0;    // 1.0 (best)
//     float F     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    
//     if (F > 20.0){
//         F = 20.0;
//     }
    
//     if (F < 0.0){
//         F =  0.0;
//     }
    

//     // hal.console->printf("z: %f , z_dot %f ,z_des: %f ,F: %f , Th %f \n", fil_z,z_dot, z_des, F, H_throttle);

//     // F = H_throttle/50.0;
//     // hal.console->printf(" %f \n ", F);
//     // hal.console->printf(" Altitude %f \n ", baro_alt);

// ///////////////////// Prioritizing the control commands [end] /////////////////////
//     // Mb1=0.0;
//     // Mb2=0.0;
//     // Mb3=0.0;
    
//     float function_F1 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
//     float function_F2 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
//     float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
//     float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

//     // hal.console->printf("F1-> %f, F2-> %f, F3-> %f, F4-> %f  \n", function_F1, function_F2, function_F3, function_F4);

//     PWM1 = Inverse_thrust_function(function_F1);
//     PWM2 = Inverse_thrust_function(function_F2);
//     PWM3 = Inverse_thrust_function(function_F3);
//     PWM4 = Inverse_thrust_function(function_F4);

//     // hal.console->printf("F -> %f Roll ->  %f Pitch ->  %f Sum_phi %f Sum_theta %f \n",F, imu_roll, imu_pitch, sat_I_gain_psi(e_phi_sum), sat_I_gain_psi(e_theta_sum));
//     // hal.console->printf("Yaw__ ->  %f H_yaw ->  %f, Mb3 -> %f\n",imu_yaw,H_yaw, Mb3);
//     // hal.console->printf("PWM %d,%d,%d,%d \n", PWM1, PWM2, PWM3, PWM4);
//     // hal.console->printf("Each N %f,%f,%f,%f \n", function_F1, function_F2, function_F3, function_F4);

// }

// void ModeMyController::PID_pos_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

//     float mass = 0.75;
//     float arm_length = 0.161;
//     float FM_devided_FF ;
//     if (battvolt >= 11.5 ){
//          FM_devided_FF = 0.24;
//     }else{
//          FM_devided_FF = 0.31;
//     }

//     float Kp_phi      = 0.012;   // 0.012 (best) 
//     float Kp_theta    = 0.015;   // 0.015 (best) 
//     float Kp_psi      = 0.1;     // 0.1 (best)

//     float Kd_phi      = 0.2;     // 0.4 (best) // 0.25 (lab 1) // 2.0 (out 1) 1.6 (testbed)
//     float Kd_theta    = 0.15;    // 0.4 (best) // 0.5 (lab 1) // // 3.0 (out 1) 2.5 (testbed)
//     float Kd_psi      = 0.2;     // // 6.0 (testbed)

//     float Ki_phi      = 0.00;    // 0.0005 (lab 1)
//     float Ki_theta    = 0.00;    // 0.0005 (lab 1)
//     float Ki_psi      = 0.00;    // 0.0005 (lab 1)

//     float Kp_x        = 0.0;
//     float Kp_y        = 0.0;
//     float Kd_x        = 0.0;
//     float Kd_y        = 0.0;

//     // if ()

//     float des_phi_pos       = -(Kp_y * ( y_des - quad_y ) + Kd_y * ( y_des_dot - quad_y_dot ) );
//     float des_theta_pos     =  (Kp_x * ( x_des - quad_x ) + Kd_x * ( x_des_dot - quad_x_dot ) );

//     float e_phi       = (des_phi+des_phi_pos)   - imu_roll;
//     float e_theta     = (des_theta+des_theta_pos) - imu_pitch;

//     float e_phi_sum   = e_phi + e_phi_prev;
//     // float Mb1_pos     = quad_y_dot;
//     float Mb1 = Kp_phi    * saturation_for_roll_pitch_angle_error(e_phi)    + Kd_phi    * (des_phi_dot - imu_roll_dot) + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
//     e_phi_prev = e_phi;

//     float e_theta_sum   = e_theta + e_theta_prev;
//     float Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta)  + Kd_theta  * (des_theta_dot - imu_pitch_dot) + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
//     e_theta_prev = e_theta;

//     ////// Yaw controller customized //////
//     float e_psi     = des_psi   - imu_yaw;

//     if (e_psi > 0.0){

//         if ( e_psi > 180.0 ){
//             e_psi = -(360.0 - e_psi);
//         }

//     }else if ( e_psi < 0.0 ) {
//         if ( -e_psi < 180.0 ){
//             e_psi = e_psi;
//         }else{
//             e_psi = 360.0 + e_psi;
//         }
//     }

//     // hal.console->printf("Yaw__ %f H_yaw ->  %f e_psi %f \n",imu_yaw,H_yaw, e_psi);
//     float e_psi_sum     = e_psi + e_psi_prev;
//     float Mb3           = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
//     e_psi_prev          = e_psi;
//     // Mb3 = 0.0;
//     ////// Yaw controller customized //////

//     float e_z   = z_des - quad_z;

//     float Kp_z        = 4.0;    // 2.0 (best)
//     float Kd_z        = 1.0;    // 1.0 (best)
//     float F     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    
//     if (F > 20.0){
//         F = 20.0;
//     }

//     if (F < 0.0){
//         F =  0.0;
//     }

//     float function_F1 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
//     float function_F2 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
//     float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
//     float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

//     // hal.console->printf("F1-> %f, F2-> %f, F3-> %f, F4-> %f  \n", function_F1, function_F2, function_F3, function_F4);

//     PWM1 = Inverse_thrust_function(function_F1);
//     PWM2 = Inverse_thrust_function(function_F2);
//     PWM3 = Inverse_thrust_function(function_F3);
//     PWM4 = Inverse_thrust_function(function_F4);

// }


// float ModeMyController::saturation_for_yaw_angle_error(float error){

//     float lim = 30.0;

//     if (error > lim){
//         error = lim;
//     }else if (error < -lim){
//         error = -lim;
//     }else {
//         error = error;
//     }
//     return error;
// }

// float ModeMyController::saturation_for_roll_pitch_angle_error(float error){

//     float lim = 30.0;

//     if (error > lim){
//         error = lim;
//     }else if (error < -lim){
//         error = -lim;
//     }else {
//         error = error;
//     }
//     return error;
// }


// float ModeMyController::sat_I_gain_ph_th(float sum){

//     float lim = 10.0;

//     if (sum > lim){
//         sum = lim;
//     }else if (sum < -lim){
//         sum = -lim;
//     }else {
//         sum = sum;
//     }
//     return sum;
// }

// float ModeMyController::sat_I_gain_psi(float sum){

//     float lim = 20.0;

//     if (sum > lim){
//         sum = lim;
//     }else if (sum < -lim){
//         sum = -lim;
//     }else {
//         sum = sum;
//     }
//     return sum;
// }

// int ModeMyController::Inverse_thrust_function(float Force){
//     int PWM = 1200;

// /////////////////////////// From the quadcopter motors  ///////////////////////////

//     if (battvolt >= 11.5 ){PWM = 1000 * (0.9206 + (sqrtf(12.8953 + 30.3264*Force)/(15.1632)));
//     }else{PWM = 1000 * (0.6021 + (sqrtf(33.2341 + 19.418*Force)/(9.5740)));}
//     if (PWM > 2000){PWM = 2000;}
//     if (PWM < 1000){PWM = 1000;}

//     return PWM;
// }

// void ModeMyController::custom_pwm_code(){

//     // if (RC_Channels::get_radio_in(CH_6) > 1600){

//         copter.init_rc_out();
//         SRV_Channels::cork();
//         hal.rcout->write(0,PWM1);
//         hal.rcout->write(1,PWM2);
//         hal.rcout->write(2,PWM3);
//         hal.rcout->write(3,PWM4);
//         SRV_Channels::push();

//         // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);
// }


// void ModeMyController::channel_arming(){
//     // if (RC_Channels::get_radio_in(CH_6) < 1200){
//     //     copter.motors->armed(false);
//     //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries
//     //     z_des = 0.0;
//     // }
//     // else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
//     //     copter.motors->armed(true);
//     //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

//     //     z_des = 0.0;
//     // }
// }

// void ModeMyController::pilot_input(){

//     // fil_ph_5    = fil_ph_4;    
//     // fil_ph_4    = fil_ph_3;
//     // fil_ph_3    = fil_ph_2;
//     // fil_ph_2    = fil_ph_1;
//     // fil_ph_1    = fil_ph;
//     // fil_ph      = (double)(channel_roll->get_control_in())/100.0;
//     // H_roll      = (fil_ph + fil_ph_1+fil_ph_2+fil_ph_3+fil_ph_4+fil_ph_5)/6.0;
//     // H_roll      = saturation_for_roll_pitch_angle_error(H_roll);
//     H_roll      = (double)(channel_roll->get_control_in())/100.0;
//     H_roll_dot  = (H_roll - H_roll_prev)/400.0;
//     H_roll_prev = H_roll;

//     // fil_th_5    = fil_th_4;    
//     // fil_th_4    = fil_th_3;
//     // fil_th_3    = fil_th_2;
//     // fil_th_2    = fil_th_1;
//     // fil_th_1    = fil_th;
//     // fil_th      = (double)(channel_pitch->get_control_in())/100.0;
//     // H_pitch     = (fil_th + fil_th_1+fil_th_2+fil_th_3+fil_th_4+fil_th_5)/6.0;
//     // H_pitch     = saturation_for_roll_pitch_angle_error(H_pitch);
//     H_pitch     = (double)(channel_pitch->get_control_in())/100.0;
//     H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
//     H_pitch_prev= H_pitch;

//     H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;
//     H_throttle  =  (double)channel_throttle->get_control_in()-500.0;

//     float dt_yaw = 1.0/100.0;
//     H_yaw = wrap_360(H_yaw + H_yaw_rate*dt_yaw);

//     if (H_throttle > -20 && H_throttle < 20){
//         H_throttle = 0.0;
//     }

//     float dt_z = 1.0/15000.0;
//     z_des       =  z_des + H_throttle * dt_z;

//     if (z_des > 5.0){
//         z_des = 5.0;
//     }
//     if (z_des < 0.0){
//         z_des = 0.0;
//     }

// }

// void ModeMyController::imu_read(){
    
//     imu_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
//     imu_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
//     imu_yaw         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
//     imu_roll_dot    =  (ahrs.get_gyro().x);             // degrees/second
//     imu_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
//     imu_yaw_dot     = -(ahrs.get_gyro().z);             // degrees/second

//     // hal.console->printf("Hi..\n");

//     //hal.console->printf("roll_dot: %f pitch_dot %f yaw_dot %f",imu_roll_dot,imu_pitch_dot,imu_yaw_dot);
//     // hal.serial(1)->begin(57600);
//     // hal.console->printf("roll: %f ,pitch: %f ,yaw: %f \n",imu_roll,imu_pitch,imu_yaw);
//     // hal.console->printf("R: %f ,Hr: %f ,P: %f ,Hp: %f \n",imu_roll,H_roll,imu_pitch,H_pitch);

// }

// void ModeMyController::battery_check(){
//     battvolt=copter.battery_volt();
// }

// void ModeMyController::thrust_measurement_code(){

//     time_thrust_mea = (AP_HAL::millis() - tstart)/1000.0;

//     if (time_thrust_mea < 5.0){
//         pwm__thrust_measurement = 1000;
//     }else if (time_thrust_mea > 5.0 && time_thrust_mea < 7.0){
//         pwm__thrust_measurement = 1050;
//     }else if (time_thrust_mea > 7.0 && time_thrust_mea < 9.0){
//         pwm__thrust_measurement = 1100;
//     }else if (time_thrust_mea > 9.0 && time_thrust_mea < 11.0){
//         pwm__thrust_measurement = 1150;
//     }else if (time_thrust_mea > 11.0 && time_thrust_mea < 13.0){
//         pwm__thrust_measurement = 1200;
//     }else if (time_thrust_mea > 13.0 && time_thrust_mea < 15.0){
//         pwm__thrust_measurement = 1250;
//     }else if (time_thrust_mea > 15.0 && time_thrust_mea < 17.0){
//         pwm__thrust_measurement = 1300;
//     }else if (time_thrust_mea > 17.0 && time_thrust_mea < 19.0){
//         pwm__thrust_measurement = 1350;
//     }else if (time_thrust_mea > 19.0 && time_thrust_mea < 21.0){
//         pwm__thrust_measurement = 1400;
//     }else if (time_thrust_mea > 21.0 && time_thrust_mea < 23.0){
//         pwm__thrust_measurement = 1450;
//     }else if (time_thrust_mea > 23.0 && time_thrust_mea < 25.0){
//         pwm__thrust_measurement = 1500;
//     }else if (time_thrust_mea > 25.0 && time_thrust_mea < 27.0){
//         pwm__thrust_measurement = 1550;
//     }else if (time_thrust_mea > 27.0 && time_thrust_mea < 29.0){
//         pwm__thrust_measurement = 1600;
//     }else if (time_thrust_mea > 29.0 && time_thrust_mea < 31.0){
//         pwm__thrust_measurement = 1650;
//     }else if (time_thrust_mea > 31.0 && time_thrust_mea < 33.0){
//         pwm__thrust_measurement = 1700;
//     }else if (time_thrust_mea > 33.0 && time_thrust_mea < 35.0){
//         pwm__thrust_measurement = 1750;
//     }else if (time_thrust_mea > 35.0 && time_thrust_mea < 37.0){
//         pwm__thrust_measurement = 1800;
//     }else if (time_thrust_mea > 37.0 && time_thrust_mea < 39.0){
//         pwm__thrust_measurement = 1850;
//     }else if (time_thrust_mea > 39.0 && time_thrust_mea < 41.0){
//         pwm__thrust_measurement = 1900;
//     }else if (time_thrust_mea > 41.0 && time_thrust_mea < 43.0){
//         pwm__thrust_measurement = 1950;
//     }else if (time_thrust_mea > 43.0 && time_thrust_mea < 45.0){
//         pwm__thrust_measurement = 2000;
//     }else if (time_thrust_mea > 45.0){
//         pwm__thrust_measurement = 1000;
//     }

//     hal.console->printf("time %f, PWM - %d\n",time_thrust_mea,pwm__thrust_measurement); 

//     copter.init_rc_out();
//     SRV_Channels::cork();
//     hal.rcout->write(0,pwm__thrust_measurement);
//     hal.rcout->write(1,1000);
//     hal.rcout->write(2,1000);
//     hal.rcout->write(3,1000);
//     SRV_Channels::push();

// }

// void ModeMyController::system_identification_x_axis()
// {
//     // Pf  = (double)channel_throttle->get_control_in()+1000;

//     // if (t_ph_sys_ID > 5.0 && t_ph_sys_ID < 7.0){
//     //     Pm1 = 50;
//     // }else{
//     //     Pm1 = 0;
//     // }

//     // PWM1 = Pf + Pm1;
//     // PWM2 = Pf - Pm1;
//     // PWM3 = Pf + Pm1;
//     // PWM4 = Pf - Pm1;

// }

// void ModeMyController::system_identification_y_axis()
// {
    
// }

// void ModeMyController::system_identification_z_axis()
// {
    
// }

void ModeMyController::run()
{
//     // const struct Location & loc_ = AP::gps.location();

//     channel_arming();
//     battery_check();
//     pilot_input();
//     imu_read();
//     quad_states();



// ////////////////////// System Identification code //////////////////////

//     // if (RC_Channels::get_radio_in(CH_6) < 1200){
//     //     copter.motors->armed(false);
//     //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries
//     //     z_des = 0.0;
//     //     Pf = 1000;
//     //     PWM1 = Pf;
//     //     PWM2 = Pf;
//     //     PWM3 = Pf;
//     //     PWM4 = Pf;
//     //     t_ph_sys_ID = 0.0;
//     //     flag_ph_sys_ID = 0;
//     // }
//     // else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
//     //     copter.motors->armed(true);
//     //     Pf  = (double)channel_throttle->get_control_in() + 1000;
//     //     PWM1 = Pf;
//     //     PWM2 = Pf;
//     //     PWM3 = Pf;
//     //     PWM4 = Pf;
//     //     t_ph_sys_ID = 0.0;
//     //     flag_ph_sys_ID = 0;
//     //     custom_pwm_code();
//     // }else if (RC_Channels::get_radio_in(CH_6) > 1600){
//     //     copter.motors->armed(true);

//     //     if ( flag_ph_sys_ID == 0){
//     //         t_start_ph_sys_ID = AP_HAL::millis();
//     //         flag_ph_sys_ID = 1;
//     //     }
//     //     t_ph_sys_ID = (AP_HAL::millis() - t_start_ph_sys_ID)/1000.0;
//     //     system_identification_x_axis();
//     //     custom_pwm_code();
//     // }


////////////////////////////////////////////////////////////////////

// ////////////////////// Main code //////////////////////

//     // if (RC_Channels::get_radio_in(CH_6) < 1200){
//     //     copter.motors->armed(false);
//     //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries
//     //     z_des = 0.0;
//     // }
//     // else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
//     //     custom_PID_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, 0.0 ,0.0); 
//     //     custom_pwm_code();
//     // }else if (RC_Channels::get_radio_in(CH_6) > 1600){

//     //     PID_pos_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, 0.0 ,0.0);
//     //     custom_pwm_code();
//     // }
// //////////////////////////////////////////////////////////////////

// }

// void ModeMyController::quad_states(){

//     // Position in inertial reference frame
//     quad_x =  inertial_nav.get_position().x / 100.0;
//     quad_y =  inertial_nav.get_position().y / 100.0;

//     // position in body reference frame
//     quad_x =  (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y) - quad_x_ini;
//     quad_y = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y) - quad_y_ini;

//     quad_z =  inertial_nav.get_position().z / 100.0 - quad_z_ini;

//     // linear velocity in inertial frame of reference
//     quad_x_dot =  inertial_nav.get_velocity().x /100.0;
//     quad_y_dot =  inertial_nav.get_velocity().y /100.0;
//     quad_z_dot =  inertial_nav.get_velocity().z /100.0;

//     // linear velocity in body reference frame
//     quad_x_dot =  (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y);
//     quad_y_dot = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y);

}