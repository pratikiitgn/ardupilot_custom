
#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Motors/AP_Motors_Class.h>
#include "mycontroller_usercode.h"
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  
// #include "AP_Math.h"

#define ESC_HZ 490
#ifndef PI
  #define PI               3.14159265358979f
#endif

int code_starting_flag = 0;

int yaw_flag_start  = 0;

float current_time  = 0.0;
float IITGN_text_start_time = 0.0;
float IITGN_text_start_time_flag = 0.0;
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

float z_des_init = 0.0;
float yaw_initially = 0.0;

uint16_t PWM1 = 1000;
uint16_t PWM2 = 1000;
uint16_t PWM3 = 1000;
uint16_t PWM4 = 1000;

uint16_t Pf  = 0;
uint16_t Pm1 = 0;
uint16_t Pm2 = 0;
uint16_t Pm3 = 0;
uint16_t Pm4 = 0;

int arm_disarm_flag = 0;

float Force         = 0.0;
float Mb1       = 0.0;
float Mb2       = 0.0;
float Mb3       = 0.0;

float t1_IITGN_traj = 0.0;
float t2_IITGN_traj = 0.0;

float sty_IITGN_traj = 0.0;
float stz_IITGN_traj = 0.0;
float eny_IITGN_traj = 0.0;
float enz_IITGN_traj = 0.0;

float light_on_off = 0.0;

void ModeStabilize::run()
{

    if (code_starting_flag == 0){

        // To intialize the code
        copter.init_rc_out();
        hal.rcout->set_freq( 15, ESC_HZ); //0xFF  0x0F->b'00001111'
        hal.rcout->enable_ch(0);
        hal.rcout->enable_ch(1);
        hal.rcout->enable_ch(2);
        hal.rcout->enable_ch(3);

        code_starting_flag = 1;

    }else{

    ///////////// Arming checks  /////////////
        if (copter.motors->armed()){
            arm_disarm_flag = 1;
        }else{
            arm_disarm_flag = 0;
        }
    
    ///////////// getting pilot inputs  /////////////
        pilot_input();

    ///////////// getting states  /////////////
        quad_states();

    ///////////// For attitude controller controller  /////////////
        attitude_altitude_controller();

    }

}

void ModeStabilize::attitude_altitude_controller(){
    if (RC_Channels::get_radio_in(CH_6) < 1200){

            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

            PWM1 = 1000;
            PWM2 = 1000;
            PWM3 = 1000;
            PWM4 = 1000;

            // initialize the states
            H_yaw           = 360.0 - (ahrs.yaw_sensor) / 100.0;
            yaw_initially   = 360.0 - (ahrs.yaw_sensor)   / 100.0;     // degrees 

            quad_x_ini =   inertial_nav.get_position().x / 100.0;
            quad_y_ini =  -inertial_nav.get_position().y / 100.0;
            quad_z_ini =   inertial_nav.get_position().z / 100.0;

            // quad_x_ini = (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y);
            // quad_y_ini = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y);
            // quad_z_ini = quad_z;

        }
        else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
            if (copter.motors->armed()){
                custom_PID_controller(H_roll, H_pitch, H_yaw, 0.0 ,0.0, 0.0, z_des ,0.0);
                
                quad_x_ini =   inertial_nav.get_position().x / 100.0;
                quad_y_ini =  -inertial_nav.get_position().y / 100.0;
                IITGN_text_start_time_flag = AP_HAL::millis()/1000.0;

            }
        }else if (RC_Channels::get_radio_in(CH_6) > 1600){
            if(copter.motors->armed()){
                IITGN_text_traj_planning();
                custom_position_controller(x_des, y_des, z_des, x_des_dot, y_des_dot, z_des_dot, H_yaw, 0.0);
            }
        }
}

void ModeStabilize::IITGN_text_traj_planning(){

Vector3f Z(0,0,0);
Vector3f A(0,0,2);
Vector3f B(0,0,5);
Vector3f C(0,1,5);
Vector3f D(0,1,2);
Vector3f E(0,2.5,2);
Vector3f FF(0,2.5,5);
Vector3f G(0,1.5,5);
Vector3f H(0,3.5,5);
Vector3f I(0,6,4.5);
Vector3f J(0,6,5);
Vector3f K(0,4,5);
Vector3f L(0,4,2);
Vector3f M(0,6,2);
Vector3f N(0,6,3.5);
Vector3f O(0,5,3.5);
Vector3f P(0,7,2);
Vector3f Q(0,7,5);
Vector3f R(0,9,2);
Vector3f S(0,9,5);
Vector3f Y(0,9,0);

float Pos_array[21][3] = {{0,0,0},
                         {0,0,2},
                         {0,0,5},
                         {0,1,5},
                         {0,1,2},
                         {0,2.5,2},
                         {0,2.5,5},
                         {0,1.5,5},
                         {0,3.5,5},
                         {0,6,4.5},
                         {0,6,5},
                         {0,4,5},
                         {0,4,2},
                         {0,6,2},
                         {0,6,3.5},
                         {0,5,3.5},
                         {0,7,2},
                         {0,7,5},
                         {0,9,2},
                         {0,9,5},
                         {0,9,0}};

float light[21][1] = {{0},
             {1},
             {0},
             {1},
             {0},
             {1},
             {1},
             {1},
             {0},
             {1},
             {1},
             {1},
             {1},
             {1},
             {1},
             {0},
             {1},
             {1},
             {1},
             {0},
             {0}};


float V_max = 0.5;

float  tZ = 0;
float  tA = tZ + (norm_2(A , Z)/V_max);
float  tB = tA + (norm_2(B , A)/V_max);
float  tC = tB + (norm_2(C , B)/V_max);
float  tD = tC + (norm_2(D , C)/V_max);
float  tE = tD + (norm_2(E , D)/V_max);
float  tF = tE + (norm_2(FF , E)/V_max);
float  tG = tF + (norm_2(G , FF)/V_max);
float  tH = tG + (norm_2(H , G)/V_max);
float  tI = tH + (norm_2(I , H)/V_max);
float  tJ = tI + (norm_2(J , I)/V_max);
float  tK = tJ + (norm_2(K , J)/V_max);
float  tL = tK + (norm_2(L , K)/V_max);
float  tM = tL + (norm_2(M , L)/V_max);
float  tN = tM + (norm_2(N , M)/V_max);
float  tO = tN + (norm_2(O , N)/V_max);
float  tP = tO + (norm_2(P , O)/V_max);
float  tQ = tP + (norm_2(Q , P)/V_max);
float  tR = tQ + (norm_2(R , Q)/V_max);
float  tS = tR + (norm_2(S , R)/V_max);
float  tY = tS + (norm_2(Y , S)/V_max);

// hal.console->printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f\n", tO,tP,tQ,tR,tS,tY);

float t_array[21][1] = {{tZ},
                        {tA},
                        {tB},
                        {tC},
                        {tD},
                        {tE},
                        {tF},
                        {tG},
                        {tH},
                        {tI},
                        {tJ},
                        {tK},
                        {tL},
                        {tM},
                        {tN},
                        {tO},
                        {tP},
                        {tQ},
                        {tR},
                        {tS},
                        {tY}};

// hal.console->printf("%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n", t_array[6][0],t_array[7][0],t_array[8][0],t_array[9][0],t_array[10][0],t_array[11][0]);

IITGN_text_start_time = AP_HAL::millis()/1000.0 - IITGN_text_start_time_flag;
float tt = IITGN_text_start_time;

for (int i = 0; i < 21; i++) {

     if (IITGN_text_start_time >= t_array[i][0]){
        t1_IITGN_traj  = t_array[i][0];
        t2_IITGN_traj  = t_array[i+1][0];
        sty_IITGN_traj = Pos_array[i][1];
        stz_IITGN_traj = Pos_array[i][2];
        eny_IITGN_traj = Pos_array[i+1][1];
        enz_IITGN_traj = Pos_array[i+1][2];
        light_on_off   = light[i][0];
        // hal.console->printf("%5.3f\n",t_array[i][0]);
     }
}

// hal.console->printf("%5.3f, %5.3f\n",t1_IITGN_traj,t2_IITGN_traj);

float ay =  min_acc_first_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);
float by = min_acc_second_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);
float cy =  min_acc_third_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);
float dy = min_acc_fourth_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);

float az =  min_acc_first_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);
float bz = min_acc_second_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);
float cz =  min_acc_third_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);
float dz = min_acc_fourth_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);

x_des = 0.0;
y_des = ay*tt*tt*tt + by*tt*tt + cy*tt + dy;
z_des = az*tt*tt*tt + bz*tt*tt + cz*tt + dz;

if (IITGN_text_start_time > 92.3101){
    x_des = 0.0;
    y_des = 9.0;
    z_des = 0.0;
}

hal.console->printf("t - %5.3f, xd - %5.3f, yd - %5.3f, zd - %5.3f \n",tt, x_des, y_des, z_des );

}

float ModeStabilize::min_acc_first_coefficient(float t1, float t2, float st, float en){
    float a = (2*(en - st)) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
    return a;
}

float ModeStabilize::min_acc_second_coefficient(float t1, float t2, float st, float en){
    float b = -(3*(t1 + t2)*(en - st)) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
    return b;
}

float ModeStabilize::min_acc_third_coefficient(float t1, float t2, float st, float en){
    float c = (6*t1*t2*(en - st)) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
    return c;
}

float ModeStabilize::min_acc_fourth_coefficient(float t1, float t2, float st, float en){
    float d = (en*t1*t1*t1 - 3*en*t1*t1*t2 + 3*st*t1*t2*t2 - st*t2*t2*t2) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
    return d;
}

float ModeStabilize::norm_2(Vector3f A, Vector3f B){

    float norm__  = (A[0] - B[0]) * (A[0] - B[0]) + (A[1] - B[1]) * (A[1] - B[1]) + (A[2] - B[2]) * (A[2] - B[2]);
    norm__ = sqrtf(norm__ );
    return norm__;

}

void ModeStabilize::quad_states(){
    // Position in inertial reference frame
    quad_x =    inertial_nav.get_position().x / 100.0;
    quad_y =   -inertial_nav.get_position().y / 100.0;
    quad_z =    inertial_nav.get_position().z / 100.0;

    quad_x =   quad_x - quad_x_ini;
    quad_y =   quad_y - quad_y_ini;
    quad_z =   quad_z - quad_z_ini;

    // linear velocity in inertial frame of reference
    quad_x_dot =    inertial_nav.get_velocity().x /100.0;
    quad_y_dot =   -inertial_nav.get_velocity().y /100.0;
    quad_z_dot =    inertial_nav.get_velocity().z /100.0;

    // linear velocity in body reference frame
    // quad_x_dot =  (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y);
    // quad_y_dot = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y);

    imu_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    imu_roll_dot    =  (ahrs.get_gyro().x);             // degrees/second
    imu_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
    imu_yaw_dot     = -(ahrs.get_gyro().z);             // degrees/second
    // float imu_yaw_rad   = imu_yaw * PI /180.0;

    // position in body reference frame
    // quad_x =  (cosf(yaw_initially)*quad_x + sinf(yaw_initially)*quad_y) - quad_x_ini;
    // quad_y = (-sinf(yaw_initially)*quad_x + cosf(yaw_initially)*quad_y) - quad_y_ini;
    // quad_z =  inertial_nav.get_position().z / 100.0 - quad_z_ini;

    // hal.console->printf("roll %5.3f, pitch %5.3f, yaw %5.3f \n",imu_roll, imu_pitch, imu_yaw);

}

void ModeStabilize::custom_PID_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

    float mass = 1.236;
    float arm_length = 0.161;
    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24;
    }else{
         FM_devided_FF = 0.31;
    }

    float Kp_phi      = 0.025;   // 0.012 (best) 
    float Kp_theta    = 0.025;   // 0.015 (best) 
    float Kp_psi      = 0.1;     // 0.1 (best)

    float Kd_phi      = 0.35;     // 0.2;
    float Kd_theta    = 0.35;    // 0.15; 
    float Kd_psi      = 0.25;     // 0.2

    float Ki_phi      = 0.00;    // 0.0005 (lab 1)
    float Ki_theta    = 0.00;    // 0.0005 (lab 1)
    float Ki_psi      = 0.00;    // 0.0005 (lab 1)

    float e_phi       = des_phi   - imu_roll;
    float e_theta     = des_theta - imu_pitch;

    float e_phi_sum   = e_phi + e_phi_prev;
    Mb1 = Kp_phi * saturation_for_roll_pitch_angle_error(e_phi)  \
        + Kd_phi * (des_phi_dot - imu_roll_dot) \
        + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
    e_phi_prev = e_phi;

    float e_theta_sum   = e_theta + e_theta_prev;
    Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta) \
        + Kd_theta  * (des_theta_dot - imu_pitch_dot) \
        + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
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

    float e_psi_sum     = e_psi + e_psi_prev;
    Mb3           = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
    e_psi_prev          = e_psi;
    ////// Yaw controller customized //////

///////////////////// Altitude controller /////////////////////
    float e_z   = z_des - quad_z;

    // hal.console->printf("Zd-> %5.2f, z-> %5.2f\n",z_des,quad_z);

    if (e_z > 5.0){
        e_z = 5.0;
    }
    if (e_z < -5.0){
        e_z = -5.0;
    }

    float Kp_z        = 10.0;    // 2.0 (best)
    float Kd_z        = 5.0;    // 1.0 (best)
    Force     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    // Force     =  mass * GRAVITY_MSS ;
    // Force     

    if (Force > 20.0){
        Force = 20.0;
    }

    if (Force < 0.0){
        Force =  0.0;
    }

    float function_F1 = Force/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = Force/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = Force/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = Force/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

    // PWM1 = 1000;
    // PWM2 = 1000;
    // PWM3 = 1000;
    // PWM4 = 1000;

}

void ModeStabilize::custom_position_controller(float x_des_func, float y_des_func, float z_des_func, float x_des_dot_func, float y_des_dot_func, float z_des_dot_func, float des_psi, float des_psi_dot){

    // Translational Position controller 
    // hal.console->printf("%5.3f, %5.3f,%5.3f\n",quad_x, quad_y, quad_z);

    float Kp_x = 10.0;
    float Kp_y = 10.0;

    float Kd_x = 5.0;
    float Kd_y = 5.0;

    float e_x  = x_des_func - quad_x ;
    float e_y  = y_des_func - quad_y ;

    float e_x_dot = x_des_dot_func - quad_x_dot;
    float e_y_dot = y_des_dot_func - quad_y_dot;

    float des_theta_x   =   Kp_x * (e_x) + Kd_x * (e_x_dot);
    float des_phi_y     = -(Kp_y * (e_y) + Kd_y * (e_y_dot));

    // des_phi_y           = H_roll;

    float des_phi_dot   = 0.0;
    float des_theta_dot = 0.0;

    float mass = 1.236;
    float arm_length = 0.161;
    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24;
    }else{
         FM_devided_FF = 0.31;
    }

    float Kp_phi      = 0.025;   // 0.012 (best) 
    float Kp_theta    = 0.025;   // 0.015 (best) 
    float Kp_psi      = 0.1;     // 0.1 (best)

    float Kd_phi      = 0.35;     // 0.2;
    float Kd_theta    = 0.35;    // 0.15; 
    float Kd_psi      = 0.25;     // 0.2

    float Ki_phi      = 0.00;    // 0.0005 (lab 1)
    float Ki_theta    = 0.00;    // 0.0005 (lab 1)
    float Ki_psi      = 0.00;    // 0.0005 (lab 1)

    float e_phi       = des_phi_y   - imu_roll;
    float e_theta     = des_theta_x - imu_pitch;

    float e_phi_sum   = e_phi + e_phi_prev;
    Mb1 = Kp_phi * saturation_for_roll_pitch_angle_error(e_phi)  \
        + Kd_phi * (des_phi_dot - imu_roll_dot) \
        + Ki_phi * sat_I_gain_ph_th(e_phi_sum);
    e_phi_prev = e_phi;

    float e_theta_sum   = e_theta + e_theta_prev;
    Mb2 = Kp_theta  * saturation_for_roll_pitch_angle_error(e_theta) \
        + Kd_theta  * (des_theta_dot - imu_pitch_dot) \
        + Ki_theta * sat_I_gain_ph_th(e_theta_sum);
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
    float e_psi_sum = e_psi + e_psi_prev;
    Mb3             = -(Kp_psi  * saturation_for_yaw_angle_error(e_psi)  + Kd_psi  * (des_psi_dot - imu_yaw_dot)) + Ki_psi * sat_I_gain_psi(e_psi_sum);;    
    e_psi_prev      = e_psi;
    ////// Yaw controller customized //////

///////////////////// Altitude controller /////////////////////
    float e_z     = z_des_func     - quad_z;
    float e_z_dot = z_des_dot_func - quad_z_dot;
    // hal.console->printf("Zd-> %5.2f, z-> %5.2f\n",z_des,quad_z);

    if (e_z > 5.0){
        e_z = 5.0;
    }
    if (e_z < -5.0){
        e_z = -5.0;
    }

    float Kp_z        = 10.0;    // 2.0 (best)
    float Kd_z        = 5.0;    // 1.0 (best)
    Force     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (e_z_dot);
    // Force     =  mass * GRAVITY_MSS ;

    if (Force > 20.0){
        Force = 20.0;
    }

    if (Force < 0.0){
        Force =  0.0;
    }

    float function_F1 = Force/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = Force/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = Force/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = Force/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

    // PWM1 = 1000;
    // PWM2 = 1000;
    // PWM3 = 1000;
    // PWM4 = 1000;

    // hal.console->printf("%5.3f, %5.3f\n",quad_x, quad_x_dot);

}

void ModeStabilize::pilot_input(){

    H_roll      = (double)(channel_roll->get_control_in())/100.0;
    H_roll_dot  = (H_roll - H_roll_prev)/400.0;
    H_roll_prev = H_roll;

    H_pitch     = (double)(channel_pitch->get_control_in())/100.0;
    H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
    H_pitch_prev= H_pitch;

    if (copter.motors->armed()){
        H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;
        float dt_yaw = 1.0/100.0;
        H_yaw = wrap_360(H_yaw + H_yaw_rate*dt_yaw);
        // hal.console->printf("H Roll %5.3f, H Roll %5.3f, H Roll %5.3f \n",H_roll, H_pitch, H_yaw);
    }

    H_throttle  =  (double)channel_throttle->get_control_in()-500.0;

    if (H_throttle < 0){
        z_des = 0.0;
    }

    if (H_throttle > 0){
        z_des = H_throttle/100.0;;
    }

    if (z_des > 5.0){
        z_des = 5.0;
    }
    if (z_des < 0.0){
        z_des = 0.0;
    }

    // if (H_throttle > -20 && H_throttle < 20){
    //     H_throttle = 0.0;
    // }

    // float dt_z = 1.0/15000.0;
    // z_des       =  z_des + H_throttle * dt_z;

}

float ModeStabilize::saturation_for_roll_pitch_angle_error(float error){

    float lim = 25.0;

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

int ModeStabilize::Inverse_thrust_function(float Force_){
    int PWM = 1200;

/////////////////////////// From the quadcopter motors  ///////////////////////////

    if (battvolt >= 11.5 ){PWM = 1000 * (0.9206 + (sqrtf(12.8953 + 30.3264*Force_)/(15.1632)));
    }else{PWM = 1000 * (0.6021 + (sqrtf(33.2341 + 19.418*Force_)/(9.5740)));}
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