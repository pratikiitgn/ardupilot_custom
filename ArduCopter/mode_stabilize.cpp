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
#define PI 3.14
float mp = 0.1;
float l1 = 1.5;
float m1 = 1.236;

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

float arm_length = 0.1768;

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

// Gains for geometric controller
float KR1           = 0.0;
float KOmega1       = 0.0;
float KI1           = 0.0;

float KR2           = 0.0;
float KOmega2       = 0.0;
float KI2           = 0.0;

float KR3           = 0.0;
float KOmega3       = 0.0;
float KI3           = 0.0;

float KR1_old       = 0.0;
float KOmega1_old   = 0.0;
float KI1_old       = 0.0;

Vector3f e_I_val_old (0.0,0.0,0.0);

// For altitude controller
float e_z_sum = 0.0;

// For cable attitude controller
Vector3f q1d_prev(0.0,0.0,0.0);
Vector3f q1_current(0.0,0.0,1.0);
Vector3f q1_current_prev(0.0,0.0,1.0);
Vector3f Omegad_1_dot(0.0,0.0,1.0);
Vector3f Omegad_1_prev(0.0,0.0,1.0);

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

            ///////////// Checking batter voltage  /////////////
                battery_check();
            ///////////// Taking pilot inputs  /////////////
                pilot_input();
            ///////////// Getting states of quadcopter /////////////
                quad_states();
            ///////////// Human operator's commands /////////////
                // void Human_operators_commands(float des_xp_x_vel,float des_xp_y_vel,float des_xp_z_vel,float des_yaw_system_vel);
                // Write down this code later on
                float des_yaw_system = H_yaw;
            ///////////// PD_Controller for payload position tracking /////////////
                Vector3f zero_vec(0.0,0.0,0.0);
                Vector3f FD(PD_controller_payload(zero_vec, zero_vec, zero_vec, zero_vec));
                // hal.console->printf("FD1-> %f,FD2-> %f,FD3-> %f \n", FD[1],FD[1],FD[2]);

            ///////////// Desired attitude of the cables /////////////
                float theta_pd = 0.0; // (in degres) Desired attitude of the cable at equilibrium condition
                Vector3f q1d(Desired_cable_attitude_estimation(des_yaw_system,theta_pd, FD));
                Vector3f q1d_dot = (q1d - q1d_prev)*10.0;
                q1d_prev = q1d;
                Vector3f Omegad_1 = Matrix_vector_mul(hatmap(q1d),q1d_dot);
                Omegad_1_dot    = (Omegad_1 - Omegad_1_prev)*10.0;
                Omegad_1_prev   = Omegad_1;
                // hal.console->printf("q1d-> %f,q2d-> %f,q3d-> %f \n", q1d[0],q1d[1],q1d[2]);

            ///////////// Design of Perpendicular components /////////////
                q1_current      = qc;
                Vector3f q1_dot = (q1_current - q1_current_prev)*10.0;
                q1_current_prev = q1_current;

                float Kq11    = 1.0;  //   (TB good)
                float Kq12    = 1.0;  //   (TB good)
                float Kq13    = 1.0;  //   (TB good)

                float Kw11    = 1.0;  //   (TB good)
                float Kw12    = 1.0;  //   (TB good)
                float Kw13    = 1.0;  //   (TB good)

                Matrix3f kq1(
                            Kq11,0.0,0.0,
                            0.0,Kq12,0.0,
                            0.0,0.0,Kq13
                            );

                Matrix3f kw1(
                            Kw11,0.0,0.0,
                            0.0,Kw12,0.0,
                            0.0,0.0,Kw13
                            );

                Vector3f e_q1        = Matrix_vector_mul(hatmap(q1d),q1_current);
                Vector3f e_omega1    = Omega_c - Two_vec_cross_product(q1_current,Two_vec_cross_product(q1_current,Omegad_1));
                
                Vector3f fist_term   = -Matrix_vector_mul(kq1,e_q1);
                Vector3f second_term = -Matrix_vector_mul(kw1,e_omega1);
                Vector3f third_term  = -constant_vec_multiplication(Two_vec_dot_product(q1_current,Omegad_1),q1_dot);
                Vector3f forth_term  = -Two_vec_cross_product(q1_current,Two_vec_cross_product(q1_current,Omegad_1_dot));
                Vector3f fifth_term  = constant_vec_multiplication((m1/mp),FD);

                Vector3f u1_perpendicular;
                u1_perpendicular = fist_term + second_term + third_term + forth_term + fifth_term;
                u1_perpendicular = Matrix_vector_mul(hatmap(q1_current),u1_perpendicular);
                u1_perpendicular = constant_vec_multiplication(m1*l1,u1_perpendicular);

            ///////////// Quadcopter desired attitude /////////////

                Vector3f b3d_quadcopter(constant_vec_multiplication(1/two_norm(u1_perpendicular), u1_perpendicular));
                Vector3f sd(cosf(des_yaw_system*PI/180), sinf(des_yaw_system*PI/180), 0.0);
                Vector3f b2d_quadcopter;
                b2d_quadcopter = Two_vec_cross_product(b3d_quadcopter,sd);
                b2d_quadcopter = constant_vec_multiplication(1/two_norm(b2d_quadcopter),b2d_quadcopter);
                Vector3f b1d_quadcopter;
                b1d_quadcopter = Two_vec_cross_product(b2d_quadcopter,b3d_quadcopter);
                b2d_quadcopter = constant_vec_multiplication(1/two_norm(b1d_quadcopter),b1d_quadcopter);

                Matrix3f Rd_quadcopter(b1d_quadcopter[0], b2d_quadcopter[0], b3d_quadcopter[0],
                                       b1d_quadcopter[1], b2d_quadcopter[1], b3d_quadcopter[1],
                                       b1d_quadcopter[2], b2d_quadcopter[2], b3d_quadcopter[2]);
                
            ///////////// For attitude controller controller  /////////////
                custom_geometric_controller(H_roll,H_pitch,H_yaw,0.0,0.0,0.0,quad_z_ini,0.0);

        }
}

void ModeStabilize::battery_check(){
    battvolt=copter.battery_volt();
}

void ModeStabilize::custom_geometric_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

    if (RC_Channels::get_radio_in(CH_6) < 1200){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

        //// Initializing the states of the quadcopters
        quad_z_ini =  inertial_nav.get_position().z / 100.0;
        H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

        float quad_x_ini_inertial =  inertial_nav.get_position().x / 100.0;
        float quad_y_ini_inertial =  inertial_nav.get_position().y / 100.0;

        quad_x_ini =  cosf(yaw_initially)*quad_x_ini_inertial + sinf(yaw_initially)*quad_y_ini_inertial;
        quad_y_ini = -sinf(yaw_initially)*quad_x_ini_inertial + cosf(yaw_initially)*quad_y_ini_inertial;

            //// Initial the PWMs
            if (copter.motors->armed()){
                // PWM1 = 1200;
                // PWM2 = 1200;
                // PWM3 = 1200;
                // PWM4 = 1200;

                PWM1 = 1000;
                PWM2 = 1000;
                PWM3 = 1000;
                PWM4 = 1000;

                // hal.console->printf("Sending PWM from first code\n");
                // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);
            }else{
                PWM1 = 1000;
                PWM2 = 1000;
                PWM3 = 1000;
                PWM4 = 1000;
            }

        }
        else {
            if (copter.motors->armed()){

            ///////////////////// Altitude controller /////////////////////

                float e_z       = z_des - quad_z;
                float e_z_dot   = (des_z_dot - quad_z_dot);
                e_z_sum   = e_z + e_z_sum;
                float max_e_z_sum = 2.5;
                if (e_z_sum > 2){
                    e_z_sum = max_e_z_sum;
                }
                if (e_z_sum < -2){
                    e_z_sum = -max_e_z_sum;
                }

                float Kp_z      = 5.0;    // 2.0 (best)
                float Kd_z      = 2.0;    // 1.0 (best)
                float Ki_z      = 0.1;    //      (best)
                // F     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
                F     =  11.0 + Kp_z * e_z + Kd_z * e_z_dot + Ki_z * e_z_sum;
                
                // hal.console->printf("z-> %f, z_des-> %f, F-> %f\n",quad_z,z_des,F);

                // F     =  11.0;

                if (F > 20.0){ F = 20.0; }
                if (F < 0.0){ F =  0.0;  }

            ///////////////////// geometric attitude controller /////////////////////

                Vector3f rpy(imu_roll*PI/180.0,imu_pitch*PI/180.0,imu_yaw*PI/180.0);
                Vector3f Omega(imu_roll_dot*PI/180.0,imu_pitch_dot*PI/180.0,imu_yaw_dot*PI/180.0);

                float des_phi_offset    = 0.0;
                float des_theta_offset  = 0.0;

                des_phi     = des_phi + des_phi_offset;
                des_theta   = des_theta + des_theta_offset;

                Vector3f rpy_d(des_phi*PI/180.0,des_theta*PI/180.0,des_psi*PI/180.0);
                Vector3f Omegad(des_phi_dot*PI/180.0,des_theta_dot*PI/180.0,des_psi_dot*PI/180.0);

                Matrix3f R(eulerAnglesToRotationMatrix(rpy));
                Matrix3f Rd(eulerAnglesToRotationMatrix(rpy_d));

                float c2 = 2.0;
                // float c1 = 1.0;

                Vector3f e_R_val        = e_R(R,Rd);
                Vector3f e_Omega_val    = e_Omega(R,Rd,Omega,Omegad);
                Vector3f e_I_val        = e_Omega_val + Vector3f(e_R_val[0]*c2,e_R_val[1]*c2,e_R_val[2]*c2);
                Vector3f e_I_val_sum    = sat_e_I(e_I_val_old + e_I_val);
                e_I_val_old             = e_I_val;

                // hal.console->printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",R[0][0],R[0][1],R[0][2],R[1][0],R[1][1],R[1][2],R[2][0],R[2][1],R[2][2]);
                // hal.serial(2)->printf("%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",arm_disarm_flag,quad_x,quad_y,quad_z,x_des,y_des,z_des,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);
                // hal.serial(2)->printf("%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",arm_disarm_flag,quad_x,quad_y,quad_z,x_des,y_des,z_des,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);
                // hal.console->printf("%f,%f,%f\n",e_R_log[0],e_R_log[1],e_R_log[2]);
                // Auto tuning Gains 

                // KR1         = KR1_old     + c1*e_R_val[0]*e_Omega_val[0];
                // KR1_old     = KR1;

                // KOmega1     = KOmega1_old + c2*e_R_val[0]*e_Omega_val[0];
                // KOmega1_old = KOmega1;

                // KI1         = KI1_old     + e_I_val * (Vector3f(e_R_val[0]*c2,e_R_val[1]*c2,e_R_val[2]*c2) + e_Omega_val);
                // KI1_old     = KI1;

                // float KI1_lim = 5;

                // if (KI1 > KI1_lim){
                //     KI1 = KI1_lim;
                // }
                // if (KI1 < KI1_lim){
                //     KI1 = KI1_lim;
                // }

            /////////////////////// Manual gain tuning  ///////////////////////

                KR1         = 0.9;  // 0.6 (TB best)  //0.4
                KOmega1     = 21;   // 10.5(TB best)  //0.5
                KI1         = 0.0;  // 0.1 (TB best)  //0.1

                KR2         = 1.3;  // 1.3  (TB good)
                KOmega2     = 24.0; // 24 (TB good)
                KI2         = 0.0;  // 0.1  (TB good)

                KR3         = 4.5;  // 6.0  (TB good)
                KOmega3     = 14.5; // 10.5 (TB good)
                KI3         = 0.1;  // 0.1  (TB good)

                Matrix3f KR(
                            KR1,0.0,0.0,
                            0.0,KR2,0.0,
                            0.0,0.0,KR3
                            );
                Matrix3f KOmega(
                            KOmega1,0.0,0.0,
                            0.0,KOmega2,0.0,
                            0.0,0.0,KOmega3
                            );
                Matrix3f KI(
                            KI1,0.0,0.0,
                            0.0,KI2,0.0,
                            0.0,0.0,KI3
                            );
                // Intertia matrix
                Matrix3f JJ(
                        0.0113, 0.0 , 0.0,
                        0.0, 0.0133,0.0,
                        0.0,0.0,0.0187
                );

                Vector3f M( Matrix_vector_mul(KR,e_R_val) + Matrix_vector_mul(KOmega,e_Omega_val) + Matrix_vector_mul(KI,e_I_val_sum) + Omega % Matrix_vector_mul(JJ,Omega));
                // M = ( Matrix_vector_mul(KR,e_R_val) + Matrix_vector_mul(KOmega,e_Omega_val));

                Mb1 =  -M[0];
                Mb2 =  -M[1];
                Mb3 =  M[2];
                // Mb1 =  0.0;
                // Mb2 =  0.0;
                // Mb3 =  0.0;

                float FM_devided_FF ;
                if (battvolt >= 11.5 ){
                    FM_devided_FF = 0.31; //0.24
                }else{
                    FM_devided_FF = 0.31;
                }

                float function_F1 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
                float function_F2 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
                float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
                float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

                PWM1 = Inverse_thrust_function(function_F1);
                PWM2 = Inverse_thrust_function(function_F2);
                PWM3 = Inverse_thrust_function(function_F3);
                PWM4 = Inverse_thrust_function(function_F4);

                // PWM1 = 1300;
                // PWM2 = 1300;
                // PWM3 = 1300;
                // PWM4 = 1300;

                // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);
                // hal.console->printf("%f,%f,%f\n",Mb1,Mb2,Mb3);
                // hal.console->printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",H_roll,imu_roll,e_R_val[0],H_pitch,imu_pitch,e_R_val[1],H_yaw,imu_yaw,e_R_val[2]);
                // hal.console->printf("%f,%f,%f,%f,%f,%f\n",H_roll,imu_roll,e_R_val[0],H_pitch,imu_pitch,e_R_val[1]);
                // hal.console->printf("%f,%f,%f\n",e_R_val[0],e_R_val[1],e_R_val[2]);

            }else{
                PWM1 = 1000;
                PWM2 = 1000;
                PWM3 = 1000;
                PWM4 = 1000;
            }
        }

}

// void ModeStabilize::Human_operators_commands(){

// }

Vector3f ModeStabilize::constant_vec_multiplication(float a, Vector3f b){
    Vector3f vec;
    vec[0] = a*b[0];
    vec[1] = a*b[1];
    vec[2] = a*b[2];
    return vec;
}

float ModeStabilize::two_norm(Vector3f v){
    float norm_val = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    return sqrtf(norm_val);
}

Vector3f ModeStabilize::Two_vec_cross_product(Vector3f vect_A, Vector3f vect_B){
    Vector3f cross_P;
    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];

    return cross_P;
}

float ModeStabilize::Two_vec_dot_product(Vector3f v1, Vector3f v2){

    return (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]);
}


Vector3f ModeStabilize::Desired_cable_attitude_estimation(float des_yaw_system, float theta_pd, Vector3f FD){
    Vector3f sd(cosf(des_yaw_system*PI/180), sinf(des_yaw_system*PI/180), 0.0);
    Vector3f FD_divided_FD_norm(constant_vec_multiplication(1/two_norm(FD), FD));

    Vector3f FD_cross_sd = Two_vec_cross_product(FD,sd);
    FD_cross_sd = constant_vec_multiplication(1/two_norm(FD_cross_sd),FD_cross_sd);
    Vector3f FD_cross_sd_cross_FD = Two_vec_cross_product(Two_vec_cross_product(FD,sd),FD);
    FD_cross_sd_cross_FD = constant_vec_multiplication(1/two_norm(FD_cross_sd_cross_FD),FD_cross_sd_cross_FD);

    Vector3f r1d(sinf(theta_pd*PI/180), 0.0, cosf(theta_pd*PI/180));

    Matrix3f Q_matrix(FD_cross_sd_cross_FD[0], FD_cross_sd[0], FD_divided_FD_norm[0],
    FD_cross_sd_cross_FD[1], FD_cross_sd[1], FD_divided_FD_norm[1],
    FD_cross_sd_cross_FD[2], FD_cross_sd[2], FD_divided_FD_norm[2]);

    Vector3f q1d(Matrix_vector_mul(Q_matrix,r1d));
    return q1d;
}

Vector3f ModeStabilize::PD_controller_payload(Vector3f xp, Vector3f xpd, Vector3f xp_dot, Vector3f xpd_dot){

    float Kxp1 = 0.0;
    float Kxp2 = 0.0;
    float Kxp3 = 0.0;

    float Kxp_dot_1 = 0.0;
    float Kxp_dot_2 = 0.0;
    float Kxp_dot_3 = 0.0;

    Matrix3f Kxp(
                Kxp1,0.0,0.0,
                0.0,Kxp2,0.0,
                0.0,0.0,Kxp3);

    Matrix3f Kxp_dot(
                Kxp_dot_1,0.0,0.0,
                0.0,Kxp_dot_2,0.0,
                0.0,0.0,Kxp_dot_3);

    Vector3f e_xp(0.0,0.0,0.0);
    e_xp = xp - xpd;

    Vector3f e_xp_dot(0.0,0.0,0.0);
    e_xp_dot = xp_dot - xpd_dot;

    Vector3f xpd_dot_dot(0.0,0.0,0.0);

    Vector3f ge3(0,0,9.81);

    Vector3f FD;
    FD = (-Matrix_vector_mul(Kxp,e_xp) - Matrix_vector_mul(Kxp_dot,e_xp_dot) + xpd_dot_dot + ge3);
    FD[0] = mp*FD[0];
    FD[1] = mp*FD[1];
    FD[2] = mp*FD[2];

    return FD;
    
}

Vector3f ModeStabilize::e_R(Matrix3f R, Matrix3f Rd){

    Vector3f error_vec(vee_map(matrix_transpose(Rd)*R - matrix_transpose(R)*Rd));
    error_vec[0] = 0.5*error_vec[0];
    error_vec[1] = 0.5*error_vec[1];
    error_vec[2] = 0.5*error_vec[2];

    return error_vec;

}

Matrix3f ModeStabilize::hatmap(Vector3f v){
    Matrix3f R (
               0,    -v[2],      v[1],
               v[2],    0,     -v[0],
               -v[1],      v[0],       0);
    return R;
}

Vector3f ModeStabilize::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}

Matrix3f ModeStabilize::matrix_transpose(Matrix3f R){
    
    Matrix3f R_T(
            R[0][0],R[1][0],R[2][0],
            R[0][1],R[1][1],R[2][1],
            R[0][2],R[1][2],R[2][2]
            );
    return R_T;
}

Vector3f ModeStabilize::e_Omega(Matrix3f R, Matrix3f Rd, Vector3f Omega, Vector3f Omegad){

    Vector3f error_vec(Omega - (matrix_transpose(R)*Rd)*Omegad);
    return error_vec;

}

Vector3f ModeStabilize::sat_e_I(Vector3f vec){
    float sat_lim = 0.5;
    for (int ii=0; ii<3; ii++){
        if (vec[ii] > sat_lim){
            vec[ii] = sat_lim;
        }
        if (vec[ii] < -sat_lim){
            vec[ii] = -sat_lim;
        }
    }
    return vec;
}

Vector3f ModeStabilize::vee_map(Matrix3f R){

    Vector3f vector(R[2][1],R[0][2],R[1][0]);
    return vector;
}

Matrix3f ModeStabilize::eulerAnglesToRotationMatrix(Vector3f rpy){
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


void ModeStabilize::pilot_input(){

    H_roll      = (double)(channel_roll->get_control_in())/100.0;
    H_roll_dot  = (H_roll - H_roll_prev)/400.0;
    H_roll_prev = H_roll;

    H_pitch     = -(double)(channel_pitch->get_control_in())/100.0;
    H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
    H_pitch_prev= H_pitch;

    H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;
    H_throttle  =  (double)channel_throttle->get_control_in()-500.0;

    float dt_yaw = 1.0/250.0;
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

    // //// To inactivate XY controller
    // // For y direction
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

    // ////  for safe landing
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
    // ////////

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

