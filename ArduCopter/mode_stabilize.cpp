
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

int code_starting_flag = 0;
float arm_length = 0.161;

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

// Variables for geometric controller
Matrix3f R_log(1.0,0.0,0.0,
               0.0,1.0,0.0,
               0.0,0.0,1.0);
Matrix3f Rd_log(1.0,0.0,0.0,
               0.0,1.0,0.0,
               0.0,0.0,1.0);
Vector3f e_R_log;
Vector3f e_Omega_log;

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

// CAC commands
float CAC_roll      = 0.0;
float CAC_pitch     = 0.0;

float Des_roll      = 0.0;
float Des_pitch     = 0.0;
float Des_roll_dot  = 0.0;
float Des_pitch_dot = 0.0;

float rate_counter_time = 0.0;
float t_start_else_loop_end = 0.0;
int rate_counter = 0;

// predefined trajectory
float des_traj_sinusoidal_start_time = 0.0;
int des_traj_sinusoidal_flag = 0;

// To filter the human data
int fil_iter = 20;
float fil_H_roll_array[]    = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float fil_H_pitch_array[]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float yaw_offsetted_IROS    = 0.0;
float yaw_offset_human      = 0.0;

int IROS_auto_z_flag = 0;
float IROS_auto_z_timer = 0.0;
float IROS_auto_z_time_flag = 0.0;

float constant_mg_IROS_offset = 0.0;
float constant_mg_IROS = 0.0;

void ModeStabilize::run()
{
        // hal.serial(2)->printf("%1d,%6.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%4d,%4d,%4d,%4d_",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw,H_yaw_rate,H_throttle,PWM1,PWM2,PWM3,PWM4);

        // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);

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

        rate_counter_time = (AP_HAL::millis() - t_start_else_loop_end)/1000.0;

        if (rate_counter_time > 1.0){
            // hal.console->printf("Rate -> %d\n",rate_counter);
            rate_counter = 0;
            t_start_else_loop_end = AP_HAL::millis();
        }else{
            rate_counter =  rate_counter + 1;
        }

    ///////////// Arming checks  /////////////
        if (copter.motors->armed()){
            arm_disarm_flag = 1;
        }else{
            arm_disarm_flag = 0;
        }

    ///////////// Data logging portenta  /////////////
        // data_logging_portenta();
    ///////////// Checking batter voltage  /////////////
        battery_check();
    ///////////// Taking pilot inputs  /////////////
        pilot_input();
    ///////////// getting states of quadcopter /////////////
        quad_states();
    ///////////// For system iedntification  /////////////
        // system_identification_main();
    ///////////// For attitude and altitude controller /////////////
        attitude_altitude_controller();

    }
}

// void ModeStabilize::data_logging_portenta(){
//     // hal.serial(2)->printf("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",arm_disarm_flag,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw);
// }

void ModeStabilize::battery_check(){
    battvolt=copter.battery_volt();
}

void ModeStabilize::attitude_altitude_controller(){
    if (RC_Channels::get_radio_in(CH_6) < 1200){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

        //// Initializing the states of the quadcopters
        quad_z_ini =  inertial_nav.get_position().z / 100.0;

        // if (copter.rangefinder_alt_ok()){
            // float abcd = copter.rangefinder_state.alt_cm_filt.get();
            // hal.console->printf("Altitude from TFMini -> %f\n",abcd);
        // }

        //////// For checkig attitude performance
        des_traj_sinusoidal_flag = 0;

        ///// For auto altitude mode
        IROS_auto_z_flag = 0;
        IROS_auto_z_time_flag = (AP_HAL::millis()/1000.0);

        // yaw_initially = (360.0-(ahrs.yaw_sensor)/ 100.0)*3.141/180.0;     // rad;
        yaw_initially = 0.0;
        H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

        yaw_offsetted_IROS = (360.0-(ahrs.yaw_sensor) / 100.0) - HH_IMU_yaw_feedback;
        yaw_offset_human   = 0.0;

        float quad_x_ini_inertial =  inertial_nav.get_position().x / 100.0;
        float quad_y_ini_inertial =  inertial_nav.get_position().y / 100.0;

        quad_x_ini =  cosf(yaw_initially)*quad_x_ini_inertial + sinf(yaw_initially)*quad_y_ini_inertial;
        quad_y_ini = -sinf(yaw_initially)*quad_x_ini_inertial + cosf(yaw_initially)*quad_y_ini_inertial;

        //// Initial the PWMs
        PWM1 = 1000;
        PWM2 = 1000;
        PWM3 = 1000;
        PWM4 = 1000;

        }
        else if (RC_Channels::get_radio_in(CH_6) > 1400 && RC_Channels::get_radio_in(CH_6) < 1600 ){
            if (copter.motors->armed()){
                // custom_PID_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, z_des ,0.0);
                // custom_geometric_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, z_des ,0.0);

                // hal.console->printf("M1->%f,M2->%f,M3->%f\n",e_R_log[0],e_R_log[1],e_R_log[2]);

                ///////////////////////// Code for IROS 2022 /////////////////////////

                    IROS_controller_code();
                ///////////////////////// Code for IROS 2022 /////////////////////////

            }
        }else if (RC_Channels::get_radio_in(CH_6) > 1600){                                                                              
            if(copter.motors->armed()){
                // custom_PID_position_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, z_des ,0.0);
                // CAC_PD_controller();
                
                ///////////////////////// Code for sending sinusoidal commands /////////////////////////

                // if (des_traj_sinusoidal_flag == 0){
                //     des_traj_sinusoidal_start_time = AP_HAL::millis()/1000.0;
                //     des_traj_sinusoidal_flag = 1;
                // }else
                // {
                //     float Time_period = 2.0;
                //     float traj_time = AP_HAL::millis()/1000.0 - des_traj_sinusoidal_start_time;
                //     H_roll = 20.0*sinf((2*PI/Time_period)*traj_time);
                // }
                // custom_geometric_controller(H_roll, H_pitch, H_yaw, H_roll_dot ,H_pitch_dot, 0.0, z_des ,0.0);

                ///////////////////////// Code for sending sinusoidal commands /////////////////////////

                ///////////////////////// Code for IROS 2022 /////////////////////////

                    IROS_controller_code();
                ///////////////////////// Code for IROS 2022 /////////////////////////


            }
        }
}

void ModeStabilize::IROS_controller_code(){

    IROS_auto_z_timer = (AP_HAL::millis()/1000.0) - IROS_auto_z_time_flag;

    // if (IROS_auto_z_timer < 5.0) {

    //     z_des = IROS_auto_z_timer*0.1;
    //     float e_z   = z_des - quad_z;
    //     float Kp_z        = 2.0;    // 2.0 (best)
    //     float Kd_z        = 2.0;    // 1.0 (best)

    //     F     =  11.0 + Kp_z * (e_z) + Kd_z * (0.0 - quad_z_dot);

    //     if (F > 14.0){
    //         F = 14.0;
    //     }

    //     if (F < 0.0){
    //         F =  0.0;
    //     }    

    //     float dt_yaw = 1.0/100.0;
    //     H_yaw = wrap_360(H_yaw + H_yaw_rate*dt_yaw);
    //     des_psi       = H_yaw;
    //     des_phi       = 0.0;
    //     des_theta     = 0.0;
    //     rpy.x = imu_roll*PI/180.0;
    //     rpy.y = imu_pitch*PI/180.0;
    //     rpy.z = imu_yaw*PI/180.0;
    //     Omega.x = 0.0;
    //     Omega.y = 0.0;
    //     Omega.z = 0.0;
    //     R = eulerAnglesToRotationMatrix(rpy);

    //     hal.console->printf("Time-> %6.2f, z_des -> %6.2f, z -> %6.2f \n", IROS_auto_z_timer, z_des, quad_z);

    // }else
    // {
        // hal.console->printf("Time-> %6.2f, z_des -> %6.2f, z -> %6.2f \n", IROS_auto_z_timer, z_des, quad_z);

        ///////// Gathering the states of the system /////////
        Vector3f rpy(imu_roll*PI/180.0,imu_pitch*PI/180.0,imu_yaw*PI/180.0);
        Matrix3f R(eulerAnglesToRotationMatrix(rpy));
        Vector3f Omega(imu_roll_dot*PI/180.0,imu_pitch_dot*PI/180.0,imu_yaw_dot*PI/180.0);
        Vector3f e_3_neg(0,0,-1);

        qpd[0] = cosf(HH_IMU_yaw_feedback*PI/180);
        qpd[1] = sinf(HH_IMU_yaw_feedback*PI/180);
        qpd[2] = 0.0;

        Vector3f qcd(0.0,0.0,-1.0);

        Vector3f qpd_HH_frame(1.0,0.0,0.0);
        // Vector3f delta_qp = Matrix_vector_mul(hatmap(qpd_HH_frame),qp);

        // Vector3f delta_qp = -Matrix_vector_mul(hatmap(qpd),qp);
        // Vector3f delta_qc = Matrix_vector_mul(hatmap(qcd),qc);
        // Vector3f delta_omega_p = Omega_p;
        // Vector3f delta_omega_c = Omega_c;

        // hal.console->printf("(%6.2f, %6.2f, %6.2f), (%6.2f,%6.2f,%6.2f)\n",delta_qp[0],delta_qp[1],delta_qp[2],delta_omega_p[0],delta_omega_p[1],delta_omega_p[2]);
        // hal.console->printf("%10.3f, %10.3f, %10.3f\n",delta_qp[0],delta_qp[1],delta_qp[2]);

        float Kqp3      = 8.5;             // 9.5
        float Kqp3_dot  = 60.0;             // 60.0

        float dt_mg = 1.0/2500.0;
        float H_pitch_mg  = (double)(channel_pitch->get_control_in()/100.0);

        if (H_pitch_mg < 5.0 && H_pitch_mg > -5.0){
            H_pitch_mg = 0.0;
        }

        constant_mg_IROS_offset = constant_mg_IROS_offset + (H_pitch_mg * dt_mg);

        if (constant_mg_IROS_offset > 4.0){
            constant_mg_IROS_offset = 4.0;
        }

        if (constant_mg_IROS_offset < -4.0){
            constant_mg_IROS_offset = -4.0;
        }

        constant_mg_IROS = 15.5 + constant_mg_IROS_offset;

        hal.console->printf("%5.2f,%5.2f\n",13.4, constant_mg_IROS);

        float delta_u3 = constant_mg_IROS + Kqp3 * (-qp[2]) + Kqp3_dot * (-qp3_dot_fil_final);
        F = delta_u3;

        // hal.console->p1793rintf("%5.3f, %5.3f\n",-delta_qp[1],-qp3_dot_fil_final*10);

        // F = 10.0;
        if (F > 20.0){
            F = 20.0;
        }

        if (F < 0.0){
            F =  0.0;
        }

        ///////////////////// Planning for yaw controller ///////////////////// 
        // hal.console->printf("(%10.3f, %10.3f)\n",imu_yaw,HH_yaw_feedback);
        ///////////////////// Planning for yaw controller /////////////////////

        ///////////////////// Planning for roll and pitch controller ///////////////////// 

        float Kp_pitch_IROS  = 8.0;
        float Kd_pitch_IROS  = 24.0;
        float des_pitch_IROS = Kp_pitch_IROS * qc[0] + Kd_pitch_IROS * qc1_dot_fil_final;

        float Kp_roll_IROS_cable  = 18.0;
        float Kd_roll_IROS_cable  = 70.0;
        float des_roll_IROS_cable = -(Kp_roll_IROS_cable * qc[1] + Kd_roll_IROS_cable * qc2_dot_fil_final);
        // des_roll_IROS_cable = 0.0;
        float Kp_roll_IROS_payload  = 35.0; // 20.0
        float Kd_roll_IROS_payload  = 70.0;

        float des_roll_IROS_payload = Kp_roll_IROS_payload * (qp[1]) + Kd_roll_IROS_payload * (qp2_dot_fil_final);
        // des_roll_IROS_payload = 0.0;
        float des_roll_IROS = des_roll_IROS_cable + des_roll_IROS_payload;

        // hal.console->printf("%5.2f,%5.2f,%5.2f,%5.2f\n",qp[0],qp[1],qp[2],delta_u3);

        ///////////////////// Planning for roll and pitch controller ///////////////////// 

        ///////////////////// geometric attitude controller /////////////////////

        // Requied states for the attitude controller //

        float des_phi       = des_roll_IROS;
        float des_theta     = des_pitch_IROS;
        float human_handle_yaw_modified_yaw_control = HH_IMU_yaw_feedback + yaw_offsetted_IROS;
        float dt_yaw = 1.0/100.0;
        H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;

        if (H_yaw_rate < 5.0 && H_yaw_rate > -5.0){
            H_yaw_rate = 0.0;
        }

        yaw_offset_human  = yaw_offset_human + H_yaw_rate*dt_yaw;
        float des_psi       = wrap_360(human_handle_yaw_modified_yaw_control + yaw_offset_human);
        
        // R = eulerAnglesToRotationMatrix(rpy);

        // hal.console->printf("%5.2f,%5.2f,%5.2f,%5.2f\n",imu_yaw,HH_IMU_yaw_feedback,yaw_offsetted_IROS,des_psi);

        float des_phi_offset    = 0.0;
        float des_theta_offset  = 2.0;

        des_phi     = des_phi + des_phi_offset;
        des_theta   = des_theta + des_theta_offset;

    // }
    float des_phi_dot   = 0.0;
    float des_theta_dot = 0.0;
    float des_psi_dot   = 0.0;

    Vector3f rpy_d(des_phi*PI/180.0,des_theta*PI/180.0,des_psi*PI/180.0);
    Vector3f Omegad(des_phi_dot*PI/180.0,des_theta_dot*PI/180.0,des_psi_dot*PI/180.0);

    Matrix3f Rd(eulerAnglesToRotationMatrix(rpy_d));

    float c2 = 2.0;
    // float c1 = 1.0;

    Vector3f e_R_val        = e_R(R,Rd);
    Vector3f e_Omega_val    = e_Omega(R,Rd,Omega,Omegad);
    Vector3f e_I_val        = e_Omega_val + Vector3f(e_R_val[0]*c2,e_R_val[1]*c2,e_R_val[2]*c2);
    Vector3f e_I_val_sum    = sat_e_I(e_I_val_old + e_I_val);
    e_I_val_old             = e_I_val;

    R_log   = R;
    Rd_log  = Rd;

    e_R_log = e_R_val;

    KR1         = 0.9;  // 0.6 (TB best)  //0.4
    KOmega1     = 21; // 10.5(TB best)  //0.5
    KI1         = 0.01;  // 0.1 (TB best)  //0.1
    KR2         = 1.3;  // 1.0  (TB good)
    KOmega2     = 24; // 13.5 (TB good)
    KI2         = 0.01;  // 0.1  (TB good)
    KR3         = 4.0;  // 1.0  (TB good)
    KOmega3     = 10.0; // 13.5 (TB good)
    KI3         = 0.00;  // 0.1  (TB good)

    // hal.console->printf("%f,%f,%f\n",KR1,KOmega1,KI1);
    // hal.console->printf("%f,%f,%f\n",Mb1,Mb2,Mb3);

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
    // Vector3f M(e_R(R,Rd));
    // Vector3f M( Matrix_vector_mul(KR,e_R(R,Rd)));
    // Vector3f M(Matrix_vector_mul(KOmega,e_Omega(R,Rd,Omega,Omegad)));

    Mb1 = -M[0];
    Mb2 = -M[1];
    Mb3 =  M[2];
    // Mb3 = 0.0;

    float FM_devided_FF;
    if (battvolt >= 11.5){
        FM_devided_FF = 0.24;
    }else{ 
        FM_devided_FF = 0.31;
    }

    float function_F1 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

    if (HH_on_off_feedback < 1){
        PWM1 = 1000;
        PWM2 = 1000;
        PWM3 = 1000;
        PWM4 = 1000;
        // PWM1 = 1170;
        // PWM2 = 1170;
        // PWM3 = 1170;
        // PWM4 = 1170;
    }

}

void ModeStabilize::CAC_PD_controller(){

}

void ModeStabilize::custom_geometric_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

    // float mass = 1.236;

///////////////////// Altitude controller /////////////////////

    float e_z   = z_des - quad_z;

    float Kp_z        = 2.0;    // 2.0 (best)
    float Kd_z        = 2.0;    // 1.0 (best)
    // F     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    // F     =  10.0 + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    F     =  11.0 + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
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

    R_log   = R;
    Rd_log  = Rd;

    e_R_log = e_R_val;

/////////////////////// Manual gain tuning  ///////////////////////

    // KR1         =   1.3;     // 0.9
    // KOmega1     = 15.00;     // 10.00
    // KI1         =  0.00;     // 0.01
    // KR2         =   1.3;     // 1.3
    // KOmega2     =  15.0;     // 24.5
    // KI2         =   0.0;     // 0.01
    // KR3         =  6.00;     // 6.0
    // KOmega3     =  10.00;     // 10.0
    // KI3         =  0.00;     // 0.0

    KR1         = 0.9;  // 0.6 (TB best)  //0.4
    KOmega1     = 21; // 10.5(TB best)  //0.5
    KI1         = 0.01;  // 0.1 (TB best)  //0.1
    KR2         = 1.3;  // 1.0  (TB good)
    KOmega2     = 24; // 13.5 (TB good)
    KI2         = 0.01;  // 0.1  (TB good)
    KR3         = 6.0;  // 1.0  (TB good)
    KOmega3     = 10.0; // 13.5 (TB good)
    KI3         = 0.00;  // 0.1  (TB good)

    // hal.console->printf("%f,%f,%f\n",KR1,KOmega1,KI1);
    // hal.console->printf("%f,%f,%f\n",Mb1,Mb2,Mb3);

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
    // Vector3f M(e_R(R,Rd));
    // Vector3f M( Matrix_vector_mul(KR,e_R(R,Rd)));
    // Vector3f M(Matrix_vector_mul(KOmega,e_Omega(R,Rd,Omega,Omegad)));

    Mb1 = -M[0];
    Mb2 = -M[1];
    Mb3 =  M[2];

    // hal.console->printf("%f,%f,%f\n",Mb1,Mb2,Mb3);

    // Mb2 = 0.0;
    // Mb3 = 0.0;

    // e_R_log     = M;
    // e_Omega_log = e_Omega(R,Rd,Omega,Omegad);

    // F = 10.0;

    float FM_devided_FF ;
    if (battvolt >= 11.5 ){
         FM_devided_FF = 0.24; //0.24
    }else{
         FM_devided_FF = 0.31;
    }

    float function_F1 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);
    // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);

}

Vector3f ModeStabilize::e_R(Matrix3f R, Matrix3f Rd){
    // Rd.transpose()*R;
    // Vector3f error_vec(vee_map(Rd.transpose()*R - R.transpose()*Rd));
    Vector3f error_vec(vee_map(matrix_transpose(Rd)*R - matrix_transpose(R)*Rd));
    // matrix_transpose()

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
        // LOGGING_ENABLED == ENABLED;
        copter.init_rc_out();
        SRV_Channels::cork();
        hal.rcout->write(0,PWM1);
        hal.rcout->write(1,PWM2);
        hal.rcout->write(2,PWM3);
        hal.rcout->write(3,PWM4);
        SRV_Channels::push();
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

    // if (quad_x < -9.9){quad_x = -9.9;}
    // if (quad_x > 9.9){quad_x = 9.9;}

    // if (quad_y < -9.9){quad_y = -9.9;}
    // if (quad_y > 9.9){quad_y = 9.9;}

    // if (quad_z < 0){quad_z = 0;}
    // if (quad_z > 5){quad_z = 5;}

    // linear velocity in inertial frame of reference
    float quad_x_dot_inertial =  inertial_nav.get_velocity().x /100.0;
    float quad_y_dot_inertial =  inertial_nav.get_velocity().y /100.0;
    quad_z_dot                =  inertial_nav.get_velocity().z /100.0;

    // linear velocity in body reference frame
    quad_x_dot =  (cosf(yaw_initially)*quad_x_dot_inertial + sinf(yaw_initially)*quad_y_dot_inertial);
    quad_y_dot = (-sinf(yaw_initially)*quad_x_dot_inertial + cosf(yaw_initially)*quad_y_dot_inertial);

    imu_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw         =  360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    imu_roll_dot    =  (ahrs.get_gyro().x);             // degrees/second
    imu_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
    imu_yaw_dot     = -(ahrs.get_gyro().z);             // degrees/second

}

void ModeStabilize::pilot_input(){

    H_roll      = (double)(channel_roll->get_control_in())/100.0;
    // To filter the pilot data for roll angle
    for (int k = 1; k > (fil_iter+1) ; k++){
        if (k == 1){
            fil_H_roll_array[k] =  H_roll;
        }else{
            fil_H_roll_array[k] =  fil_H_roll_array[k-1];
        }
    }
    H_roll_dot  = (H_roll - H_roll_prev)/400.0;
    H_roll_prev = H_roll;

    H_pitch     = (double)(channel_pitch->get_control_in())/100.0;
    // To filter the pilot data for roll angle
    for (int h = 1; h > (fil_iter+1) ; h++){
        if (h == 1){
            fil_H_pitch_array[h] =  H_pitch;
        }else{
            fil_H_pitch_array[h] =  fil_H_pitch_array[h-1];
        }
    }
    H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
    H_pitch_prev= H_pitch;

    // // To filter the pilot data for pitch angle
    // for (int k =1; k++; k < fil_iter ){
    //     float fil_data_phi =  ;
    // }

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
    if (PWM < 1200){PWM = 1200;}

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