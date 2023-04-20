
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

////// System parameters
float arm_length            = 0.161;

////// Used flag variables
int code_starting_flag      = 0;
int yaw_flag_start          = 0;
int arm_disarm_flag         = 0;

////// Time parameters
float current_time          = 0.0;
float rate_counter_time     = 0.0;
float t_start_else_loop_end = 0.0;
int rate_counter            = 0;
float frequency_of_the_code = 400.0;

////// Quadcopter state variables
float quad_x        = 0.0;
float quad_y        = 0.0;
float quad_z        = 0.0;
float quad_x_dot    = 0.0;
float quad_y_dot    = 0.0;
float quad_z_dot    = 0.0;
float imu_roll      = 0.0;
float imu_pitch     = 0.0;
float imu_yaw       = 0.0;
float imu_roll_dot  = 0.0;
float imu_pitch_dot = 0.0;
float imu_yaw_dot   = 0.0;
Matrix3f R_log(1.0,0.0,0.0,
               0.0,1.0,0.0,
               0.0,0.0,1.0);

////// Desired state of the system
float x_des  = 0.0;
float y_des  = 0.0;
float z_des  = 0.0;
float yaw_des = 0.0;
float x_des_dot  = 0.0;
float y_des_dot  = 0.0;
float z_des_dot  = 0.0;
float yaw_des_dot = 0.0;

Matrix3f Rd_log(1.0,0.0,0.0,
               0.0,1.0,0.0,
               0.0,0.0,1.0);
float Des_roll      = 0.0;
float Des_pitch     = 0.0;
float Des_roll_dot  = 0.0;
float Des_pitch_dot = 0.0;


////// Errors in states
Vector3f e_R_log;
Vector3f e_Omega_log;
Vector3f e_I_val_old (0.0,0.0,0.0);

////// Initial state of the system
float quad_x_ini = 0.0;
float quad_y_ini = 0.0;
float quad_z_ini = 0.0;
float imu_yaw_ini = 0.0;

////// Human commands
float H_roll        = 0.0;
float H_pitch       = 0.0;
float H_yaw_rate    = 0.0;
float H_throttle    = 0.0;

float H_yaw         = 0.0;

float H_roll_dot    = 0.0;
float H_pitch_dot   = 0.0;
float H_pitch_prev  = 0.0;
float H_roll_prev   = 0.0;

////// Battery variables
float battvolt      = 0.0;

////// Initialization of gains values

float KR1           = 0.0;
float KOmega1       = 0.0;
float KI1           = 0.0;

float KR2           = 0.0;
float KOmega2       = 0.0;
float KI2           = 0.0;

float KR3           = 0.0;
float KOmega3       = 0.0;
float KI3           = 0.0;

//////// Savitzky–Golay filter global variables

// float y_minus_2_sq = 0.0;
// float y_minus_1_sq = 0.0;
// float y_current_sq = 0.0;
// float y_plus_1_sq  = 0.0;
// float y_plus_2_sq  = 0.0;

float x_des_dot_SQ_current    = 0.0;
float x_des_dot_SQ_minus_1    = 0.0;
float x_des_dot_SQ_minus_2    = 0.0;
float x_des_dot_SQ_plus_1     = 0.0;
float x_des_dot_SQ_plus_2     = 0.0;
float x_des_dot_SQ_filter     = 0.0;

//////// First order low pass filter

// Define filter parameters
// const float dt = 0.0025; // Sample time (50Hz)
// const float RC = 1/(2*M_PI*20); // Cut-off frequency (20Hz)

// Define filter variables
float x_des_dot_filteredValue   = 0;
float x_des_dot_previousValue   = 0;
float y_des_dot_filteredValue   =  0;
float y_des_dot_previousValue   = 0;
float z_des_dot_filteredValue   = 0;
float z_des_dot_previousValue   = 0;
float yaw_des_dot_filteredValue = 0.0;
float yaw_des_dot_previousValue = 0.0;

//////// Numerical intergration with Trapezoindal rule



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

    //////////// To count the frequency of the code
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

    ///////////// Checking batter voltage  /////////////
        battery_check();
    ///////////// Taking pilot inputs  /////////////
        pilot_input();
    ///////////// getting states of quadcopter /////////////
        quad_states();
    ///////////// For attitude and altitude controller /////////////
        attitude_altitude_controller();

    /////////////// Debuging to disarm the quadcopter ///////////////
    // bool AP_Arming_Copter::disarm(const AP_Arming::Method method, bool do_disarm_checks){
    //     return true;
    // }

    // if (RC_Channels::get_radio_in(CH_8) > 1500){
        
    // }

    }
}

void ModeStabilize::battery_check(){
    battvolt=copter.battery_volt();
}

void ModeStabilize::attitude_altitude_controller(){
    if (RC_Channels::get_radio_in(CH_6) < 1200){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

        //// Initializing the states of the quadcopters
        quad_z_ini =  inertial_nav.get_position().z / 100.0;

        // imu_yaw_ini = (360.0-(ahrs.yaw_sensor)/ 100.0)*3.141/180.0;     // rad;
        imu_yaw_ini = 0.0;
        H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

        float quad_x_ini_inertial =  inertial_nav.get_position().x / 100.0;
        float quad_y_ini_inertial =  inertial_nav.get_position().y / 100.0;

        quad_x_ini =  cosf(imu_yaw_ini)*quad_x_ini_inertial + sinf(imu_yaw_ini)*quad_y_ini_inertial;
        quad_y_ini = -sinf(imu_yaw_ini)*quad_x_ini_inertial + cosf(imu_yaw_ini)*quad_y_ini_inertial;

        //// Initial the PWMs
        PWM1 = 1000;
        PWM2 = 1000;
        PWM3 = 1000;
        PWM4 = 1000;

        }else if (RC_Channels::get_radio_in(CH_5) < 1200){
            if (copter.motors->armed()){
                    IROS_controller_code();
            }
        }else if (RC_Channels::get_radio_in(CH_5) > 1400 && RC_Channels::get_radio_in(CH_5) < 1600){
            if (copter.motors->armed()){
                    IROS_controller_code();
            }
        }else if (RC_Channels::get_radio_in(CH_5) > 1600 && RC_Channels::get_radio_in(CH_5) < 2000){
            if (copter.motors->armed()){
                    IROS_controller_code();
            }
        }
}


void ModeStabilize::FUNC_disarm(){
    copter.motors->armed(false);
}

void ModeStabilize::IROS_controller_code(){

        ///////// Gathering the states of the system /////////
        Vector3f rpy(imu_roll*PI/180.0,imu_pitch*PI/180.0,imu_yaw*PI/180.0);
        Matrix3f R(eulerAnglesToRotationMatrix(rpy));
        Vector3f Omega(imu_roll_dot*PI/180.0,imu_pitch_dot*PI/180.0,imu_yaw_dot*PI/180.0);

        F = 0.0;

        // F = 10.0;
        if (F > 20.0){
            F = 20.0;
        }

        if (F < 0.0){
            F =  0.0;
        }

        if (H_yaw_rate < 5.0 && H_yaw_rate > -5.0){
            H_yaw_rate = 0.0;
        }

    Vector3f rpy_d(0.0*PI/180.0,0.0*PI/180.0,0.0*PI/180.0);
    Vector3f Omegad(0.0*PI/180.0,0.0*PI/180.0,0.0*PI/180.0);

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

void ModeStabilize::custom_geometric_controller(float des_phi, float des_theta, float des_psi,float des_phi_dot, float des_theta_dot, float des_psi_dot, float des_z, float des_z_dot){

    // float mass = 1.236;

///////////////////// Altitude controller /////////////////////

    float e_z   = z_des - quad_z;

    float Kp_z        = 2.0;    // 2.0 (best)
    float Kd_z        = 2.0;    // 1.0 (best)
    // F     =  mass * GRAVITY_MSS + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    // F     =  10.0 + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);
    F     =  11.0 + Kp_z * (e_z) + Kd_z * (des_z_dot - quad_z_dot);

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

void ModeStabilize::quad_states(){
    // Position in inertial reference frame
    float quad_x_inertial =  inertial_nav.get_position().x / 100.0;
    float quad_y_inertial =  inertial_nav.get_position().y / 100.0;

    // position in body reference frame
    quad_x =  (cosf(imu_yaw_ini)*quad_x_inertial + sinf(imu_yaw_ini)*quad_y_inertial) - quad_x_ini;
    quad_y = (-sinf(imu_yaw_ini)*quad_x_inertial + cosf(imu_yaw_ini)*quad_y_inertial) - quad_y_ini;

    quad_y = -quad_y;

    quad_z =  (inertial_nav.get_position().z / 100.0) - quad_z_ini;

    // linear velocity in inertial frame of reference
    float quad_x_dot_inertial =  inertial_nav.get_velocity().x /100.0;
    float quad_y_dot_inertial =  inertial_nav.get_velocity().y /100.0;
    quad_z_dot                =  inertial_nav.get_velocity().z /100.0;

    // linear velocity in body reference frame
    quad_x_dot =  (cosf(imu_yaw_ini)*quad_x_dot_inertial + sinf(imu_yaw_ini)*quad_y_dot_inertial);
    quad_y_dot = (-sinf(imu_yaw_ini)*quad_x_dot_inertial + cosf(imu_yaw_ini)*quad_y_dot_inertial);

    imu_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw         =  360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    imu_roll_dot    =  (ahrs.get_gyro().x);             // degrees/second
    imu_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
    imu_yaw_dot     = -(ahrs.get_gyro().z);             // degrees/second

}

void ModeStabilize::pilot_input(){

    H_roll      = -(double)(channel_roll->get_control_in())/100.0;          // range: (-45 to 45)
    H_pitch     = -(double)(channel_pitch->get_control_in())/100.0;         // range: (-45 to 45)
    H_yaw_rate  = -(double)(channel_yaw->get_control_in())/100.0;           // range: (-45 to 45)
    H_throttle  =  (double)(channel_throttle->get_control_in())-500.0;      // range: (-500 to 500)

    //////// Common values 
    float max_des_velocity_horizontal   = 5.0;      // (m/s)
    float max_des_velocity_vertical     = 5.0;      // (m/s)
    float max_yaw_dot_                  = 25.0;     // (deg/s)
    float slop_for_horizontal_scaling   = 45.0 / max_des_velocity_horizontal;
    float slop_for_vertical_scaling     = 500.0 / max_des_velocity_vertical;
    float slop_for_yaw_scaling          = 45.0 / max_yaw_dot_;

    //////// Convert H_pitch into desired velocity in x
    // Trim (-2 to 2) data. This will help preventing the drone to drift
    if (H_pitch > -2.0 && H_pitch < 2.0){H_pitch = 0.0;}
    // Start the values considering -2 and 2 as the origin
    if (H_pitch < -2.0 && H_pitch > -45.0){H_pitch = H_pitch + 2.0;}
    if (H_pitch >  2.0 && H_pitch <  45.0){H_pitch = H_pitch - 2.0;}
    // Scale the (-45 to 45) values to desired velocity limits
    x_des_dot   = H_pitch/slop_for_horizontal_scaling;
    // First ordered low pass filter
    x_des_dot_filteredValue = 0.686 * x_des_dot_previousValue + 0.314 * x_des_dot;
    x_des_dot_previousValue = x_des_dot_filteredValue;
    x_des           = x_des +( x_des_dot_filteredValue + x_des_dot_previousValue) / 2 * 0.0025;
    ////////////////////////////////

    //////// Convert H_roll into desired velocity in y
    // Trim (-2 to 2) data. This will help preventing the drone to drift
    if (H_roll > -2.0 && H_roll < 2.0){H_roll = 0.0;}
    // Start the values considering -2 and 2 as the origin
    if (H_roll < -2.0 && H_roll > -45.0){H_roll = H_roll + 2.0;}
    if (H_roll >  2.0 && H_roll <  45.0){H_roll = H_roll - 2.0;}
    // Scale the (-45 to 45) values to desired velocity limits
    y_des_dot   = H_roll/slop_for_horizontal_scaling;
    // First ordered low pass filter
    y_des_dot_filteredValue = 0.686 * y_des_dot_previousValue + 0.314 * y_des_dot;
    y_des_dot_previousValue = y_des_dot_filteredValue;
    y_des           = y_des +( y_des_dot_filteredValue + y_des_dot_previousValue) / 2 * 0.0025;
    ////////////////////////////////

    //////// Convert H_throttle into desired velocity in z
    // Trim (-20 to 20) data. This will help preventing the drone to drift
    if (H_throttle > -20.0 && H_throttle < 20.0){H_throttle = 0.0;}
    // Start the values considering -2 and 2 as the origin
    if (H_throttle < -20.0 && H_throttle > -500.0){H_throttle = H_throttle + 20.0;}
    if (H_throttle >  20.0 && H_throttle <  500.0){H_throttle = H_throttle - 20.0;}
    // Scale the (-500 to 500) values to desired velocity limits
    z_des_dot   = H_throttle/slop_for_vertical_scaling;
    // First ordered low pass filter
    z_des_dot_filteredValue = 0.686 * z_des_dot_previousValue + 0.314 * z_des_dot;
    z_des_dot_previousValue = z_des_dot_filteredValue;
    z_des           = z_des +( z_des_dot_filteredValue + z_des_dot_previousValue) / 2 * 0.0025;
    ////////////////////////////////

    //////// Convert H_yaw_rate into desired yaw rate
    // Trim (-2 to 2) data. This will help preventing the drone to drift
    if (H_yaw_rate > -2.0 && H_yaw_rate < 2.0){H_yaw_rate = 0.0;}
    // Start the values considering -2 and 2 as the origin
    if (H_yaw_rate < -2.0 && H_yaw_rate > -45.0){H_yaw_rate = H_yaw_rate + 2.0;}
    if (H_yaw_rate >  2.0 && H_yaw_rate <  45.0){H_yaw_rate = H_yaw_rate - 2.0;}
    // Scale the (-45 to 45) values to desired velocity limits
    yaw_des_dot   = H_yaw_rate/slop_for_yaw_scaling;
    // First ordered low pass filter
    yaw_des_dot_filteredValue = 0.686 * yaw_des_dot_previousValue + 0.314 * yaw_des_dot;
    yaw_des_dot_previousValue = yaw_des_dot_filteredValue;
    yaw_des           = yaw_des +( yaw_des_dot_filteredValue + yaw_des_dot_previousValue) / 2 * 0.0025;
    ////////////////////////////////

    //////// To debug the code
    // hal.console->printf("%f,%f,%f,%f\n",x_des_dot,y_des_dot,z_des_dot,yaw_des_dot);
    hal.console->printf("%f,%f,%f,%f\n",x_des,y_des,z_des,yaw_des);

}

float ModeStabilize::SQ_filter_fifth_order(float y_minus_2, float y_minus_1, float y, float y_plus_1, float y_plus_2){

    float SQ_fil_data = (1.0/35.0)*(-3.0 * y_minus_2 + 12.0 * y_minus_1 + 17.0 * y + 12.0 * y_plus_1 - 3.0 * y_plus_2);
    return SQ_fil_data;
}

int ModeStabilize::Inverse_thrust_function(float Force){
    int PWM = 1200;
    if (battvolt >= 11.5 ){PWM = 1000 * (0.9206 + (sqrtf(12.8953 + 30.3264*Force)/(15.1632)));
    }else{PWM = 1000 * (0.6021 + (sqrtf(33.2341 + 19.418*Force)/(9.5740)));}
    if (PWM > 2000){PWM = 2000;}
    if (PWM < 1200){PWM = 1200;}
    return PWM;
}