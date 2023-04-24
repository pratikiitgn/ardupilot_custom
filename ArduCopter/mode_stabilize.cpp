
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

extern const AP_HAL::HAL &hal;

#define ESC_HZ 490
#define PI 3.14

////// System parameters
const float arm_length = 0.161;
const float mass_quad = 1.236;
const float gravity_acc = 9.81;

// Intertia matrix
const Matrix3f JJ(
    0.0113, 0.0, 0.0,
    0.0, 0.0133, 0.0,
    0.0, 0.0, 0.0187);

////// Used flag variables
int code_starting_flag = 0;
int yaw_flag_start = 0;

////// Time parameters
float current_time = 0.0;
float rate_counter_time = 0.0;
float t_start_else_loop_end = 0.0;
int rate_counter = 0;
float frequency_of_the_code = 400.0;

////// System state variables
float quad_x = 0.0;
float quad_y = 0.0;
float quad_z = 0.0;
float quad_x_dot = 0.0;
float quad_y_dot = 0.0;
float quad_z_dot = 0.0;

Vector3f rpy(0.0, 0.0, 0.0);
Matrix3f R(1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0);
Vector3f Omega(0.0, 0.0, 0.0);

Vector3f qc(0.0, 0.0, 1.0);
Vector3f Omega_c(0.0, 0.0, 0.0);

////// Desired state of the system
float x_des = 0.0;
float y_des = 0.0;
float z_des = 0.0;
float yaw_des = 0.0;
float x_des_dot = 0.0;
float y_des_dot = 0.0;
float z_des_dot = 0.0;
float yaw_des_dot = 0.0;

Vector3f rpyd(0.0, 0.0, 0.0);
Matrix3f Rd(1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);
Vector3f Omegad(0.0, 0.0, 0.0);

////// Errors in states
Vector3f e_I_val_old(0.0, 0.0, 0.0);

////// Initial state of the system
float quad_x_ini = 0.0;
float quad_y_ini = 0.0;
float quad_z_ini = 0.0;
float imu_yaw_ini = 0.0;

////// Human commands
float H_roll = 0.0;
float H_pitch = 0.0;
float H_yaw_rate = 0.0;
float H_throttle = 0.0;

float H_yaw = 0.0;

float H_roll_dot = 0.0;
float H_pitch_dot = 0.0;
float H_pitch_prev = 0.0;
float H_roll_prev = 0.0;

////// Battery variables
float battvolt = 0.0;

////// Initialization of gains values

const float KR1 = 0.6;     // 0.6 (lab)
const float KOmega1 = 0.6; // 0.8 (lab)

const float KR2 = 0.6;     // 0.9 (lab)
const float KOmega2 = 0.6; // 0.8 (lab)

const float KR3 = 5.0;     // 5.0  (lab)
const float KOmega3 = 1.0; // 1.0 (lab)

Matrix3f KR(
    KR1, 0.0, 0.0,
    0.0, KR2, 0.0,
    0.0, 0.0, KR3);
Matrix3f KOmega(
    KOmega1, 0.0, 0.0,
    0.0, KOmega2, 0.0,
    0.0, 0.0, KOmega3);

const float Kp_x = 4.0; // 2.0 (best)
const float Kd_x = 2.0; // 1.0 (best)

const float Kp_y = 4.0; // 2.0 (best)
const float Kd_y = 2.0; // 1.0 (best)

const float Kp_z = 4.0; // 2.0 (best)
const float Kd_z = 2.0; // 1.0 (best)

Matrix3f Kxq(
    Kp_x, 0.0, 0.0,
    0.0, Kp_y, 0.0,
    0.0, 0.0, Kp_z);

Matrix3f Kxq_dot(
    Kd_x, 0.0, 0.0,
    0.0, Kd_y, 0.0,
    0.0, 0.0, Kd_z);

//////// First order low pass filter
// Define filter parameters
// const float dt = 0.0025; // Sample time (50Hz)
// const float RC = 1/(2*M_PI*20); // Cut-off frequency (20Hz)
// Define filter variables

float x_des_dot_previousValue = 0.0;
float y_des_dot_previousValue = 0.0;
float z_des_dot_previousValue = 0.0;
float yaw_des_dot_previousValue = 0.0;
float H_roll_previousValue = 0.0;
float H_pitch_previousValue = 0.0;
float H_throttle_previousValue = 0.0;
float H_yaw_rate_previousValue = 0.0;
float encoder_roll_feedback_previousValue = 0.0;
float encoder_pitch_feedback_previousValue = 0.0;
float qc_dot_1_previousValue = 0.0;
float qc_dot_2_previousValue = 0.0;
float qc_dot_3_previousValue = 0.0;
float quad_z_previousValue = 0.0;

//////// Calculations for second order filtering
// Define filter parameters
const float dt = 1.0 / 400.0; // Sample time (400Hz)
const float fc = 40;          // Cut-off frequency (40Hz)
const float damping = 0.7;    // Damping ratio

// Define filter variables
float filteredValue = 0;
float previousValue = 0;
float previousFilteredValue = 0;

// Calculate filter coefficients
const float w0 = 2 * PI * fc;
const float alpha = sinf(w0) / (2 * damping);
const float beta = dt * w0 / 2;

///////// Calculation of Savitzky–Golay filter
/////////

// Define filter parameters
const int window_size = 7;                   // Window size (odd number)
const int poly_order = 2;                    // Polynomial order
const int half_size = (window_size - 1) / 2; // Half of window size

// Define filter variables
int filteredValue_SG = 0;
// Define Savitzky-Golay filter coefficients
float sg_coeffs[window_size] = {0.0357142857, 0.1071428571, 0.1785714286, 0.2142857143, 0.2142857143, 0.1785714286, 0.1071428571}; // Define data buffer
float data_buffer[window_size] = {0};
bool quad_z_changed = false;

//////// Calculation of cable attitude device
////////
Vector3f qc_prev(0.0, 0.0, -1.0);

//////// Applying simple moving average filter
const int WINDOW_SIZE_simple_moving_average = 20;
// Initialize array to store last 5 samples
float samples_simple_moving_average[WINDOW_SIZE_simple_moving_average] = {0};
int index_simple_moving_average = 0;


// //////// Applying weighted moving average filter
// const int WINDOW_SIZE_WeightedMovingAverage = 5;
// // Initialize array to store last 5 samples
// float samples_WeightedMovingAverage[WINDOW_SIZE_WeightedMovingAverage] = {0};
// int index_WeightedMovingAverage = 0;
// // Set weights for weighted moving average filter
// float weights[WINDOW_SIZE_WeightedMovingAverage] = {0.1, 0.2, 0.2, 0.4, 0.4};

void ModeStabilize::run()
{

    if (code_starting_flag == 0)
    {
        // To intialize the code
        copter.init_rc_out();
        hal.rcout->set_freq(15, ESC_HZ); // 0xFF  0x0F->b'00001111'
        hal.rcout->enable_ch(0);
        hal.rcout->enable_ch(1);
        hal.rcout->enable_ch(2);
        hal.rcout->enable_ch(3);
        code_starting_flag = 1;
    }
    else
    {
        //////////// To count the frequency of the code
        rate_counter_time = (AP_HAL::millis() - t_start_else_loop_end) / 1000.0;

        if (rate_counter_time > 1.0)
        {
            // hal.console->printf("Rate -> %d\n",rate_counter);
            rate_counter = 0;
            t_start_else_loop_end = AP_HAL::millis();
        }
        else
        {
            rate_counter = rate_counter + 1;
        }

        ///////////// Checking batter voltage  /////////////
        battery_check();

        ///////////// Initialize the system states until arming  /////////////
        if (copter.motors->armed())
        {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN); // AP_Motors class.cpp libraries
            if (RC_Channels::get_radio_in(CH_6) < 1500)
            {
                //// run the motors at 1150 PWM
                // PWM1 = 1100;
                // PWM2 = 1100;
                // PWM3 = 1100;
                // PWM4 = 1100;

                PWM1 = 1000;
                PWM2 = 1000;
                PWM3 = 1000;
                PWM4 = 1000;

                //// Reset the states
                x_des = 0.0;
                y_des = 0.0;
                z_des = 0.0;
                yaw_des = 360.0 - (ahrs.yaw_sensor) / 100.0; // degrees
                x_des_dot = 0.0;
                y_des_dot = 0.0;
                z_des_dot = 0.0;
                yaw_des_dot = 0.0;
                quad_x_ini = inertial_nav.get_position().x / 100.0;
                quad_y_ini = inertial_nav.get_position().y / 100.0;
                quad_z_ini = inertial_nav.get_position().z / 100.0;
                //// Safe landing code
                if (RC_Channels::get_radio_in(CH_8) > 1500)
                {
                    // copter.motors->armed(false);
                }
            }
            else if (RC_Channels::get_radio_in(CH_6) > 1500)
            {
                if (RC_Channels::get_radio_in(CH_5) < 1200)
                {
                    custom_Stabilize_mode();
                    if (RC_Channels::get_radio_in(CH_8) > 1500)
                    {
                        // copter.motors->armed(false);
                        //// run the motors at 1150 PWM
                        PWM1 = 1000;
                        PWM2 = 1000;
                        PWM3 = 1000;
                        PWM4 = 1000;
                    }
                }
                else if (RC_Channels::get_radio_in(CH_5) > 1300 && RC_Channels::get_radio_in(CH_5) < 1700)
                {
                    custom_Loiter_mode();
                    if (RC_Channels::get_radio_in(CH_8) > 1500)
                    {
                        // copter.motors->armed(false);
                        //// run the motors at 1150 PWM
                        PWM1 = 1000;
                        PWM2 = 1000;
                        PWM3 = 1000;
                        PWM4 = 1000;
                    }
                }
                else if (RC_Channels::get_radio_in(CH_5) > 1800)
                {
                    NL_SQCSP_mode();
                    if (RC_Channels::get_radio_in(CH_8) > 1500)
                    {
                        // copter.motors->armed(false);
                        //// run the motors at 1150 PWM
                        PWM1 = 1000;
                        PWM2 = 1000;
                        PWM3 = 1000;
                        PWM4 = 1000;
                    }
                }
            }
        }
        else
        {
            x_des = 0.0;
            y_des = 0.0;
            z_des = 0.0;
            // yaw_des = 360.0 - (ahrs.yaw_sensor) / 100.0; // degrees
            x_des_dot = 0.0;
            y_des_dot = 0.0;
            z_des_dot = 0.0;
            yaw_des_dot = 0.0;
            quad_x_ini = inertial_nav.get_position().x / 100.0;
            quad_y_ini = inertial_nav.get_position().y / 100.0;
            quad_z_ini = inertial_nav.get_position().z / 100.0;

            //// run the motors at 1150 PWM
            PWM1 = 1000;
            PWM2 = 1000;
            PWM3 = 1000;
            PWM4 = 1000;
        }
        ///////////// Taking pilot inputs  /////////////
        pilot_input();
        ///////////// getting states of quadcopter /////////////
        quad_states();
        ///////////// getting cable attitude from the CAM device /////////////
        get_CAM_device_data();
    }
}

void ModeStabilize::custom_Stabilize_mode()
{

    F = mass_quad * gravity_acc + Kp_z * (z_des - quad_z) + Kd_z * (quad_z_dot - z_des_dot);
    F = Thrust_saturation(F);
    // hal.console->printf("%3.4f,%3.4f,%3.4f\n", quad_z, quad_z_dot, F);

    Matrix3f Rd_temp(eulerAnglesToRotationMatrix(rpyd * PI / 180.0));
    Rd = Rd_temp;

    Vector3f M(Matrix_vector_mul(KR, e_R(R, Rd)) + Matrix_vector_mul(KOmega, e_Omega(R, Rd, Omega, Omegad)) + Omega % Matrix_vector_mul(JJ, Omega));
    Mb1 = -M[0];
    Mb2 = -M[1];
    Mb3 = M[2];

    final_F_M_calling();

    // Vector3f check_ = e_R(R, Rd);
    // Vector3f check__ = e_Omega(R, Rd, Omega, Omegad);
    // hal.console->printf("%3.4f,%3.4f\n", check_[0], check__[0]);
    // hal.console->printf("%3.2f,%3.2f,%3.2f,%3.2f\n", F,M[0],M[1],M[2]);
    // hal.console->printf("%3.4f,%3.4f,%3.4f\n",check_[0],check_[1],check_[2]);
    // hal.console->printf("%3.4f,%3.4f,%3.4f\n",check__[0],check__[1],check__[2]);
}

void ModeStabilize::custom_Loiter_mode()
{
    Vector3f e_3_with_gravity(0, 0, gravity_acc);
    Vector3f u = (Matrix_vector_mul(Kxq, e_X()) + Matrix_vector_mul(Kxq_dot, e_X_dot()) + e_3_with_gravity);
    u[0] = mass_quad * u[0];
    u[1] = mass_quad * u[1];
    u[2] = mass_quad * u[2];

    Vector3f b3d;
    b3d[0] = u[0] / norm_of_vec(u);
    b3d[1] = u[1] / norm_of_vec(u);
    b3d[2] = u[2] / norm_of_vec(u);

    Vector3f b1c(cosf(yaw_des * PI / 180.0), sinf(yaw_des * PI / 180.0), 0.0);
    Vector3f b2d;
    b2d = Matrix_vector_mul(hatmap(b3d), b1c);
    Vector3f b1d;
    b1d = Matrix_vector_mul(hatmap(b2d), b3d);

    Matrix3f Rd_temp_(b1d[0], b2d[0], b3d[0],
                      b1d[1], b2d[1], b3d[1],
                      b1d[2], b2d[2], b3d[2]);
    Rd = Rd_temp_;

    final_F_M_calling();
}

void ModeStabilize::NL_SQCSP_mode()
{
}

void ModeStabilize::get_CAM_device_data()
{

    //////// Apply low pass filter on the CAM device data
    // First ordered low pass filter
    encoder_roll_feedback = 0.686 * encoder_roll_feedback_previousValue + 0.314 * encoder_roll_feedback;
    encoder_roll_feedback_previousValue = encoder_roll_feedback;

    // First ordered low pass filter
    encoder_pitch_feedback = 0.686 * encoder_pitch_feedback_previousValue + 0.314 * encoder_pitch_feedback;
    encoder_pitch_feedback_previousValue = encoder_pitch_feedback;

    Vector3f e_3_neg(0, 0, -1);

    // Calculate rotation about pitch axis of CAM device
    Matrix3f CAM_R_y(
        cosf(encoder_pitch_feedback * PI / 180), 0, sinf(encoder_pitch_feedback * PI / 180),
        0, 1, 0,
        -sinf(encoder_pitch_feedback * PI / 180), 0, cosf(encoder_pitch_feedback * PI / 180));

    // Calculate rotation about roll axis of CAM device
    Matrix3f CAM_R_x(
        1, 0, 0,
        0, cosf(encoder_roll_feedback * PI / 180), -sinf(encoder_roll_feedback * PI / 180),
        0, sinf(encoder_roll_feedback * PI / 180), cosf(encoder_roll_feedback * PI / 180));

    qc = Matrix_vector_mul(R, Matrix_vector_mul(CAM_R_x, Matrix_vector_mul(CAM_R_y, e_3_neg)));
    Vector3f qc_dot = (qc - qc_prev) * frequency_of_the_code;

    // Omega_c = Matrix_vector_mul(hatmap(qc), qc_dot);
    qc_prev = qc;

    // First ordered low pass filter
    qc_dot[0] = 0.686 * qc_dot_1_previousValue + 0.314 * qc_dot[0];
    qc_dot_1_previousValue = qc_dot[0];

    // First ordered low pass filter
    qc_dot[1] = 0.686 * qc_dot_2_previousValue + 0.314 * qc_dot[1];
    qc_dot_2_previousValue = qc_dot[1];

    // First ordered low pass filter
    qc_dot[2] = 0.686 * qc_dot_3_previousValue + 0.314 * qc_dot[2];
    qc_dot_3_previousValue = qc_dot[2];

    // hal.console->printf("%0.3f,", imu_roll_log);
    // hal.console->printf("%0.3f,", encoder_roll_feedback_filtered);
    // hal.console->printf("%0.3f,", imu_pitch_log);
    // hal.console->printf("%0.3f\n", encoder_pitch_feedback_filtered);
    // hal.console->printf("%0.3f,%0.3f,%0.3f \n", qc_dot[0], qc_dot[1], qc_dot[2]);
}

float ModeStabilize::Thrust_saturation(float f_value)
{
    float max_thrust_value = 15.0;
    if (f_value > max_thrust_value)
    {
        f_value = max_thrust_value;
    }
    if (f_value < 0.0)
    {
        f_value = 0.0;
    }
    return f_value;
}

void ModeStabilize::battery_check()
{
    battvolt = copter.battery_volt();
}

void ModeStabilize::FUNC_disarm()
{
    copter.motors->armed(false);
}

Vector3f ModeStabilize::e_X()
{
    Vector3f err_pos_quad(x_des - quad_x, y_des - quad_y, z_des - quad_z);
    return err_pos_quad;
}

Vector3f ModeStabilize::e_X_dot()
{
    Vector3f err_vel_quad(quad_x_dot - x_des_dot, quad_y_dot - y_des_dot, quad_z_dot - z_des_dot);
    return err_vel_quad;
}

Vector3f ModeStabilize::e_R(Matrix3f R_quad, Matrix3f Rd_quad)
{
    Vector3f error_vec(vee_map(matrix_transpose(Rd_quad) * R_quad - matrix_transpose(R_quad) * Rd_quad));
    return error_vec;
}

Vector3f ModeStabilize::e_Omega(Matrix3f R_func, Matrix3f Rd_func, Vector3f Omega_func, Vector3f Omegad_func)
{
    Vector3f error_vec(Omega_func - (matrix_transpose(R_func) * Rd_func) * Omegad_func);
    return error_vec;
}
Matrix3f ModeStabilize::hatmap(Vector3f v)
{
    Matrix3f R_from_hatmap(
        0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0);
    return R_from_hatmap;
}

Vector3f ModeStabilize::vee_map(Matrix3f R_func)
{
    Vector3f vector(R_func[2][1], R_func[0][2], R_func[1][0]);
    return vector;
}

Vector3f ModeStabilize::Matrix_vector_mul(Matrix3f R_quad, Vector3f v_quad)
{
    Vector3f mul_vector(
        R_quad[0][0] * v_quad[0] + R_quad[0][1] * v_quad[1] + R_quad[0][2] * v_quad[2],
        R_quad[1][0] * v_quad[0] + R_quad[1][1] * v_quad[1] + R_quad[1][2] * v_quad[2],
        R_quad[2][0] * v_quad[0] + R_quad[2][1] * v_quad[1] + R_quad[2][2] * v_quad[2]);
    return mul_vector;
}

Matrix3f ModeStabilize::matrix_transpose(Matrix3f R_func)
{
    Matrix3f R_T(
        R_func[0][0], R_func[1][0], R_func[2][0],
        R_func[0][1], R_func[1][1], R_func[2][1],
        R_func[0][2], R_func[1][2], R_func[2][2]);
    return R_T;
}

float ModeStabilize::norm_of_vec(Vector3f vec_)
{
    float norm_of_vec__ = sqrtf(vec_[0] * vec_[0] + vec_[1] * vec_[1] + vec_[2] * vec_[2]);
    return norm_of_vec__;
}

Matrix3f ModeStabilize::eulerAnglesToRotationMatrix(Vector3f rpy_func)
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
    return R_z * R_y * R_x;
}

void ModeStabilize::quad_states()
{

    // Position in inertial reference frame
    quad_x = (inertial_nav.get_position().x / 100.0) - quad_x_ini; // m
    quad_y = (inertial_nav.get_position().y / 100.0) - quad_y_ini; // m

    // Linear velocity in inertial frame of reference
    quad_x_dot = inertial_nav.get_velocity().x / 100.0; // m/s
    quad_y_dot = inertial_nav.get_velocity().y / 100.0; // m/s

    ///////// Get the data from the TF mini plus
    // if (copter.rangefinder_alt_ok()){
    // quad_z = (copter.rangefinder_state.alt_cm) / 100.0;
    quad_z = (copter.rangefinder_state.alt_cm);
    // hal.console->printf("%3.3f,",quad_z);

    ///////// Applying first ordered low pass filter
    // quad_z = 0.686 * quad_z_previousValue + 0.314 * quad_z;
    // quad_z_previousValue = quad_z;

    /////// Applying moving average filter
    float smoothedSample_simple_moving_average = updateMovingAverage(quad_z);

    // //////// Applying weighted moving average filter
    // float smoothedSample_weighted_moving_average = updateWeightedMovingAverage(quad_z);

    //////// Taking time derivative of quad_z_vel
    // quad_z_dot = (quad_z - quad_z_previousValue) / dt;

    //////// To debug the code
    hal.console->printf("%3.3f,", quad_z);
    hal.console->printf("%3.3f\n", smoothedSample_simple_moving_average);
    // hal.console->printf("%3.3f\n", smoothedSample_weighted_moving_average);

    // Attitude of the quadcopter
    rpy[0] = (ahrs.roll_sensor) / 100.0;        // degrees
    rpy[1] = -(ahrs.pitch_sensor) / 100.0;      // degrees
    rpy[2] = 360.0 - (ahrs.yaw_sensor) / 100.0; // degrees
    Matrix3f R_temp(eulerAnglesToRotationMatrix(rpy * PI / 180.0));
    R = R_temp;

    // Angular velocity of the quadcopter
    Omega[0] = (ahrs.get_gyro().x);  // degrees/second
    Omega[1] = -(ahrs.get_gyro().y); // degrees/second
    Omega[2] = -(ahrs.get_gyro().z); // degrees/second

    // float abcd = RangeFinderState.alt_cm;
}

float ModeStabilize::updateMovingAverage(float newSample)
{
    // Add new sample to array and update index
    samples_simple_moving_average[index_simple_moving_average] = newSample;
    index_simple_moving_average = (index_simple_moving_average + 1) % WINDOW_SIZE_simple_moving_average;

    // Calculate sum of samples in window
    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE_simple_moving_average; i++)
    {
        sum += samples_simple_moving_average[i];
    }

    // Calculate and return moving average
    float average = sum / WINDOW_SIZE_simple_moving_average;

    // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f,%3.3f, and ,", samples_simple_moving_average[0],samples_simple_moving_average[1],samples_simple_moving_average[2],samples_simple_moving_average[3],samples_simple_moving_average[4]);

    return average;
}

// float ModeStabilize::updateWeightedMovingAverage(float newSample)
// {
//     // Add new sample to array and update index
//     samples_WeightedMovingAverage[index_WeightedMovingAverage] = newSample;
//     index_WeightedMovingAverage = (index_WeightedMovingAverage + 1) % WINDOW_SIZE_WeightedMovingAverage;

//     // Calculate sum of samples in window
//     float weightedSum = 0;
//     for (int i = 0; i < WINDOW_SIZE_WeightedMovingAverage; i++)
//     {
//         // weightedSum += weights[i] * samples_WeightedMovingAverage[(index_WeightedMovingAverage + i) % WINDOW_SIZE_WeightedMovingAverage];
//         weightedSum += weights[i] * samples_WeightedMovingAverage[(index_WeightedMovingAverage + i) % WINDOW_SIZE_WeightedMovingAverage];
//     }

//     // Calculate and return moving average
//     float average = weightedSum / WINDOW_SIZE_simple_moving_average;

//     // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f,%3.3f, and ,", samples_simple_moving_average[0],samples_simple_moving_average[1],samples_simple_moving_average[2],samples_simple_moving_average[3],samples_simple_moving_average[4]);

//     return average;
// }

void ModeStabilize::pilot_input()
{

    H_roll = (double)(channel_roll->get_control_in()) / 100.0; // range: (-45 to 45)
    // First ordered low pass filter
    H_roll = 0.686 * H_roll_previousValue + 0.314 * H_roll;
    H_roll_previousValue = H_roll;

    H_pitch = -(double)(channel_pitch->get_control_in()) / 100.0; // range: (-45 to 45)
    // First ordered low pass filter
    H_pitch = 0.686 * H_pitch_previousValue + 0.314 * H_pitch;
    H_pitch_previousValue = H_pitch;

    H_yaw_rate = -(double)(channel_yaw->get_control_in()) / 100.0; // range: (-45 to 45)
    // First ordered low pass filter
    H_yaw_rate = 0.686 * H_yaw_rate_previousValue + 0.314 * H_yaw_rate;
    H_yaw_rate_previousValue = H_yaw_rate;

    H_throttle = (double)(channel_throttle->get_control_in()) - 500.0; // range: (-500 to 500)
    // First ordered low pass filter
    H_throttle = 0.686 * H_throttle_previousValue + 0.314 * H_throttle;
    H_throttle_previousValue = H_throttle;

    //////// Common values
    float max_des_velocity_horizontal = 5.0; // (m/s)
    float max_des_velocity_vertical = 5.0;   // (m/s)
    float max_yaw_dot_ = 90.0;               // (deg/s)
    float slop_for_horizontal_scaling = 45.0 / max_des_velocity_horizontal;
    float slop_for_vertical_scaling = 500.0 / max_des_velocity_vertical;
    float slop_for_yaw_scaling = 45.0 / max_yaw_dot_;

    //////// Convert H_pitch into desired velocity in x
    // Trim (-2 to 2) data. This will help preventing the drone to drift
    if (H_pitch > -2.0 && H_pitch < 2.0)
    {
        H_pitch = 0.0;
    }
    // Start the values considering -2 and 2 as the origin
    if (H_pitch < -2.0 && H_pitch > -45.0)
    {
        H_pitch = H_pitch + 2.0;
    }
    if (H_pitch > 2.0 && H_pitch < 45.0)
    {
        H_pitch = H_pitch - 2.0;
    }
    // Scale the (-45 to 45) values to desired velocity limits
    x_des_dot = H_pitch / slop_for_horizontal_scaling;
    // First ordered low pass filter
    x_des_dot = 0.686 * x_des_dot_previousValue + 0.314 * x_des_dot;
    x_des_dot_previousValue = x_des_dot;
    x_des = x_des + (x_des_dot + x_des_dot_previousValue) / 2 * 0.0025;
    x_des = Bounds_on_XY_des(x_des);
    ////////////////////////////////

    //////// Convert H_roll into desired velocity in y
    // Trim (-2 to 2) data. This will help preventing the drone to drift
    if (H_roll > -2.0 && H_roll < 2.0)
    {
        H_roll = 0.0;
    }
    // Start the values considering -2 and 2 as the origin
    if (H_roll < -2.0 && H_roll > -45.0)
    {
        H_roll = H_roll + 2.0;
    }
    if (H_roll > 2.0 && H_roll < 45.0)
    {
        H_roll = H_roll - 2.0;
    }
    // Scale the (-45 to 45) values to desired velocity limits
    y_des_dot = H_roll / slop_for_horizontal_scaling;
    // First ordered low pass filter
    y_des_dot = 0.686 * y_des_dot_previousValue + 0.314 * y_des_dot;
    y_des_dot_previousValue = y_des_dot;
    y_des = y_des + (y_des_dot + y_des_dot_previousValue) / 2 * 0.0025;
    y_des = Bounds_on_XY_des(y_des);
    ////////////////////////////////

    //////// Convert H_throttle into desired velocity in z
    // Trim (-20 to 20) data. This will help preventing the drone to drift
    if (H_throttle > -20.0 && H_throttle < 20.0)
    {
        H_throttle = 0.0;
    }
    // Start the values considering -2 and 2 as the origin
    if (H_throttle < -20.0 && H_throttle > -500.0)
    {
        H_throttle = H_throttle + 20.0;
    }
    if (H_throttle > 20.0 && H_throttle < 500.0)
    {
        H_throttle = H_throttle - 20.0;
    }
    // Scale the (-500 to 500) values to desired velocity limits
    z_des_dot = H_throttle / slop_for_vertical_scaling;
    // First ordered low pass filter
    z_des_dot = 0.686 * z_des_dot_previousValue + 0.314 * z_des_dot;
    z_des_dot_previousValue = z_des_dot;
    z_des = z_des + (z_des_dot + z_des_dot_previousValue) / 2 * 0.0025;
    z_des = Bounds_on_Z_des(z_des);
    ////////////////////////////////

    //////// Convert H_yaw_rate into desired yaw rate
    // Trim (-2 to 2) data. This will help preventing the drone to drift
    if (H_yaw_rate > -2.0 && H_yaw_rate < 2.0)
    {
        H_yaw_rate = 0.0;
    }
    // Start the values considering -2 and 2 as the origin
    if (H_yaw_rate < -2.0 && H_yaw_rate > -45.0)
    {
        H_yaw_rate = H_yaw_rate + 2.0;
    }
    if (H_yaw_rate > 2.0 && H_yaw_rate < 45.0)
    {
        H_yaw_rate = H_yaw_rate - 2.0;
    }
    // Scale the (-45 to 45) values to desired velocity limits
    yaw_des_dot = H_yaw_rate / slop_for_yaw_scaling;
    // First ordered low pass filter
    yaw_des_dot = 0.686 * yaw_des_dot_previousValue + 0.314 * yaw_des_dot;
    yaw_des_dot_previousValue = yaw_des_dot;
    yaw_des = yaw_des + (yaw_des_dot + yaw_des_dot_previousValue) / 2 * 0.0025;
    yaw_des = wrap_360(yaw_des);
    ////////////////////////////////

    ///////// For stabilize mode
    rpyd[0] = H_roll;
    rpyd[1] = H_pitch;
    rpyd[2] = yaw_des;

    //////// To debug the code
    // hal.console->printf("%f,%f,%f,%f\n",x_des_dot,y_des_dot,z_des_dot,yaw_des_dot);
    // hal.console->printf("%f,%f,%f,%f\n",x_des,y_des,z_des,yaw_des);
    // hal.console->printf("%3.2f,%3.2f,%3.2f,%3.2f\n", rpyd[0], rpyd[1], rpyd[2], z_des);
}

float ModeStabilize::SQ_filter_fifth_order(float y_minus_2, float y_minus_1, float y, float y_plus_1, float y_plus_2)
{
    float SQ_fil_data = (1.0 / 35.0) * (-3.0 * y_minus_2 + 12.0 * y_minus_1 + 17.0 * y + 12.0 * y_plus_1 - 3.0 * y_plus_2);
    return SQ_fil_data;
}

float ModeStabilize::Bounds_on_XY_des(float value)
{
    float Bounds_on_XY_des_value = 5.0;
    if (value > Bounds_on_XY_des_value)
    {
        value = Bounds_on_XY_des_value;
    }
    if (value < -Bounds_on_XY_des_value)
    {
        value = -Bounds_on_XY_des_value;
    }
    return value;
}

float ModeStabilize::Bounds_on_Z_des(float value)
{
    float Bounds_on_XY_des_value = 5.0;
    if (value > Bounds_on_XY_des_value)
    {
        value = Bounds_on_XY_des_value;
    }
    if (value < -Bounds_on_XY_des_value)
    {
        value = -Bounds_on_XY_des_value;
    }
    return value;
}

int ModeStabilize::Inverse_thrust_function(float Force)
{
    int PWM = 1200;
    if (battvolt >= 11.5)
    {
        PWM = 1000 * (0.9206 + (sqrtf(12.8953 + 30.3264 * Force) / (15.1632)));
    }
    else
    {
        PWM = 1000 * (0.6021 + (sqrtf(33.2341 + 19.418 * Force) / (9.5740)));
    }
    if (PWM > 2000)
    {
        PWM = 2000;
    }
    if (PWM < 1200)
    {
        PWM = 1200;
    }
    return PWM;
}

void ModeStabilize::final_F_M_calling()
{

    float FM_devided_FF;
    if (battvolt >= 11.5)
    {
        FM_devided_FF = 0.24;
    }
    else
    {
        FM_devided_FF = 0.31;
    }

    float function_F1 = F / 4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) + Mb3 / (4.0 * FM_devided_FF);
    float function_F2 = F / 4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) + Mb3 / (4.0 * FM_devided_FF);
    float function_F3 = F / 4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) - Mb3 / (4.0 * FM_devided_FF);
    float function_F4 = F / 4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) - Mb3 / (4.0 * FM_devided_FF);

    PWM1 = Inverse_thrust_function(function_F1);
    PWM2 = Inverse_thrust_function(function_F2);
    PWM3 = Inverse_thrust_function(function_F3);
    PWM4 = Inverse_thrust_function(function_F4);

    PWM1 = 1000;
    PWM2 = 1000;
    PWM3 = 1000;
    PWM4 = 1000;
}