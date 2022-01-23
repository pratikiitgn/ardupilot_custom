#pragma once

extern float H_roll;
extern float H_pitch;
extern float H_yaw;

extern float imu_roll;
extern float imu_pitch;
extern float imu_yaw;

extern float imu_roll_dot;
extern float imu_pitch_dot;
extern float imu_yaw_dot;

extern float quad_x;
extern float quad_y;
extern float quad_z;

extern float quad_x_dot;
extern float quad_y_dot;
extern float quad_z_dot;

extern float H_yaw;
extern float H_yaw_rate;
extern float z_des;
extern float H_throttle;

extern uint16_t PWM1;
extern uint16_t PWM2;
extern uint16_t PWM3;
extern uint16_t PWM4;

extern uint16_t Pf;
extern uint16_t Pm1;
extern uint16_t Pm2;
extern uint16_t Pm3;
extern uint16_t Pm4;

extern float t_ph_sys_ID;
extern int arm_disarm_flag;

extern float F;
extern float Mb1;
extern float Mb2;
extern float Mb3;

extern float x_des;
extern float y_des;
extern float z_des;

// Variable for geometric controller
extern Matrix3f R_log;
extern Matrix3f Rd_log;
extern Vector3f e_R_log;
extern Vector3f e_Omega_log;

// extern float des_phi;
// extern float des_theta;
// extern float des_yaw;


// From the CAM device 
extern float encoder_roll_feedback;
extern float encoder_pitch_feedback;
