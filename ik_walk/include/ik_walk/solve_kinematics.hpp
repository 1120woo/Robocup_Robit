#ifndef SOLVE_KINEMATICS_HPP_
#define SOLVE_KINEMATICS_HPP_

#include <iostream>
#include <fstream>
#include <cmath>
#include "ros/ros.h"
#include "walk_pattern.hpp"

//msg---------------------------------------------
#include <msg_generate/Motor_msg.h>
#include "imu_position_control.hpp"
#include "zmp_position_control.hpp"
#include "pid_control_float.h"

#define PI 3.141592653589793
#define deg2rad PI/(double)180
#define rad2deg (double)180.0/PI

#define Leg 10
#define All 0


using namespace std;

class IK_Solve
{
public:

    ros::Publisher Motor_Pub;
    ros::Publisher Motor_Position_Pub;

    void solve(double pX_r, double pY_r, double pZ_r, double RYaw, double pX_l, double pY_l, double pZ_l, double LYaw, int body,double Shoulder_Pattern_X_R,double Shoulder_Pattern_X_L,double Shoulder_Pattern_Y_R,double Shoulder_Pattern_Y_L,double Balance_Theta1,double Balance_Theta2,double Balance_Theta3, double R_Rise_Condition, double L_Rise_Condition, double R_Rise_Max, double L_Rise_Max);
    double ang2pos(double angle);
    void init_save();
    void motor_packet(int body, int limit, double R_Rise_Condition, double L_Rise_Condition, double R_Rise_Max, double L_Rise_Max);
    void Balance_Control_Body_Upright(double Input_Data,double Robot_Z, double Time  ,double Rise_Condition , double Input_Data2 , double Balance_Value_Msg ,double Balance_Value_Msg2,double Zmp_Pitch_Limit_Left ,double Zmp_Pitch_Limit_Right ,double Input_Data3,double Balance_Value_Msg3,double Balance_Value_Msg4, double L_1, double Zmp_Roll_Limit_Left, double Zmp_Roll_Limit_Right ,int Support_Condition);


    double IK_X;
    double IMU_PITCH;
    double Amp_IK_X=0.0;
    double Amp_Pitch_Angle=0.0;
    double Amp_Roll_Angle=0.0;
    struct Balance_Theta
    {
        double Theta1=0;
        double Theta2=0;
        double Theta3=0;

        double Right_Theta1 = 0;
        double Right_Theta2 = 0;
        double Right_Theta3 = 0;

        double Left_Theta1 = 0;
        double Left_Theta2 = 0;
        double Left_Theta3 = 0;
    };
    Balance_Theta Now_Balance_Theta;

    struct Signal_Robot_Support_Foot
    {
        bool Right_Foot;
        bool Left_Foot;
        bool Both_Feet;
    }Signal_Robot_Support_Foot_Data;
    struct Motor_Angle
    {
        //1 ~ 4096
        int Motor_Angle_0 = 0;
        int Motor_Angle_1 = 0;
        int Motor_Angle_10 = 0;
        int Motor_Angle_11 = 0;
        int Motor_Angle_12 = 0;
        int Motor_Angle_13 = 0;
        int Motor_Angle_14 = 0;
        int Motor_Angle_15 = 0;
        int Motor_Angle_16 = 0;
        int Motor_Angle_17 = 0;
        int Motor_Angle_18 = 0;
        int Motor_Angle_19 = 0;
        int Motor_Angle_20 = 0;
        int Motor_Angle_21 = 0;


    };
    struct Percentage_of_IK_Motor
    {
        double Motor_Multiple_10 = 1.0;
        double Motor_Multiple_11 = 1.0;
        double Motor_Multiple_12 = 1.0;
        double Motor_Multiple_13 = 1.0;
        double Motor_Multiple_14 = 1.0;
        double Motor_Multiple_15 = 1.0;
        double Motor_Multiple_16 = 1.0;
        double Motor_Multiple_17 = 1.0;
        double Motor_Multiple_18 = 1.0;
        double Motor_Multiple_19 = 1.0;
        double Motor_Multiple_20 = 1.0;
        double Motor_Multiple_21 = 1.0;
    };
    Motor_Angle Now_Motor_Angle;
    Motor_Angle Past_Motor_Angle;
    Motor_Angle Buffer_Motor_Angle;

    Percentage_of_IK_Motor Now_Percentage_of_IK_Motor;
    Percentage_of_IK_Motor Past_Percentage_of_IK_Motor;

    struct Compensate_Angle
    {
        double Compensate_Angle_Arm_2 = 0.0;
        double Compensate_Angle_Arm_3 = 0.0;
        double Compensate_Angle_Left_10 = 0.0;
        double Compensate_Angle_Left_14 = 0.0;
        double Compensate_Angle_Left_16 = 0.0;
        double Compensate_Angle_Left_18 = 0.0;
        double Compensate_Angle_Left_20 = 0.0;
        double Compensate_Angle_Right_11 = 0.0;
        double Compensate_Angle_Right_15 = 0.0;
        double Compensate_Angle_Right_17 = 0.0;
        double Compensate_Angle_Right_19 = 0.0;
        double Compensate_Angle_Right_21 = 0.0;
        double Horizontal_Angle = 0.0;
        double Vertical_Angle = 0.0;
        double R_Vertical_Angle = 0.0;
        double L_Vertical_Angle = 0.0;

    }Compensate_Angle_Data;

    int now_15_Angle_value;
    int now_16_Angle_value;


private:

    unsigned int g_DXL_ID_position[30] = {0, }; //output
    unsigned int g_DXL_ID_Save_position[30] = {0, };
    double InvLegAngle[12] = {0, };
    int Motion_Data[400] = {0, };
    bool ikinit_flag = true;

    //foot_trajectory angle_speed;
};


#endif /* SOLVE_HPP_ */
