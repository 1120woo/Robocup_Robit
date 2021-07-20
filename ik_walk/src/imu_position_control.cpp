#include "../include/ik_walk/imu_position_control.hpp"
#include "../include/ik_walk/ik_walk.hpp"

//2019.5.09 pitch_control_success
//2019.5.10 roll_control_try_01
//Author robit

double Knee_Add_Angle = 0;
double pos_pitch_OLIMIT = 0;
double neg_pitch_OLIMIT = 0;

using namespace std;

void IK_pos_control::cout_imu_data(float receive_r,float receive_p,float receive_y)
{
    //cout << receive_r << endl;
    //cout << receive_p << endl;
    //cout << receive_y << endl;
}

void IK_pos_control::PD_Pitch_control(double input,double pitch_GP,double pitch_GI,double pitch_GD,double pitch_ELIMIT,double pitch_OLIMIT,double pitch_neg_Target,double pitch_pos_Target)
{

    if(pitch_pos_Target >= input && pitch_neg_Target <= input)
    {
        pitch_OLIMIT = 1;
    }

//    cout << "=====================================" << endl;
//    cout << "pitch_pos_Target     >>"  << pitch_pos_Target << endl;
//    cout << "pitch_neg_Target     >> " << pitch_neg_Target << endl;
//    cout << "=====================================" << endl;


    PID_Control_init(&pitch_control, pitch_GP, pitch_GI, pitch_GD, pitch_ELIMIT, pitch_OLIMIT);

    if(input > pitch_pos_Target)
    {
        PID_Control_Float(&pitch_control, pitch_pos_Target, input);
        Pitch_ADD_Angle = pitch_control.nowOutput;

        cout<<"Pitch_ADD_Angle  "<<Pitch_ADD_Angle<<endl;

//        cout << "pos_Target // input     >>"  << input << endl;
//        cout << "pos_Target // nowOutput >> " << pitch_control.nowOutput << endl;
    }
    else if(input < pitch_neg_Target)
    {
        PID_Control_Float(&pitch_control, pitch_neg_Target, input);
        Pitch_ADD_Angle = pitch_control.nowOutput;

//        cout << "neg_Target // input    >>"  << input << endl;
//        cout << "neg_Target //nowOutput >> " << pitch_control.nowOutput << endl;
    }
    else
    {
        Pitch_ADD_Angle = 0;
    }

//    cout << "pitch_Olimit = " << pitch_OLIMIT << endl;
}

void IK_pos_control::PD_Roll_control(double input,double roll_GP,double roll_GI,double roll_GD,double roll_ELIMIT,double roll_OLIMIT,double roll_neg_Target,double roll_pos_Target)
{

    if(roll_pos_Target >= input && roll_neg_Target <= input)
    {
        roll_OLIMIT = 1;
    }

//    cout << "=====================================" << endl;
//    cout << "roll_pos_Target     >>"  << roll_pos_Target << endl;
//    cout << "roll_neg_Target     >> " << roll_neg_Target << endl;
//    cout << "=====================================" << endl;

    PID_Control_init(&roll_control,roll_GP, roll_GI, roll_GD, roll_ELIMIT, roll_OLIMIT);

    if(input > roll_pos_Target)
    {
        PID_Control_Float(&roll_control, roll_pos_Target, input);
        Roll_ADD_Angle = roll_control.nowOutput;

//        cout << "pos_Target // input     >>"  << input << endl;
//        cout << "pos_Target // nowOutput >> " << roll_control.nowOutput << endl;
    }
    else if(input < roll_neg_Target)
    {
        PID_Control_Float(&roll_control, roll_neg_Target, input);
        Roll_ADD_Angle = roll_control.nowOutput;

//        cout << "neg_Target // input     >>"  << input << endl;
//        cout << "neg_Target // nowOutput >> " << roll_control.nowOutput << endl;
    }
    else
    {
        Roll_ADD_Angle = 0;
    }

//    cout << "roll_control.input     >> " << input << endl;
//    cout << "roll_control.nowoutput >> " << roll_control.nowOutput << endl;

}


