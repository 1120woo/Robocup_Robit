#include "../include/ik_walk/solve_kinematics.hpp"
#include "../include/ik_walk/ik_walk.hpp"


double IK_Solve::ang2pos(double angle)
{
    return (double)((angle * 4096.0) / 360.0);
}

void IK_Solve::init_save()
{
    std::string addr;

    addr = "/home/robit/catkin_ws/src/ik_walk/init/0320_initm.h";

    std::ifstream HeaderFile(addr.c_str());

    if(!(HeaderFile.is_open())){
        cout<<"---------------Warning-----------"<<endl;
        cout<<"-------Not Defined Init File-----"<<endl;
        cout<<"---------------------------------"<<endl;
        std::exit(0);
    }
    int i;
    for(i = 0; i < 2; i++)
    {
        HeaderFile.ignore(65535, '[');
    }
    HeaderFile.ignore(100, '{');

    i = 0;
    char ch;
    while(!HeaderFile.eof())
    {
        HeaderFile >> Motion_Data[i];
        HeaderFile >> ch;
        if(ch == ' ')
            break;
        if(ch == '}')
            break;
        i++;
        if(i > 400)
            break;
    }
    HeaderFile.close();

    for(int DXL_ID = 0; DXL_ID <= 22; DXL_ID++)
    {
        g_DXL_ID_position[DXL_ID] = Motion_Data[11 + DXL_ID]*4;
        g_DXL_ID_Save_position[DXL_ID] = Motion_Data[11 + DXL_ID]*4;

        cout<<"g_DXL_ID_Save_position["<<DXL_ID<<"] = "<<(g_DXL_ID_Save_position[DXL_ID]/4)<<endl;
    }
}
void IK_Solve::Balance_Control_Body_Upright(double Input_Data, double Robot_Z, double Time  ,double Rise_Condition , double Input_Data2 , double Balance_Value_Msg ,double Balance_Value_Msg2 ,double Zmp_Pitch_Limit_Left ,double Zmp_Pitch_Limit_Right ,double Input_Data3 ,double Balance_Value_Msg3,double Balance_Value_Msg4, double L_1, double Zmp_Roll_Limit_Left, double Zmp_Roll_Limit_Right, int Support_Condition)
{



    //%Link_Parameter%
    double Link_1 = L_1;
    double Link_2 = L_1;

    //%Receieve Parameter from UI that x,y coordinate%

    //Assume robot center



    //cout<<"sss : "<<Input_Data<<endl;

    if(Zmp_Pitch_Limit_Left >= 60.0 || Zmp_Pitch_Limit_Right >= 60.0)
    {
        cout<<"Min"<<endl;
        Amp_IK_X = 1.0;
//        Amp_Pitch_Angle = 4.0;
    }
    else if(Zmp_Pitch_Limit_Left <= -60.0 || Zmp_Pitch_Limit_Right <= -60.0)
    {
        cout<<"Max"<<endl;
        Amp_IK_X = 1.0;
//        Amp_Pitch_Angle = 2.0;
    }
    else
    {
        cout<<"Normal"<<endl;
        Amp_IK_X = 0.5;
        Amp_Pitch_Angle = 1.0;
    }

    if(Zmp_Roll_Limit_Left >= 150.0 || Zmp_Roll_Limit_Right >= 150.0)
    {
        Amp_Roll_Angle = 1.0;
    }
    else if(Zmp_Roll_Limit_Left >= -150.0 || Zmp_Roll_Limit_Right >= -150.0)
    {
        Amp_Roll_Angle = 1.0;
    }
    else
    {
        Amp_Roll_Angle = 1.0;
    }



    IK_X = ((Input_Data) * Balance_Value_Msg*Amp_IK_X) ;
    double IK_Y = Robot_Z; //Assume robot leg height190 when Theta2 = 90 => height =190


//    Compensate_Angle_Data.Horizontal_Angle = ((Input_Data2) * Balance_Value_Msg2*Amp_Pitch_Angle);
    double V_Cos_Theta = 1 - (Input_Data3*Input_Data3)/(2*IK_Y*IK_Y);
    double V_Sin_Theta = sqrt(1-(V_Cos_Theta*V_Cos_Theta));
    Compensate_Angle_Data.Vertical_Angle = atan2(V_Sin_Theta,V_Cos_Theta) * Balance_Value_Msg3; //unit is Radian

    Compensate_Angle_Data.Compensate_Angle_Arm_2 = Input_Data3 * Balance_Value_Msg4;
    Compensate_Angle_Data.Compensate_Angle_Arm_3 = Input_Data3 * Balance_Value_Msg4;

    cout<<"Input_Data"<<Input_Data<<endl<<endl;
    cout<<"IK_X : "<<IK_X<<endl;
    if(IK_X>70.0)IK_X=70.0;
    else if(IK_X<-70.0)IK_X=-70.0;


    //%Inverse_Kinematic RR%
    double D_Cos_Theta2 = ((IK_X*IK_X)+(IK_Y*IK_Y)-(Link_1*Link_1)-(Link_2*Link_2))/(2*Link_1*Link_2);
    double D_Sin_Theta2 = sqrt(1-(D_Cos_Theta2*D_Cos_Theta2));
    Now_Balance_Theta.Theta2 = atan2(D_Sin_Theta2,D_Cos_Theta2);  //unit is Radian
    double D_Cos_Theta1 = ((Link_1+Link_2*cos(Now_Balance_Theta.Theta2))*IK_X+Link_2*sin(Now_Balance_Theta.Theta2)*IK_Y)/((Link_1+Link_2*cos(Now_Balance_Theta.Theta2))*((Link_1+Link_2*cos(Now_Balance_Theta.Theta2)))+((Link_2*sin(Now_Balance_Theta.Theta2)))*((Link_2*sin(Now_Balance_Theta.Theta2))));
    double D_Sin_Theta1 = (((-Link_2)*sin(Now_Balance_Theta.Theta2)*IK_X)+((Link_1+Link_2*cos(Now_Balance_Theta.Theta2))*IK_Y))/((Link_1+Link_2*cos(Now_Balance_Theta.Theta2))*((Link_1+Link_2*cos(Now_Balance_Theta.Theta2)))+((Link_2*sin(Now_Balance_Theta.Theta2))*(Link_2*sin(Now_Balance_Theta.Theta2))));
    Now_Balance_Theta.Theta1 = atan2(D_Sin_Theta1,D_Cos_Theta1);  //unit is Radian

    //%Inverse_Kinematic End_Effector%
    Now_Balance_Theta.Theta3 = (90*deg2rad)-(Now_Balance_Theta.Theta1)-(Now_Balance_Theta.Theta2); //%unit is Radian


    cout<<"Support_Condition :             "<<Support_Condition<<endl;
    if(Support_Condition == 1)
    {
        Now_Balance_Theta.Right_Theta1 = Now_Balance_Theta.Theta1;
        Now_Balance_Theta.Right_Theta2 = Now_Balance_Theta.Theta2;
        Now_Balance_Theta.Right_Theta3 = Now_Balance_Theta.Theta3;
        Compensate_Angle_Data.R_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;

    }
    else if(Support_Condition == -1)
    {

        Now_Balance_Theta.Left_Theta1 = Now_Balance_Theta.Theta1;
        Now_Balance_Theta.Left_Theta2 = Now_Balance_Theta.Theta2;
        Now_Balance_Theta.Left_Theta3 = Now_Balance_Theta.Theta3;
        Compensate_Angle_Data.L_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;

    }
    else if(Support_Condition == 0 || Support_Condition == 99)
    {
        Now_Balance_Theta.Right_Theta1 = Now_Balance_Theta.Theta1;
        Now_Balance_Theta.Right_Theta2 = Now_Balance_Theta.Theta2;
        Now_Balance_Theta.Right_Theta3 = Now_Balance_Theta.Theta3;

        Now_Balance_Theta.Left_Theta1 = Now_Balance_Theta.Theta1;
        Now_Balance_Theta.Left_Theta2 = Now_Balance_Theta.Theta2;
        Now_Balance_Theta.Left_Theta3 = Now_Balance_Theta.Theta3;

        Compensate_Angle_Data.Vertical_Angle = 0;
        Compensate_Angle_Data.R_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;
        Compensate_Angle_Data.L_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;
    }

//    Signal_Robot_Support_Foot_Data.Both_Feet=true;



//    if(Rise_Condition==0.0)
//    {
//        Signal_Robot_Support_Foot_Data.Both_Feet=true;
//    }

//    else if(Rise_Condition>0)
//    {
//        if(Time <0 )
//        {
//            Signal_Robot_Support_Foot_Data.Both_Feet=true;
//            //        Signal_Robot_Support_Foot_Data.Right_Foot=false;
//            //        Signal_Robot_Support_Foot_Data.Left_Foot=false;
//        }
//        else if(0.0625<Time && Time<0.4375)
//        {
//            Signal_Robot_Support_Foot_Data.Right_Foot=true;
//            //        Signal_Robot_Support_Foot_Data.Left_Foot=true;
//            //        Signal_Robot_Support_Foot_Data.Both_Feet=true;
//        }
//        else if(0.5625<Time && Time<0.9375)
//        {
//            Signal_Robot_Support_Foot_Data.Left_Foot=true;
//            //        Signal_Robot_Support_Foot_Data.Right_Foot=true;
//            //        Signal_Robot_Support_Foot_Data.Both_Feet=true;
//        }
//        else
//        {
//            Signal_Robot_Support_Foot_Data.Both_Feet=true;
//            //        Signal_Robot_Support_Foot_Data.Right_Foot=true;
//            //        Signal_Robot_Support_Foot_Data.Left_Foot=true;
//        }


//    }

    //cout<<"Horizen : "<<Compensate_Angle_Data.Horizontal_Angle<<endl;
    cout<<"ZMP : "<<IK_X<<endl;
    cout<<"Rise_condition"<<Rise_Condition<<endl;



}

void IK_Solve::solve(double pX_r, double pY_r, double pZ_r, double RYaw, double pX_l, double pY_l, double pZ_l, double LYaw, int body,double Shoulder_Pattern_X_R, double Shoulder_Pattern_X_L,double Shoulder_Pattern_Y_R, double Shoulder_Pattern_Y_L, double Balance_Theta1,double Balance_Theta2,double Balance_Theta3, double R_Rise_Condition, double L_Rise_Condition, double R_Rise_Max, double L_Rise_Max)
{
    const double l_0 = 55/*60.5*/;      //      골반 너비
    const double l_1 = 55/*60.5*/;      //      골반 너비
    const double l_2 = 130;      //     축간 거리
    const double l_3 = 130;      //     축간 거리

    double LYaw1;
    double RYaw1;
    double LValid = 0;
    double RValid = 0;
    double temp[4] = {0,0,0,0};
    double temp1[4] = {0,0,0,0};
    msg_generate::ik_angle_sim Position_Info;

    LYaw1 = (double)(LYaw*deg2rad);

    LValid = (double)(((pX_l*pX_l) + ((pY_l-l_0)*(pY_l-l_0)) + ((pZ_l)*(pZ_l)) - (l_2*l_2) - (l_3*l_3) ) / (2*l_2*l_3));

    InvLegAngle[0] = LYaw1;
    InvLegAngle[3] = (double)acos(LValid);
    InvLegAngle[1] = (double)(atan2(pZ_l, -pX_l*(double)sin(LYaw1)+(pY_l-l_0)*(double)cos(LYaw1)));

    temp[0] = (double)(-l_3*sin(InvLegAngle[3]));
    temp[1] = (double)(l_2+l_3*cos(InvLegAngle[3]));
    temp[2] = (double)(pX_l*cos(LYaw1)+(pY_l-l_0)*sin(LYaw1));
    temp[3] = (double)(-pX_l*sin(LYaw1)*cos(InvLegAngle[1])+(pY_l-l_0)*cos(LYaw1)*cos(InvLegAngle[1])+pZ_l*sin(InvLegAngle[1]));

    LValid = (double)((temp[0]*temp[3]-temp[1]*temp[2])/(temp[0]*temp[0]+temp[1]*temp[1]));

    InvLegAngle[2] = (double)asin(LValid);
    InvLegAngle[1] = (double)((double)InvLegAngle[1] + (double)(PI/2.0));

    InvLegAngle[4] = -InvLegAngle[3] - InvLegAngle[2];
    InvLegAngle[5] = (double)(1.0*(double)InvLegAngle[1]);

    //------------------------------------------------------------------------------------------------

    RYaw1 = (double)(RYaw*deg2rad);

    RValid = (double)(((pX_r*pX_r) + ((pY_r+l_0)*(pY_r+l_0)) + ((pZ_r)*(pZ_r)) - (l_2*l_2) - (l_3*l_3) ) / (2*l_2*l_3));

    InvLegAngle[6] = RYaw1;
    InvLegAngle[9] = (double)acos(RValid);
    InvLegAngle[7] = (double)(atan2(pZ_r, -pX_r*sin(RYaw1)+(l_0+pY_r)*cos(RYaw1)));

    temp1[0] = (double)(-l_3*sin(InvLegAngle[9]));
    temp1[1] = (double)(l_2+l_3*cos(InvLegAngle[9]));
    temp1[2] = (double)(pX_r*cos(RYaw1)+(pY_r+l_0)*sin(RYaw1));
    temp1[3] = (double)(-pX_r*sin(RYaw1)*cos(InvLegAngle[7])+(pY_r+l_0)*cos(RYaw1)*cos(InvLegAngle[7])+pZ_r*sin(InvLegAngle[7]));

    RValid = (double)((temp1[0]*temp1[3]-temp1[1]*temp1[2])/(temp1[0]*temp1[0]+temp1[1]*temp1[1]));

    InvLegAngle[8] = (double)asin(RValid);
    InvLegAngle[7] = (double)((double)InvLegAngle[7]+PI/2);

    InvLegAngle[10] = -InvLegAngle[9] - InvLegAngle[8];
    InvLegAngle[11] = (double)(1.0*(double)InvLegAngle[7]);

    //cout<<"Compensate_Angle_Data.Vertical_Angle"<<Compensate_Angle_Data.Vertical_Angle<<endl;
    for(int i = 0; i < 23; i++)
    {
        g_DXL_ID_position[i] = g_DXL_ID_Save_position[i];
    }

    //case1
//        if(Signal_Robot_Support_Foot_Data.Both_Feet)
//        {
//            cout<<"xxx"<<endl;
//            Compensate_Angle_Data.Compensate_Angle_Left_14  += 0/*InvLegAngle[2]*rad2deg-((Now_Balance_Theta.Theta3*rad2deg)-180)*/;
//            Compensate_Angle_Data.Compensate_Angle_Left_16  += 0/*InvLegAngle[3]*rad2deg-(Now_Balance_Theta.Theta2*rad2deg)*/;
//            Compensate_Angle_Data.Compensate_Angle_Left_18  += 0/*InvLegAngle[4]*rad2deg-((Now_Balance_Theta.Theta1*rad2deg)+90)*/;

//            Compensate_Angle_Data.Compensate_Angle_Right_15 += 0/*InvLegAngle[8]*rad2deg-((Now_Balance_Theta.Theta3*rad2deg)-180)*/;
//            Compensate_Angle_Data.Compensate_Angle_Right_17 += 0/*InvLegAngle[9]*rad2deg-(Now_Balance_Theta.Theta2*rad2deg)*/;
//            Compensate_Angle_Data.Compensate_Angle_Right_19 += 0/*InvLegAngle[10]*rad2deg-((Now_Balance_Theta.Theta1*rad2deg)+90)*/;
//        }
//        else if(Signal_Robot_Support_Foot_Data.Right_Foot)
//        {
//            Compensate_Angle_Data.Compensate_Angle_Left_14  += 0;
//            Compensate_Angle_Data.Compensate_Angle_Left_16  += 0;
//            Compensate_Angle_Data.Compensate_Angle_Left_18  += 0;
//            Compensate_Angle_Data.Compensate_Angle_Right_15 +=-((Now_Balance_Theta.Theta3*rad2deg)-180);
//            Compensate_Angle_Data.Compensate_Angle_Right_17 +=-(Now_Balance_Theta.Theta2*rad2deg);
//            Compensate_Angle_Data.Compensate_Angle_Right_19 += 0/*InvLegAngle[10]*rad2deg-((Now_Balance_Theta.Theta1*rad2deg)+90)*/;
//        }
//        else if(Signal_Robot_Support_Foot_Data.Left_Foot)
//        {
//            Compensate_Angle_Data.Compensate_Angle_Left_14  +=-((Now_Balance_Theta.Theta3*rad2deg)-180);
//            Compensate_Angle_Data.Compensate_Angle_Left_16  +=-(Now_Balance_Theta.Theta2*rad2deg);
//            Compensate_Angle_Data.Compensate_Angle_Left_18  += 0/*InvLegAngle[4]*rad2deg-((Now_Balance_Theta.Theta1*rad2deg)+90)*/;
//            Compensate_Angle_Data.Compensate_Angle_Right_15 += 0;
//            Compensate_Angle_Data.Compensate_Angle_Right_17 += 0;
//            Compensate_Angle_Data.Compensate_Angle_Right_19 += 0;
//        }

    //case2
    //        Compensate_Angle_Data.Compensate_Angle_Left_14  = InvLegAngle[2]*rad2deg-((Now_Balance_Theta.Theta3*rad2deg)-180);
    //        Compensate_Angle_Data.Compensate_Angle_Left_16  = InvLegAngle[3]*rad2deg-(Now_Balance_Theta.Theta2*rad2deg);
    //        Compensate_Angle_Data.Compensate_Angle_Left_18  = InvLegAngle[4]*rad2deg-((Now_Balance_Theta.Theta1*rad2deg)+90);

    //        Compensate_Angle_Data.Compensate_Angle_Right_15 = InvLegAngle[8]*rad2deg-((Now_Balance_Theta.Theta3*rad2deg)-180);
    //        Compensate_Angle_Data.Compensate_Angle_Right_17 = InvLegAngle[9]*rad2deg-(Now_Balance_Theta.Theta2*rad2deg);
    //        Compensate_Angle_Data.Compensate_Angle_Right_19 = InvLegAngle[10]*rad2deg-((Now_Balance_Theta.Theta1*rad2deg)+90);



    Compensate_Angle_Data.Compensate_Angle_Left_10  = InvLegAngle[1]*rad2deg - (Compensate_Angle_Data.L_Vertical_Angle*rad2deg) + (Imu_Balance.Roll_ADD_Angle*2)/*-Imu_Info.roll_acc*0.01*/;
    Compensate_Angle_Data.Compensate_Angle_Left_14  = InvLegAngle[2]*rad2deg-((Now_Balance_Theta.Left_Theta3*rad2deg)-180)+(Imu_Balance.Pitch_ADD_Angle*2.3)/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
    Compensate_Angle_Data.Compensate_Angle_Left_16  = InvLegAngle[3]*rad2deg-(Now_Balance_Theta.Left_Theta2*rad2deg)-(Imu_Balance.Pitch_ADD_Angle*2.3);
    Compensate_Angle_Data.Compensate_Angle_Left_18  = InvLegAngle[4]*rad2deg-((Now_Balance_Theta.Left_Theta1*rad2deg)+90)/*+(Imu_Balance.Pitch_ADD_Angle*1.2)*/;
    Compensate_Angle_Data.Compensate_Angle_Left_20 = -InvLegAngle[5]*rad2deg + (Compensate_Angle_Data.L_Vertical_Angle*rad2deg) /*- (Imu_Balance.Roll_ADD_Angle*1)*/;

    Compensate_Angle_Data.Compensate_Angle_Right_11  = InvLegAngle[7]*rad2deg + (Compensate_Angle_Data.R_Vertical_Angle*rad2deg) + (Imu_Balance.Roll_ADD_Angle*2)/*-Imu_Info.roll_acc*0.01*/;
    Compensate_Angle_Data.Compensate_Angle_Right_15 = InvLegAngle[8]*rad2deg-((Now_Balance_Theta.Right_Theta3*rad2deg)-180)+(Imu_Balance.Pitch_ADD_Angle*2.3)/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
    Compensate_Angle_Data.Compensate_Angle_Right_17 = InvLegAngle[9]*rad2deg-(Now_Balance_Theta.Right_Theta2*rad2deg)-(Imu_Balance.Pitch_ADD_Angle*2.3);
    Compensate_Angle_Data.Compensate_Angle_Right_19 = InvLegAngle[10]*rad2deg-((Now_Balance_Theta.Right_Theta1*rad2deg)+90)/*+(Imu_Balance.Pitch_ADD_Angle*1.2)*/;
    Compensate_Angle_Data.Compensate_Angle_Right_21 = -InvLegAngle[11]*rad2deg - (Compensate_Angle_Data.R_Vertical_Angle*rad2deg)/*- (Imu_Balance.Roll_ADD_Angle*1)*/;


//    cout<<"Compensate_Angle_Data.Compensate_Angle_Left_14"<<endl;
//    cout<<Compensate_Angle_Data.Compensate_Angle_Left_14<<endl;


//    cout<<"Now_Balance_Theta.Theta1*rad2deg"<<endl;
//    cout<<Now_Balance_Theta.Theta1*rad2deg<<endl;
//    cout<<"Now_Balance_Theta.Theta2*rad2deg"<<endl;
//    cout<<Now_Balance_Theta.Theta2*rad2deg<<endl;
//    cout<<"Now_Balance_Theta.Theta3*rad2deg"<<endl;
//    cout<<Now_Balance_Theta.Theta3*rad2deg<<endl;

//    cout<<"Compensate_Angle_Data.Compensate_Angle_Right_15"<<endl;
//    cout<<Compensate_Angle_Data.Compensate_Angle_Right_15<<endl;
//    cout<<"Compensate_Angle_Data.Compensate_Angle_Left_16"<<endl;
//    cout<<Compensate_Angle_Data.Compensate_Angle_Left_16<<endl;
//    cout<<"Compensate_Angle_Data.Compensate_Angle_Right_17"<<endl;
//    cout<<Compensate_Angle_Data.Compensate_Angle_Right_17<<endl;
//    cout<<"Imu_Balance.Pitch_ADD_Angle  "<<Imu_Balance.Pitch_ADD_Angle<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_10"<<Now_Motor_Angle.Motor_Angle_10<<endl;

    if(Shoulder_Pattern_Y_R<=0.0)
    {
        Shoulder_Pattern_Y_R = 0.0;
    }
    else if(Shoulder_Pattern_Y_L<=0.0)
    {
        Shoulder_Pattern_Y_L = 0.0;
    }


   // cout<<"11 : "<<g_DXL_ID_position[11]<<endl;
    g_DXL_ID_position[0] -= ang2pos(Shoulder_Pattern_X_R - 2.5*Imu_Balance.Pitch_ADD_Angle/*-Compensate_Angle_Data.Horizontal_Angle*3*/);
    g_DXL_ID_position[1] += ang2pos(Shoulder_Pattern_X_L - 2.5*Imu_Balance.Pitch_ADD_Angle/*-Compensate_Angle_Data.Horizontal_Angle*3*/);
    g_DXL_ID_position[2] -= ang2pos(-2.0*Shoulder_Pattern_Y_R + Compensate_Angle_Data.Compensate_Angle_Arm_2);
    g_DXL_ID_position[3] -= ang2pos(-2.0*Shoulder_Pattern_Y_L + Compensate_Angle_Data.Compensate_Angle_Arm_3);

    g_DXL_ID_position[12] -= ang2pos(InvLegAngle[0]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_12)     +    Now_Motor_Angle.Motor_Angle_12;       //left_Yaw
    g_DXL_ID_position[14] += ang2pos(InvLegAngle[2]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_14+Compensate_Angle_Data.Compensate_Angle_Left_14)     +    Now_Motor_Angle.Motor_Angle_14 - Now_Param.X.X*0.5;       //left_pitch
    g_DXL_ID_position[16] += ang2pos(InvLegAngle[3]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_16+Compensate_Angle_Data.Compensate_Angle_Left_16)     +    Now_Motor_Angle.Motor_Angle_16;       //left_pitch
    g_DXL_ID_position[18] -= ang2pos(InvLegAngle[4]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_18+Compensate_Angle_Data.Compensate_Angle_Left_18/*+Compensate_Angle_Data.Horizontal_Angle*/)     +    Now_Motor_Angle.Motor_Angle_18 ;      //left_pitch
    g_DXL_ID_position[10] -= ang2pos(InvLegAngle[1]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_10 + Compensate_Angle_Data.Compensate_Angle_Left_10)     +    Now_Motor_Angle.Motor_Angle_10;       //left_roll
    g_DXL_ID_position[20] += ang2pos((-InvLegAngle[5]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_20/**2.5*//*+Compensate_Angle_Data.L_Vertical_Angle*/ + Compensate_Angle_Data.Compensate_Angle_Left_20))  +    Now_Motor_Angle.Motor_Angle_20;       //left_roll

    g_DXL_ID_position[13] -= ang2pos(InvLegAngle[6]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_13)     +    Now_Motor_Angle.Motor_Angle_13;       //right_Yaw
    g_DXL_ID_position[15] -= ang2pos(InvLegAngle[8]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_15+Compensate_Angle_Data.Compensate_Angle_Right_15)     +    Now_Motor_Angle.Motor_Angle_15 - Now_Param.X.X*0.5;       //right_pitch
    g_DXL_ID_position[17] -= ang2pos(InvLegAngle[9]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_17+Compensate_Angle_Data.Compensate_Angle_Right_17)     +    Now_Motor_Angle.Motor_Angle_17;       //right_pitch
    g_DXL_ID_position[19] += ang2pos(InvLegAngle[10]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_19+Compensate_Angle_Data.Compensate_Angle_Right_19/*+Compensate_Angle_Data.Horizontal_Angle*/)    +    Now_Motor_Angle.Motor_Angle_19;       //right_pitch
    g_DXL_ID_position[11] -= ang2pos(InvLegAngle[7]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_11/**0.4*/ + Compensate_Angle_Data.Compensate_Angle_Right_11)     +    Now_Motor_Angle.Motor_Angle_11;       //right_roll
    g_DXL_ID_position[21] += ang2pos((-InvLegAngle[11]*rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_21/**5.5*//*+Compensate_Angle_Data.R_Vertical_Angle*/ + Compensate_Angle_Data.Compensate_Angle_Right_21)) +    Now_Motor_Angle.Motor_Angle_21;       //right_roll

//    cout<<"11 : "<<g_DXL_ID_position[11]<<endl;
    //now_16_Angle_value = g_DXL_ID_position[16];
    //now_15_Angle_value = g_DXL_ID_position[15];

    //    cout<<"E14 : "<<ang2pos(Compensate_Angle_Data.Compensate_Angle_Left_14)<<endl;
    //    cout<<"E16 : "<<ang2pos(Compensate_Angle_Data.Compensate_Angle_Left_16)<<endl;
    //    cout<<"E18 : "<<ang2pos(Compensate_Angle_Data.Compensate_Angle_Left_18)<<endl;
    //    cout<<"E15 : "<<ang2pos(Compensate_Angle_Data.Compensate_Angle_Right_15)<<endl;
    //    cout<<"E17 : "<<ang2pos(Compensate_Angle_Data.Compensate_Angle_Right_17)<<endl;
    //    cout<<"E19 : "<<ang2pos(Compensate_Angle_Data.Compensate_Angle_Right_19)<<endl;
    //    cout<<"ang2pos(InvLegAngle[8]*rad2deg) : "<< ang2pos(InvLegAngle[8]*rad2deg)<<endl;

    cout<<"10 : "<<g_DXL_ID_position[10]<<endl;
    cout<<"11 : "<<g_DXL_ID_position[11]<<endl;
////    cout<<"12 : "<<g_DXL_ID_position[12]<<endl;
////    cout<<"13 : "<<g_DXL_ID_position[13]<<endl;
//    cout<<"14 : "<<g_DXL_ID_position[14]<<endl;
//    cout<<"15 : "<<g_DXL_ID_position[15]<<endl;
//    cout<<"16 : "<<g_DXL_ID_position[16]<<endl;
//    cout<<"17 : "<<g_DXL_ID_position[17]<<endl;
//    cout<<"18 : "<<g_DXL_ID_position[18]<<endl;
//    cout<<"19 : "<<g_DXL_ID_position[19]<<endl;
//    cout<<"20 : "<<g_DXL_ID_position[20]<<endl;
//    cout<<"21 : "<<g_DXL_ID_position[21]<<endl;

//    cout<<"POS // Now_Motor_Angle.Motor_Angle_20 : "<<Now_Motor_Angle.Motor_Angle_20<<endl;
//    cout<<"ANG //Compensate_Angle_Data.Compensate_Angle_Left_20: "<<Compensate_Angle_Data.Compensate_Angle_Left_20<<endl;
//    cout<<"ANG //InvLegAngle[5]*rad2deg : "<<InvLegAngle[5]*rad2deg<<endl;
//    cout<<"ANG //Compensate_Angle_Data.R_Vertical_Angle : "<<Compensate_Angle_Data.R_Vertical_Angle<<endl;
//    cout<<"ANG //Compensate_Angle_Data.L_Vertical_Angle : "<<Compensate_Angle_Data.L_Vertical_Angle<<endl;
    cout<<"Compensate_Angle_Data.R_Vertical_Angle*rad2deg"<<Compensate_Angle_Data.R_Vertical_Angle*rad2deg<<endl;

    /////////////////////////////simulation/////////////////////////
    Position_Info.ang_l_0 = g_DXL_ID_position[0];
    Position_Info.ang_l_1 = g_DXL_ID_position[2];
    Position_Info.ang_l_2 = g_DXL_ID_position[10];
    Position_Info.ang_l_3 = g_DXL_ID_position[12];
    Position_Info.ang_l_4 = g_DXL_ID_position[14];
    Position_Info.ang_l_5 = g_DXL_ID_position[16];
    Position_Info.ang_l_6 = g_DXL_ID_position[18];
    Position_Info.ang_l_7 = g_DXL_ID_position[20];
    Position_Info.ang_l_8 = g_DXL_ID_position[4];

    Position_Info.ang_r_0 = g_DXL_ID_position[1];
    Position_Info.ang_r_1 = g_DXL_ID_position[3];
    Position_Info.ang_r_2 = g_DXL_ID_position[11];
    Position_Info.ang_r_3 = g_DXL_ID_position[13];
    Position_Info.ang_r_4 = g_DXL_ID_position[15];
    Position_Info.ang_r_5 = g_DXL_ID_position[17];
    Position_Info.ang_r_6 = g_DXL_ID_position[19];
    Position_Info.ang_r_7 = g_DXL_ID_position[21];
    Position_Info.ang_r_8 = g_DXL_ID_position[5];

    Motor_Position_Pub.publish(Position_Info);





//    for(int DXL_ID = 0; DXL_ID <= 22; DXL_ID++)
//    {
//        cout<<"g_DXL_ID_position["<<DXL_ID<<"] = "<<(g_DXL_ID_position[DXL_ID]/4)<<endl;
//    }

//    cout<<"Now_Motor_Angle.Motor_Angle_10  "<<Now_Motor_Angle.Motor_Angle_10<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_11  "<<Now_Motor_Angle.Motor_Angle_11<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_12  "<<Now_Motor_Angle.Motor_Angle_12<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_13  "<<Now_Motor_Angle.Motor_Angle_13<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_14  "<<Now_Motor_Angle.Motor_Angle_14<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_15  "<<Now_Motor_Angle.Motor_Angle_15<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_16  "<<Now_Motor_Angle.Motor_Angle_16<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_17  "<<Now_Motor_Angle.Motor_Angle_17<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_18  "<<Now_Motor_Angle.Motor_Angle_18<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_19  "<<Now_Motor_Angle.Motor_Angle_19<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_20  "<<Now_Motor_Angle.Motor_Angle_20<<endl<<endl;
//    cout<<"Now_Motor_Angle.Motor_Angle_21  "<<Now_Motor_Angle.Motor_Angle_21<<endl<<endl;

//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_10  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_10<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_11  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_11<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_12  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_12<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_13  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_13<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_14  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_14<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_15  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_15<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_16  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_16<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_17  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_17<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_18  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_18<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_19  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_19<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_20  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_20<<endl<<endl;
//    cout<<"Now_Percentage_of_IK_Motor.Motor_Multiple_21  "<<Now_Percentage_of_IK_Motor.Motor_Multiple_21<<endl<<endl;


    if(body == All)
        motor_packet(All, 23, R_Rise_Condition, L_Rise_Condition, R_Rise_Max, L_Rise_Max);
    else if(body == Leg)
        motor_packet(Leg, 22, R_Rise_Condition, L_Rise_Condition, R_Rise_Max, L_Rise_Max);



    if(ikinit_flag)
    {
        cout<<endl<<"====  walk init ===="<<endl<<endl;
        for(int DXL_ID = 0; DXL_ID <= 22; DXL_ID++)
        {
            cout<< "g_DXL_ID_position[" << DXL_ID << "] = " << (g_DXL_ID_position[DXL_ID]/4) <<endl;
        }
        ikinit_flag = false;
    }
}

void IK_Solve::motor_packet(int body, int limit, double R_Rise_Condition, double L_Rise_Condition, double R_Rise_Max, double L_Rise_Max)
{
//    cout<<"Rise_Condition  "<<Rise_Condition<<endl;
//    cout<<"Rise_Max  "<<Rise_Max<<endl;
    msg_generate::Motor_msg Motor_Info;
    if(body == Leg)
    {
        for(int i = 0; i < 4; i++)
        {
            Motor_Info.id.push_back(i);
            int pos = g_DXL_ID_position[i];
            Motor_Info.position.push_back(pos);
            Motor_Info.speed.push_back(400);
        }
    }

    for(int i = body; i < limit; i++)
    {
        Motor_Info.id.push_back(i);
        int pos = g_DXL_ID_position[i];
        Motor_Info.position.push_back(pos);

        if(((L_Rise_Condition==0) && (R_Rise_Condition <= (R_Rise_Max*0.3)))||((R_Rise_Condition==0) && (L_Rise_Condition <= (L_Rise_Max*0.3))))//0.3 is Rise pattern's point of inflection
        {
//            if(L_Rise_Condition == 0)
//            {
//                cout<<"@@@@@@@@@@@@@@@@@@@@@@@@RMotor speed down@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl<<endl;
//                Motor_Info.speed.push_back(200*R_Rise_Condition/(R_Rise_Max*0.3) + 1);
//            }

//            else if(R_Rise_Condition == 0)
//            {
//                cout<<"@@@@@@@@@@@@@@@@@@@@@@@@LMotor speed down@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl<<endl;
//                Motor_Info.speed.push_back(200*L_Rise_Condition/(L_Rise_Max*0.3) + 1);
//            }
            Motor_Info.speed.push_back(0);
//            cout<<"@@@@@@@@@@@@@@@@@@@@@@@@Motor speed down@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl<<endl;
//            cout<<"R_Rise_Condition  "<<R_Rise_Condition<<endl;
//            cout<<"R_Rise_Max  "<<R_Rise_Max<<endl;
//            cout<<"L_Rise_Condition  "<<L_Rise_Condition<<endl;
//            cout<<"L_Rise_Max  "<<L_Rise_Max<<endl;
        }
        else
        {
            Motor_Info.speed.push_back(0);
//            cout<<"@@@@@@@@@@@@@@@@@@@@@@@@Motor speed up@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl<<endl;
//            cout<<"R_Rise_Condition  "<<R_Rise_Condition<<endl;
//            cout<<"R_Rise_Max  "<<R_Rise_Max<<endl;
//            cout<<"L_Rise_Condition  "<<L_Rise_Condition<<endl;
//            cout<<"L_Rise_Max  "<<L_Rise_Max<<endl;
        }
    }

    Motor_Info.length = Motor_Info.id.size();
    Motor_Info.mode = 3;


    Motor_Pub.publish(Motor_Info);
}

