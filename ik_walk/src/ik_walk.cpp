#include "../include/ik_walk/ik_walk.hpp"

using namespace std;
Robit_Humanoid_Walk Walk;
double data[FILTERDATA];
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_walk");
    ros::NodeHandle Robit;

    //SUBSCIRBE_MSG
    Imu_Sub = Robit.subscribe("imu",100,imuCallback);
    Tune2walk_Sub = Robit.subscribe("tune2walk",100,tune2walkCallback);
    Master2ik_Sub = Robit.subscribe("master2ik",100,master2ik_callback);
    Zmp_Sub = Robit.subscribe("zmp_msg",100,zmpcallback);

    //PUBLISH_MSG
    Ikend_Pub = Robit.advertise<msg_generate::ikend_msg>("ikend",100);
    IK.Motor_Pub = Robit.advertise<msg_generate::Motor_msg>("Dynamixel", 100);
    Ikcoordinate_Pub = Robit.advertise<msg_generate::ikcoordinate_msg>("ikcoordinate", 100);
    walk_pattern_Pub = Robit.advertise<msg_generate::walk_pattern>("walk_pattern",100);
    Com_Pub = Robit.advertise<msg_generate::com_msg>("COM",100);
    IK.Motor_Position_Pub = Robit.advertise<msg_generate::ik_angle_sim>("sim_position", 100);//simulation

    ros::Timer timer = Robit.createTimer(ros::Duration(0.01),timer_callback);
    IK.init_save();
    sleep(1);
    get_parameters();
    Walk.Generate_Pattern(Now_Param);
    IK.Balance_Control_Body_Upright(0,Now_Param.Z.Default_Z_Right,Init_Position_Time,Init_Rise_Condition,Init_Position_Pitch,Init_Position_Balance_Msg,Init_Position_Balance_Msg,0,0,0,0,0,Model_Data.Link2Link,0,0,99);
//    cout<<"Model_Data.Center2Leg  "<<Model_Data.Center2Leg<<endl<<endl;
//    cout<<"Model_Data.Link2Link  "<<Model_Data.Link2Link<<endl<<endl;
//    cout<<"Model_Data.Init_Z_Up  "<<Model_Data.Init_Z_Up<<endl<<endl;
//    cout<<"Now_Param.Y.Default_Y_Right       "<<Now_Param.Y.Default_Y_Right<<endl;
//    cout<<"Now_Param.Y.Default_Y_Left       "<<Now_Param.Y.Default_Y_Left<<endl;
//    cout<<"Now_Param.Z.Default_Z_Right       "<<Now_Param.Z.Default_Z_Right<<endl;
//    cout<<"Now_Param.Z.Default_Z_Left       "<<Now_Param.Z.Default_Z_Left<<endl;

    IK.solve(0.0, Now_Param.Y.Default_Y_Right,Now_Param.Z.Default_Z_Right, 0.0, 0.0,Now_Param.Y.Default_Y_Left,Now_Param.Z.Default_Z_Left, 0.0, All,0,0,0,0,0,0,0,0,0,0,0);
    sleep(1);

    while(ros::ok())// cycle
    {
        ros::spinOnce();
    }

    return 0;

}
void Robit_Humanoid_Walk::Walk_Start_End(Walk_Parameters &Now_Param)
{

    if(Past_Param.IK_Flag != Ik_Flag_Past)
    {
        if(Past_Param.IK_Flag)
        {
            Start_Flag = true;
            cout<<"Start_Flag"<<endl;
        }
        else if(!Past_Param.IK_Flag && Timer_Time==0)
        {
            End_Flag = true;
            cout<<"End_Flag"<<endl;
        }
    }



    if(Start_Flag)
    {
        IK.Now_Motor_Angle = IK.Past_Motor_Angle;
        IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;

        //        cout<<"Now_Param.Start_Entire_Time"<<Now_Param.Start_Entire_Time<<endl;
        //        cout<<"Accel : "<<Accel_Entire_Time<<endl;
        //        cout<<"TImer time start : "<<Timer_Time_Start<<endl;
        //        cout<<"Timer time : "<<Timer_Time<<endl;
        //        cout<<"startcnt : "<<Start_Cnt<<endl;
        if(Timer_Time_Start ==0.0)
        {
            Now_Param.IK_Flag = true;
            Start_Cnt++;
        }

        if(Start_Cnt >=Repeat_Start_Cycle)
        {
            Timer_Time = Timer_Time_Start;
            if(fabs(Timer_Time_Start-Timer_Time)<3)
            {

                Start_Cnt =1;
                Start_Flag = false;
                Ikend_Info.ikend = 0;
                Ikend_Pub.publish(Ikend_Info);
            }

            //Timer_Time_Start =0.0;
        }


    }

    else if(End_Flag )
    {
        if(Timer_Time_End ==0)
        {
            End_Cnt--;
            //cout<<"END_CNT : "<<End_Cnt<<endl;
        }
        if(End_Cnt<1 && Timer_Time_End==0)
        {
            //End_Check = true;
            End_Cnt=Repeat_End_Cycle;
            End_Flag=false;
            //Ready_To_Start = false;
            Now_Param.IK_Flag = false;
            Ik_Flag_Past = false;
            Timer_Time_End =0.0;




            IK.Now_Motor_Angle = IK.Past_Motor_Angle;
            IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;
            IK.solve(0.0, Now_Param.Y.Default_Y_Right,Now_Param.Z.Default_Z_Right, 0.0, 0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left, 0.0, Leg,0,0,0,0,0,0,0,0,0,0,0);
            Ikend_Info.ikend = 1;
            Ikend_Pub.publish(Ikend_Info);
        }
    }

    if(Now_Param.IK_Flag && (Start_Flag || End_Flag))
    {
        Result_Pattern(Now_Param);
        if(Start_Flag) End_Cnt = Repeat_End_Cycle;
        else if(End_Flag) Start_Cnt = 1;

    }
    else if(Now_Param.IK_Flag && (!Start_Flag && !End_Flag))
    {
        cout<<"normal"<<endl<<endl;
        Result_Pattern(Now_Param); 
    }


}
void Robit_Humanoid_Walk::Generate_Pattern(Walk_Parameters &Now_Param)
{

    ///////////////////////////Single_OR_Double_Pattern///////////////////////////

    if(Balancing_Param.Balance.Ratio_Check_Flag == 1)
    {

        ///////////////////////////Start_COM_Pattern///////////////////////////

        /////////////////////////////////////2021_01_04_IRC
//        Start_COM_Pattern.put_point(0.0,  0.0,  -16.0,  -0.0);
//        Start_COM_Pattern.put_point(0.125,  -1.0,  0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.375,  -1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.625,  1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.875,  1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(1.0,  0.0,  -16.0,  -0.0);

        //////////////////////////////////////2021_0203_Soc_1

//        Start_COM_Pattern.put_point(0.0,  0.0,  -16.0,  -0.0);
//        Start_COM_Pattern.put_point(0.125,  -1.0,  0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.375,  -1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.625,  1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.875,  1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(1.0,  0.0,  -16.0,  -0.0);

        //////////////////////////////////////2021_0203_Soc_2

//        Start_COM_Pattern.put_point(0.0,  0.0,  -12.0,  -0.0);
//        Start_COM_Pattern.put_point(0.125,  -1.0,  0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.25,  -1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.5,  0.0,  12.0,  -0.0);
//        Start_COM_Pattern.put_point(0.625,  1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.75,  1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(1.0,  0.0,  -12.0,  -0.0);

        //////////////////////////////////////2021_0203_Soc_2

//        Start_COM_Pattern.put_point(0.0,  0.0,  -8.0,  -0.0);
//        Start_COM_Pattern.put_point(0.25,  -1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.75,  1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(1.0,  0.0,  -8.0,  -0.0);

        //////////////////////////////////////2021_0713_Soc_2

        Start_COM_Pattern.put_point(0.0,  -1.0,  0.0,  -0.0);
        Start_COM_Pattern.put_point(0.5,  1.0,  -0.0,  -0.0);
        Start_COM_Pattern.put_point(1.0,  -1.0,  0.0,  -0.0);

        //////////////////////////////////////2021_0714_Soc_2

//        Start_COM_Pattern.put_point(0.0,  1.0,  0.0,  -0.0);
//        Start_COM_Pattern.put_point(0.5,  -1.0,  -0.0,  -0.0);
//        Start_COM_Pattern.put_point(1.0,  1.0,  0.0,  -0.0);

        ///////////////////////////Start_Rise_Pattern///////////////////////////

        ///////////////////////////////////////2021_01_04_IRC
//        Start_Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        Start_Rise_Pattern.put_point(0.625,  0.0,  0.0,  0.0);//
//        Start_Rise_Pattern.put_point(0.75,   1.0,    0.0,    0.0);
//        Start_Rise_Pattern.put_point(0.875,   0.0,    0.0,    0.0);//0.75
//        Start_Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

        /////////////////////////////////////2021_0203_Soc_1,2

//        Start_Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
//        Start_Rise_Pattern.put_point(0.5625, 0.0, 0.0, 0.0);//
//        Start_Rise_Pattern.put_point(0.65625, 0.3, 5.33, 0.0);
//        Start_Rise_Pattern.put_point(0.75, 1.0, 0.0, 0.0);//0.75
//        Start_Rise_Pattern.put_point(0.84375, 0.3, -5.33, 0.0);
//        Start_Rise_Pattern.put_point(0.9375, 0.0, 0.0, 0.0);
//        Start_Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

        /////////////////////////////////////2021_0713_Soc_1,2

        Start_Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
        Start_Rise_Pattern.put_point(0.3125, 0.0, 0.0, 0.0);//
        Start_Rise_Pattern.put_point(0.40625, 0.3, 5.33, 0.0);
        Start_Rise_Pattern.put_point(0.5, 1.0, 0.0, 0.0);//0.75
        Start_Rise_Pattern.put_point(0.59375, 0.3, -5.33, 0.0);
        Start_Rise_Pattern.put_point(0.6875, 0.0, 0.0, 0.0);
        Start_Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

        /////////////////////////////////////2021_0714_Soc_1,2

//        Start_Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
//        Start_Rise_Pattern.put_point(0.28125, 0.0, 0.0, 0.0);//
//        Start_Rise_Pattern.put_point(0.5, 1.0, 0.0, 0.0);//0.75
//        Start_Rise_Pattern.put_point(0.71875, 0.0, 0.0, 0.0);
//        Start_Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

        ///////////////////////////Normal///////////////////////////

        ///////////////////////////////////////2020,2021_01_04_IRC
//        Step_Pattern.put_point(0.0,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.25,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.5,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.75,0.5,0.0,0.0);
//        Step_Pattern.put_point(1.0,0.5,0.0,0.0);

//        Step_Pattern.put_point(0.0,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.125,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.375,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.625,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.875,0.5,0.0,0.0);
//        Step_Pattern.put_point(1.0,0.5,0.0,0.0);

        ///////////////////////////////////////2021_0203_Soc_1,2

//        Step_Pattern.put_point(0.0,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.0625,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.4375,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.5625,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.9375,0.5,0.0,0.0);
//        Step_Pattern.put_point(1.0,0.5,0.0,0.0);

        ///////////////////////////////////////2021_0713_Soc_1,2

        //Step_Pattern.put_point(0.0,0.0,-5.33,0.0);
        Step_Pattern.put_point(0.0,0.0,-4.0,0.0);
        Step_Pattern.put_point(0.25,-0.5,0.0,0.0);
        //Step_Pattern.put_point(0.1875,-0.5,0.0,0.0);
        //Step_Pattern.put_point(0.3125,-0.5,0.0,0.0);
        Step_Pattern.put_point(0.75,0.5,0.0,0.0);
        //Step_Pattern.put_point(0.6875,0.5,0.0,0.0);
        //Step_Pattern.put_point(0.8125,0.5,0.0,0.0);
        //Step_Pattern.put_point(1.0,0.0,-5.33,0.0);
        Step_Pattern.put_point(1.0,0.0,-4.0,0.0);

        ///////////////////////////////////////2021_0714_Soc_1,2

//        Step_Pattern.put_point(0.0,0.0,-4.57,0.0);
//        Step_Pattern.put_point(0.21875,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.28125,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.71875,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.78125,0.5,0.0,0.0);
//        Step_Pattern.put_point(1.0,0.0,-4.57,0.0);

        /////////////////////////////////////2021_01_04_IRC
//        Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        Rise_Pattern.put_point(0.5,  0.0,  0.0,  0.0);//
//        Rise_Pattern.put_point(0.625,   1.0,    0.0,    0.0);
//        Rise_Pattern.put_point(0.75,   0.0,    0.0,    0.0);//0.75
//        Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

//        Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        Rise_Pattern.put_point(0.625,  0.0,  0.0,  0.0);//
//        Rise_Pattern.put_point(0.75,   1.0,    0.0,    0.0);
//        Rise_Pattern.put_point(0.875,   0.0,    0.0,    0.0);//0.75
//        Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

        /////////////////////////////////////2021_0203_Soc_1,2   4:1

//        Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
//        Rise_Pattern.put_point(0.5625, 0.0, 0.0, 0.0);//
//        Rise_Pattern.put_point(0.65625, 0.3, 5.33, 0.0);
//        Rise_Pattern.put_point(0.75, 1.0, 0.0, 0.0);//0.75
//        Rise_Pattern.put_point(0.84375, 0.3, -5.33, 0.0);
//        Rise_Pattern.put_point(0.9375, 0.0, 0.0, 0.0);
//        Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

        /////////////////////////////////////2021_0713_Soc_1,2

        Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
        Rise_Pattern.put_point(0.3125, 0.0, 0.0, 0.0);//
        Rise_Pattern.put_point(0.40625, 0.3, 5.33, 0.0);
        Rise_Pattern.put_point(0.5, 1.0, 0.0, 0.0);//0.75
        Rise_Pattern.put_point(0.59375, 0.3, -5.33, 0.0);
        Rise_Pattern.put_point(0.6875, 0.0, 0.0, 0.0);
        Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

        /////////////////////////////////////2021_0714_Soc_1,2

//        Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
//        Rise_Pattern.put_point(0.28125, 0.0, 0.0, 0.0);//
//        Rise_Pattern.put_point(0.5, 1.0, 0.0, 0.0);//0.75
//        Rise_Pattern.put_point(0.71875, 0.0, 0.0, 0.0);
//        Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

        //////////////////////////////////////2021_01_04_IRC
//        COM_Pattern.put_point(0.0,  -1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.25,  -1.0,  0.0,  -0.0);
//        COM_Pattern.put_point(0.5,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.75,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  -1.0,  -0.0,  -0.0);

//        COM_Pattern.put_point(0.0,  0.0,  -16.0,  -0.0);
//        COM_Pattern.put_point(0.125,  -1.0,  0.0,  -0.0);
//        COM_Pattern.put_point(0.375,  -1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.625,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.875,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  0.0,  -16.0,  -0.0);

        //////////////////////////////////////2021_0203_Soc_1

//        COM_Pattern.put_point(0.0,  0.0,  -16.0,  -0.0);
//        COM_Pattern.put_point(0.125,  -1.0,  0.0,  -0.0);
//        COM_Pattern.put_point(0.375,  -1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.625,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.875,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  0.0,  -16.0,  -0.0);

        //////////////////////////////////////2021_0203_Soc_2

//        COM_Pattern.put_point(0.0,  0.0,  -12.0,  -0.0);
//        COM_Pattern.put_point(0.125,  -1.0,  0.0,  -0.0);
//        COM_Pattern.put_point(0.25,  -1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.5,  0.0,  12.0,  -0.0);
//        COM_Pattern.put_point(0.625,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.75,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  0.0,  -12.0,  -0.0);

        //////////////////////////////////////2021_0203_Soc_2

//        COM_Pattern.put_point(0.0,  0.0,  -8.0,  -0.0);
//        COM_Pattern.put_point(0.25,  -1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.75,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  0.0,  -8.0,  -0.0);

        //////////////////////////////////////2021_0713_Soc_2

        COM_Pattern.put_point(0.0,  -1.0,  0.0,  -0.0);
        COM_Pattern.put_point(0.5,  1.0,  -0.0,  -0.0);
        COM_Pattern.put_point(1.0,  -1.0,  0.0,  -0.0);

        //////////////////////////////////////2021_0714_Soc_2

//        COM_Pattern.put_point(0.0,  1.0,  0.0,  -0.0);
//        COM_Pattern.put_point(0.5,  -1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  1.0,  0.0,  -0.0);


        //////////////////////////////////////2021_01_04_IRC

//        Turn_Pattern.put_point(0.0,  0.5,  -8.0,  0.0);
//        Turn_Pattern.put_point(0.125,  0.0,  0.0,  0.0);
//        Turn_Pattern.put_point(0.625,  0.0,  0.0,  0.0);
//        Turn_Pattern.put_point(0.875,  1.0,  0.0,  0.0);
//        Turn_Pattern.put_point(1.0,  0.5,  -8.0,  0.0);

        //////////////////////////////////////2021_0203_Soc_1,2

//        Turn_Pattern.put_point(0.0,  0.83,  -5.33,  0.0);
//        Turn_Pattern.put_point(0.3125,  0.0,  0.0,  0.0);
//        Turn_Pattern.put_point(0.5625,  0.0,  0.0,  0.0);
//        Turn_Pattern.put_point(0.9375,  1.0,  0.0,  0.0);
//        Turn_Pattern.put_point(1.0,  0.83,  -5.33,  0.0);

        //////////////////////////////////////2021_0713_Soc_1,2

        //Turn_Pattern.put_point(0.0,  0.17,  -5.33,  0.0);
        Turn_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
        //Turn_Pattern.put_point(0.0625,  0.0,  0.0,  0.0);
        Turn_Pattern.put_point(0.3125,  0.0,  0.0,  0.0);
        Turn_Pattern.put_point(0.6875,  1.0,  0.0,  0.0);
        Turn_Pattern.put_point(1.0,  0.0,  0.0,  0.0);

        //////////////////////////////////////2021_0714_Soc_1,2

//        //Turn_Pattern.put_point(0.0,  0.17,  -5.33,  0.0);
//        Turn_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        //Turn_Pattern.put_point(0.0625,  0.0,  0.0,  0.0);
//        Turn_Pattern.put_point(0.28125,  0.0,  0.0,  0.0);
//        Turn_Pattern.put_point(0.71875,  1.0,  0.0,  0.0);
//        Turn_Pattern.put_point(1.0,  0.0,  0.0,  0.0);






    }
    else if(Balancing_Param.Balance.Ratio_Check_Flag ==0)
    {


        ///////////////////////////Start_COM_Pattern///////////////////////////

        /////////////////////////////////////2021_07_05_Soc
        Start_COM_Pattern.put_point(0.0,  0.0,  -8.0,  -0.0);
        Start_COM_Pattern.put_point(0.25,  -1.0,  -0.0,  -0.0);
        Start_COM_Pattern.put_point(0.75,  1.0,  -0.0,  -0.0);
        Start_COM_Pattern.put_point(1.0,  0.0,  -8.0,  -0.0);

        ///////////////////////////Start_Rise_Pattern///////////////////////////

        ///////////////////////////////////////2020
        Start_Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
        Start_Rise_Pattern.put_point(0.5, 0.0, 0.0, 0.0);//
        Start_Rise_Pattern.put_point(0.625, 0.3, 4.0, 0.0);
        Start_Rise_Pattern.put_point(0.75, 1.0, 0.0, 0.0);//0.75
        Start_Rise_Pattern.put_point(0.875, 0.3, -4.0, 0.0);
        Start_Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

        /////////////////////////////////////2021_01_21_Soc
//        Start_Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        Start_Rise_Pattern.put_point(0.3,  0.0,  0.0,  0.0);//
//        Start_Rise_Pattern.put_point(0.5,   1.0,    0.0,    0.0);
//        Start_Rise_Pattern.put_point(0.7,   0.0,    0.0,    0.0);
//        Start_Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

        ///////////////////////////Normal///////////////////////////

        ///////////////////////////////////////2020_IRC
//        Step_Pattern.put_point(0.0,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.25,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.5,-0.5,0.0,0.0);
//        Step_Pattern.put_point(0.75,0.5,0.0,0.0);
//        Step_Pattern.put_point(1.0,0.5,0.0,0.0);

        ///////////////////////////////////////2021_07_05_Soc
        Step_Pattern.put_point(0.0,0.5,0.0,0.0);
        Step_Pattern.put_point(0.0625,0.5,0.0,0.0);
        Step_Pattern.put_point(0.4375,-0.5,0.0,0.0);
        Step_Pattern.put_point(0.5625,-0.5,0.0,0.0);
        Step_Pattern.put_point(0.9375,0.5,0.0,0.0);
        Step_Pattern.put_point(1.0,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.0,-0.5,0.0,0.0);//old
//        Step_Pattern.put_point(0.25,0,0.0,0.0);
//        Step_Pattern.put_point(0.5,0.5,0.0,0.0);
//        Step_Pattern.put_point(0.75,0,0.0,0.0);
//        Step_Pattern.put_point(1.0,-0.5,0.0,0.0);

        //origin_single\\
        //4,7
//        Rise_Pattern.put_point(0.0,  0.0,  0.0,  1.0);
//        Rise_Pattern.put_point(0.4,  1.0,  0.0,  0.0);//
//        Rise_Pattern.put_point(0.7,   0.0,    0.0,    0.0);
//        Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

        ///////////////////////////////////////2020
//        Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        Rise_Pattern.put_point(0.25,  0.0,  0.0,  0.0);//
//        Rise_Pattern.put_point(0.65,   1.0,    0.0,    0.0);
//        Rise_Pattern.put_point(0.95,   0.0,    0.0,    0.0);
//        Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

//        /////////////////////////////////////2021_01_21_Soc
//        Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        Rise_Pattern.put_point(0.3,  0.0,  0.0,  0.0);//
//        Rise_Pattern.put_point(0.5,   1.0,    0.0,    0.0);
//        Rise_Pattern.put_point(0.7,   0.0,    -0.0,    0.0);
//        Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

//        Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);
//        Rise_Pattern.put_point(0.25,  0.0,  0.0,  0.0);//
//        Rise_Pattern.put_point(0.5,   1.0,    0.0,    0.0);
//        Rise_Pattern.put_point(0.75,   0.0,    0.0,    0.0);
//        Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

        /////////////////////////////////////2021_07_05_Soc

        Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
        Rise_Pattern.put_point(0.55, 0.0, 0.0, 0.0);//
        Rise_Pattern.put_point(0.625, 0.3, 4.0, 0.0);
        Rise_Pattern.put_point(0.75, 1.0, 0.0, 0.0);//0.75
        Rise_Pattern.put_point(0.875, 0.3, -4.0, 0.0);
        Rise_Pattern.put_point(0.95, 0.0, 0.0, 0.0);
        Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

//        Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
//        Rise_Pattern.put_point(0.2, 0.0, 0.0, 0.0);//
//        Rise_Pattern.put_point(0.5, 1.0, 0.0, 0.0);
//        Rise_Pattern.put_point(0.8, 0.0, 0.0, 0.0);//0.75
//        Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

//        Rise_Pattern.put_point(0.0,  0.0,  0.0,  0.0);//old
//        Rise_Pattern.put_point(0.3,  1.0,  0.0,  0.0);
//        Rise_Pattern.put_point(0.6,   0.0,    0.0,    0.0);
//        Rise_Pattern.put_point(1.0,   0.0,    0.0,    0.0);

        //////////////////////////////////////2021_07_05_Soc

        COM_Pattern.put_point(0.0,  0.0,  -8.0,  -0.0);
        COM_Pattern.put_point(0.25,  -1.0,  -0.0,  -0.0);
        COM_Pattern.put_point(0.75,  1.0,  -0.0,  -0.0);
        COM_Pattern.put_point(1.0,  0.0,  -8.0,  -0.0);

//        COM_Pattern.put_point(0.0,  -1.0,  0.0,  -0.0);
//        COM_Pattern.put_point(0.5,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  -1.0,  0.0,  -0.0);

//        COM_Pattern.put_point(0.0,  0.0,  8.0,  -0.0);//old
//        COM_Pattern.put_point(0.25,  1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(0.75,  -1.0,  -0.0,  -0.0);
//        COM_Pattern.put_point(1.0,  0.0,  8.0,  -0.0);

        /////////////////////////////////////2021_07_05_Soc
        Turn_Pattern.put_point(0.0,  0.83,  -5.33,  0.0);
        Turn_Pattern.put_point(0.3125,  0.0,  0.0,  0.0);
        Turn_Pattern.put_point(0.5625,  0.0,  0.0,  0.0);
        Turn_Pattern.put_point(0.9375,  1.0,  0.0,  0.0);
        Turn_Pattern.put_point(1.0,  0.83,  -2.67,  0.0);
//        Turn_Pattern.put_point(0.0,  0.0,  0.0,  0.0);//old
//        Turn_Pattern.put_point(0.5,  1.0,  0.0,  0.0);
//        Turn_Pattern.put_point(1.0,  0.0,  0.0,  0.0);
    }




}
void Robit_Humanoid_Walk::Result_Pattern(Walk_Parameters &Now_Param)
{
    Time_Right_Leg_Start = Timer_Time_Start/Now_Param.Sink_Entire_Time;
    Time_Left_Leg_Start = Timer_Time_Start/Now_Param.Sink_Entire_Time+0.5;

    Time_Right_Leg = Timer_Time/Now_Param.Entire_Time;
    Time_Left_Leg  = Timer_Time/Now_Param.Entire_Time+0.5;

    Time_Right_Leg_End = Timer_Time_End/Now_Param.End_Entire_Time;
    Time_Left_Leg_End = Timer_Time_End/Now_Param.End_Entire_Time+0.5;
    if(Time_Left_Leg_Start >= 1.0)  Time_Left_Leg_Start -= 1.0;
    if(Time_Left_Leg >= 1.0)        Time_Left_Leg -= 1.0;
    if(Time_Left_Leg_End >= 1.0)    Time_Left_Leg_End -= 1.0;


    if(Balancing_Param.Balance.Ratio_Check_Flag == 1)
    {
        Past_Param.Check_ratio.Ratio_Flag = 1;
        cout<<"Past_Param.Check_ratio.Ratio_Flag     "<<Past_Param.Check_ratio.Ratio_Flag<<endl<<endl;
    }
    else if(Balancing_Param.Balance.Ratio_Check_Flag == 0)
    {
        Past_Param.Check_ratio.Ratio_Flag = 0;
        cout<<"Past_Param.Check_ratio.Ratio_Flag     "<<Past_Param.Check_ratio.Ratio_Flag<<endl<<endl;
    }


    if(Balancing_Param.Balance.Balance_Roll_Flag_imu && Imu_Info.roll <= Balancing_Param.Balance.Balance_Roll_Neg_Target_imu)
    {
//        Now_Param.Z.Rise_Right_Leg = 0.0;
//        Now_Param.Z.Rise_Left_Leg = 0.0;
//        Now_Param.Y.Swing_Right_Leg = 0.0;
//        Now_Param.Y.Swing_Left_Leg = 0.0;
//        Now_Param.X.X = 0.0;
//        Now_Param.X.Tuning_X = 0.0;
//        Now_Param.X.Default_X_Left = 0.0;
//        Now_Param.X.Default_X_Right = 0.0;
//        Now_Param.Yaw_R.Yaw = 0;
//        Now_Param.Yaw_R.Tuning_Yaw = 0;
//        Now_Param.Yaw_L.Yaw = 0;
//        Now_Param.Yaw_L.Tuning_Yaw = 0;
//        Compansate_Flag_L = true;
//        Compansate_Flag_R = false;
//        cout<<"Compansate_Flag_L             "<<Compansate_Flag_L<<endl;
//        cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<endl;

    }
    else if(Balancing_Param.Balance.Balance_Roll_Flag_imu && Imu_Info.roll >= Balancing_Param.Balance.Balance_Roll_Pos_Target_imu)
    {
//        Now_Param.Z.Rise_Right_Leg = 0.0;
//        Now_Param.Z.Rise_Left_Leg = 0.0;
//        Now_Param.Y.Swing_Right_Leg = 0.0;
//        Now_Param.Y.Swing_Left_Leg = 0.0;
//        Now_Param.X.X = 0.0;
//        Now_Param.X.Tuning_X = 0.0;
//        Now_Param.X.Default_X_Left = 0.0;
//        Now_Param.X.Default_X_Right = 0.0;
//        Now_Param.Yaw_R.Yaw = 0;
//        Now_Param.Yaw_R.Tuning_Yaw = 0;
//        Now_Param.Yaw_L.Yaw = 0;
//        Now_Param.Yaw_L.Tuning_Yaw = 0;
//        Compansate_Flag_R = true;
//        Compansate_Flag_L = false;
//        cout<<"Compansate_Flag_R               "<<Compansate_Flag_R<<endl;
//        cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"<<endl;

    }




    if(Time_Right_Leg == 0 || Time_Left_Leg == 0)
    {
        Now_Param.Check_ratio = Past_Param.Check_ratio;
//        Now_Param.Y.Swing_Right_Leg = Past_Param.Y.Swing_Right_Leg;
//        Now_Param.Y.Swing_Left_Leg = Past_Param.Y.Swing_Left_Leg;
        Now_Param.Y.Default_Y_Right = Past_Param.Y.Default_Y_Right;
        Now_Param.Y.Default_Y_Left = Past_Param.Y.Default_Y_Left;

        /////////////////////////////////////
//        Now_Param.X = Past_Param.X;

        Now_Param.Z.Default_Z_Right = Past_Param.Z.Default_Z_Right;
        Now_Param.Z.Default_Z_Left= Past_Param.Z.Default_Z_Left;
//        Now_Param.Z.Rise_Right_Leg= Past_Param.Z.Rise_Right_Leg;
//        Now_Param.Z.Rise_Left_Leg = Past_Param.Z.Rise_Left_Leg;


        if(Compansate_Flag_L && (Imu_Info.roll >= Balancing_Param.Balance.Balance_Roll_Neg_Target_imu && Imu_Info.roll <= Balancing_Param.Balance.Balance_Roll_Pos_Target_imu))
        {
//            Now_Param.Y.Swing_Right_Leg = Past_Param.Y.Swing_Right_Leg;
//            Now_Param.Y.Swing_Left_Leg = Past_Param.Y.Swing_Left_Leg;
//            Now_Param.Z.Rise_Right_Leg= Past_Param.Z.Rise_Right_Leg;
//            Now_Param.Z.Rise_Left_Leg = Past_Param.Z.Rise_Left_Leg;
//            Now_Param.X = Past_Param.X;
//            Now_Param.Yaw_R = Past_Param.Yaw_R;
//            Now_Param.Yaw_L = Past_Param.Yaw_L;
//            Compansate_Flag_2 = true;
//            Compansate_Cnt++;

//            if(Compansate_Cnt >= Repeat_Compansate_Cycle)
//            {
//                Timer_Time = Timer_Time_Start;
//                if(fabs(Timer_Time_Start-Timer_Time)<3)
//                {

//                    Compansate_Cnt =1;
//                    Compansate_Flag_L = false;
//                    Compansate_Flag_2 = false;
//                }

//            }
        }

        else if(Compansate_Flag_R && (Imu_Info.roll >= Balancing_Param.Balance.Balance_Roll_Neg_Target_imu && Imu_Info.roll <= Balancing_Param.Balance.Balance_Roll_Pos_Target_imu))
        {
//            Now_Param.Y.Swing_Right_Leg = Past_Param.Y.Swing_Right_Leg;
//            Now_Param.Y.Swing_Left_Leg = Past_Param.Y.Swing_Left_Leg;
//            Now_Param.Z.Rise_Right_Leg= Past_Param.Z.Rise_Right_Leg;
//            Now_Param.Z.Rise_Left_Leg = Past_Param.Z.Rise_Left_Leg;
//            Now_Param.X = Past_Param.X;
//            Now_Param.Yaw_R = Past_Param.Yaw_R;
//            Now_Param.Yaw_L = Past_Param.Yaw_L;
//            Compansate_Flag_2 = true;
//            Compansate_Cnt++;

//            if(Compansate_Cnt >= Repeat_Compansate_Cycle)
//            {
//                Timer_Time = Timer_Time_Start;
//                if(fabs(Timer_Time_Start-Timer_Time)<3)
//                {

//                    Compansate_Cnt =1;
//                    Compansate_Flag_R = false;
//                    Compansate_Flag_2 = false;
//                }

//            }
        }


        else
        {
            Now_Param.Y.Swing_Right_Leg = Past_Param.Y.Swing_Right_Leg;
            Now_Param.Y.Swing_Left_Leg = Past_Param.Y.Swing_Left_Leg;
            Now_Param.Z.Rise_Right_Leg= Past_Param.Z.Rise_Right_Leg;
            Now_Param.Z.Rise_Left_Leg = Past_Param.Z.Rise_Left_Leg;
            Now_Param.X = Past_Param.X;
            Now_Param.Yaw_R = Past_Param.Yaw_R;
            Now_Param.Yaw_L = Past_Param.Yaw_L;
            Compansate_Flag_L = false;
            Compansate_Flag_R = false;
            Compansate_Flag_2 = false;
        }

//        Now_Param.Yaw_R = Past_Param.Yaw_R;
//        Now_Param.Yaw_L = Past_Param.Yaw_L;

        //////////////////////////////

        Now_Param.Entire_Time = Past_Param.Entire_Time;
        Now_Param.Shoulder.Swing_Right_Shoulder = Past_Param.Shoulder.Swing_Right_Shoulder;
        Now_Param.Shoulder.Swing_Left_Shoulder = Past_Param.Shoulder.Swing_Left_Shoulder;
        Now_Param.Frequency = Past_Param.Frequency;
        Now_Param.Check_ratio = Past_Param.Check_ratio;

        if(End_Flag)
        {
            Now_Param.Entire_Time = Now_Param.End_Entire_Time;
        }




        if(Now_Param.Yaw_R.Yaw != Past_Param.Yaw_R.Yaw)
        {
            Temp_Param_Yaw_R = Now_Param.Yaw_R.Yaw;
            Yaw_R_Change = true;
        }
        else
        {
            Yaw_R_Change = false;
            Time_Yaw_R=0.0;
        }

        Now_Param.Yaw_R.Tuning_Yaw = Past_Param.Yaw_R.Tuning_Yaw;

        //        if(End_Cnt==Repeat_End_Cycle)
        //        {
        //            Now_Param.Entire_Time = Now_Param.End_Entire_Time;
        //        }

        //else//when if robot is moving,sending coordinate message
        //{

        //}
    }
    else if(fabs(Time_Right_Leg - 0.5) <= 0.02)
    {
        if(Now_Param.Yaw_L.Yaw != Past_Param.Yaw_L.Yaw)
        {
            Temp_Param_Yaw_L = Now_Param.Yaw_L.Yaw;
            Yaw_L_Change = true;
        }
        else
        {
            Yaw_L_Change = false;
            Time_Yaw_L = 0.0;
        }

        Now_Param.Yaw_L.Tuning_Yaw = Past_Param.Yaw_L.Tuning_Yaw;
    }
    else if(fabs(Time_Right_Leg - 0.25) <= 0.02 || fabs(Time_Right_Leg - 0.75) <= 0.02)
    {
        Now_Param.Y.Tuning_Side = Past_Param.Y.Tuning_Side;
        Now_Param.Y.Swing_Side_Right = Past_Param.Y.Swing_Side_Right;
        Now_Param.Y.Swing_Side_Left = Past_Param.Y.Swing_Side_Left;

        Now_Param.X.Tuning_X = Past_Param.X.Tuning_X;
        Now_Param.X.Default_X_Right = Past_Param.X.Default_X_Right;
        Now_Param.X.Default_X_Left = Past_Param.X.Default_X_Left;

        if(Now_Param.Y.Side != Past_Param.Y.Side)
        {
            Temp_Param_Side = Now_Param.Y.Side;
            Side_Change = true;
        }
        else
        {
            Side_Change = false;
            Time_Side = 0.0;
        }

        if(Now_Param.X.X != Past_Param.X.X)
        {
            Temp_Param_X = Now_Param.X.X;
            X_Change = true;
        }
        else
        {
            X_Change = false;
            Time_X = 0.0;
        }

        if(End_Cnt > 1 && End_Flag)
        {
            Now_Param.Y.Side=0.0;
            Now_Param.X.X = 0.0;
            Now_Param.Yaw_R.Yaw = 0.0;
            Now_Param.Yaw_L.Yaw = 0.0;
            Side_Change = false;
            X_Change = false;
            Yaw_L_Change = false;
            Yaw_R_Change = false;
        }
    }

    if(Side_Change)
    {
        Time_Side++;

        foot_trajectory Param_Pattern;
        Param_Pattern.put_point(0.0, Temp_Param_Side, 0.0, 0.0);
        Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.Y.Side, 0.0, 0.0);

        Now_Param.Y.Side = Param_Pattern.result(Time_Side);
        if(Now_Param.Y.Side < 0.1 && Now_Param.Y.Side > -0.1) Now_Param.Y.Side = 0;

        if(Time_Side >= Now_Param.Entire_Time) Time_Side = 0.0;
    }
    if(X_Change)
    {
        Time_X++;

        foot_trajectory Param_Pattern;
        Param_Pattern.put_point(0.0, Temp_Param_X, 0.0, 0.0);
        Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.X.X, 0.0, 0.0);

        Now_Param.X.X = Param_Pattern.result(Time_X);
        if(Now_Param.X.X < 0.1 && Now_Param.X.X > -0.1) Now_Param.X.X = 0;

        if(Time_X >= Now_Param.Entire_Time) Time_X = 0.0;
    }
    if(Yaw_L_Change)
    {
        Time_Yaw_L++;
        foot_trajectory Param_Pattern;
        Param_Pattern.put_point(0.0, Temp_Param_Yaw_L, 0.0, 0.0);
        Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.Yaw_L.Yaw, 0.0, 0.0);

        Now_Param.Yaw_L.Yaw = Param_Pattern.result(Time_Yaw_L);
        if(Now_Param.Yaw_L.Yaw < 0.1 && Now_Param.Yaw_L.Yaw > -0.1) Now_Param.Yaw_L.Yaw = 0;

        if(Time_Yaw_L >= Now_Param.Entire_Time) Time_Yaw_L = 0.0;
    }
    if(Yaw_R_Change)
    {
        Time_Yaw_R++;
        foot_trajectory Param_Pattern;
        Param_Pattern.put_point(0.0, Temp_Param_Yaw_R, 0.0, 0.0);
        Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.Yaw_R.Yaw, 0.0, 0.0);

        Now_Param.Yaw_R.Yaw = Param_Pattern.result(Time_Yaw_R);
        if(Now_Param.Yaw_R.Yaw < 0.1 && Now_Param.Yaw_R.Yaw > -0.1) Now_Param.Yaw_R.Yaw = 0;

        if(Time_Yaw_R >= Now_Param.Entire_Time) Time_Yaw_R = 0.0;
    }

    if(Start_Flag)
    {


        Accel_Rise_R = ((Now_Param.Z.Rise_Right_Leg-Now_Param.Z.Start_Rise)/Repeat_Start_Cycle)*Start_Cnt;
        Accel_Rise_L = ((Now_Param.Z.Rise_Left_Leg-Now_Param.Z.Start_Rise)/Repeat_Start_Cycle)*Start_Cnt;
        Accel_Swing_R = ((Now_Param.Y.Swing_Right_Leg-Now_Param.Y.Start_Swing)/Repeat_Start_Cycle)*Start_Cnt;
        Accel_Swing_L = ((Now_Param.Y.Swing_Left_Leg-Now_Param.Y.Start_Swing)/Repeat_Start_Cycle)*Start_Cnt;

        Accel_Entire_Time = ((Now_Param.Entire_Time-Now_Param.Start_Entire_Time)/Repeat_Start_Cycle)*Start_Cnt;

        Now_Param.Sink_Entire_Time = Now_Param.Start_Entire_Time+Accel_Entire_Time;


        if(Accel_Rise_R>fabs(Now_Param.Z.Rise_Right_Leg-Now_Param.Z.Start_Rise)) Accel_Rise_R=fabs(Now_Param.Z.Rise_Right_Leg-Now_Param.Z.Start_Rise);
        if(Accel_Rise_L>fabs(Now_Param.Z.Rise_Left_Leg-Now_Param.Z.Start_Rise))  Accel_Rise_L=fabs(Now_Param.Z.Rise_Left_Leg-Now_Param.Z.Start_Rise);
        if(Accel_Swing_R>fabs(Now_Param.Y.Swing_Right_Leg-Now_Param.Y.Start_Swing)) Accel_Swing_R=fabs(Now_Param.Y.Swing_Right_Leg-Now_Param.Y.Start_Swing);
        if(Accel_Swing_L>fabs(Now_Param.Y.Swing_Left_Leg-Now_Param.Y.Start_Swing))  Accel_Swing_L=fabs(Now_Param.Y.Swing_Left_Leg-Now_Param.Y.Start_Swing);

        if(Accel_Entire_Time>fabs(Now_Param.Entire_Time-Now_Param.Start_Entire_Time)) Accel_Entire_Time = fabs(Now_Param.Entire_Time-Now_Param.Start_Entire_Time);


        cout<<endl<<"**************Start_Walking*****************"<<endl<<endl;
        //        cout<<"Now_Param.Entire_Time : "<<Now_Param.Entire_Time<<endl;
        cout<<"Now_Param.Sink_Entire_Time : "<<Now_Param.Sink_Entire_Time<<endl;
        //        cout<<"Accel_Entire_Time"<<Accel_Entire_Time<<endl;



        //        cout<<"Accel_Rise_R : "<<Accel_Rise_R<<endl;
        //        cout<<"Accel_Rise_L : "<<Accel_Rise_L<<endl;

        //        cout<<"N_R : "<<Now_Param.Z.Rise_Right_Leg<<endl;
        //        cout<<"S_R : "<<Now_Param.Z.Start_Rise<<endl;

        cout<<"Accel_Swing_R : "<<Accel_Swing_R<<endl;
        cout<<"Accel_Swing_L : "<<Accel_Swing_L<<endl;



        Smooth_Flag = true;

        //RIGHT--------------------------------------------------------------------------------------
        Kinetic_X_R = Step_Pattern.result(Time_Right_Leg_Start)*(Now_Param.X.Tuning_X) + Now_Param.X.Default_X_Right;

        Kinetic_Y_R = -Start_COM_Pattern.result(Time_Right_Leg_Start)*(Now_Param.Y.Start_Swing+Accel_Swing_R)+Now_Param.Y.Default_Y_Right;

        Kinetic_Z_R = Start_Rise_Pattern.result(Time_Right_Leg_Start)*(Now_Param.Z.Start_Rise+Accel_Rise_R)+Now_Param.Z.Default_Z_Right;

        Kinetic_Yaw_R = Turn_Pattern.result(Time_Right_Leg)*(Now_Param.Yaw_R.Tuning_Yaw);

        //LEFT--------------------------------------------------------------------------------------
        Kinetic_X_L = Step_Pattern.result(Time_Left_Leg_Start)*(Now_Param.X.Tuning_X) + Now_Param.X.Default_X_Left;

        Kinetic_Y_L = Start_COM_Pattern.result(Time_Left_Leg_Start)*(Now_Param.Y.Start_Swing+Accel_Swing_L) + Now_Param.Y.Default_Y_Left;

        Kinetic_Z_L = Start_Rise_Pattern.result(Time_Left_Leg_Start)*(Now_Param.Z.Start_Rise+Accel_Rise_L) + Now_Param.Z.Default_Z_Left;

        Kinetic_Yaw_L = Turn_Pattern.result(Time_Left_Leg)*(Now_Param.Yaw_L.Tuning_Yaw);

    }

    else if(Compansate_Flag_2)
    {


        Accel_Rise_R = (fabs(Now_Param.Z.Rise_Right_Leg-Now_Param.Z.Start_Rise)/Repeat_Compansate_Cycle)*Compansate_Cnt;
        Accel_Rise_L = (fabs(Now_Param.Z.Rise_Left_Leg-Now_Param.Z.Start_Rise)/Repeat_Compansate_Cycle)*Compansate_Cnt;
        Accel_Swing_R = (fabs(Now_Param.Y.Swing_Right_Leg-Now_Param.Y.Start_Swing)/Repeat_Compansate_Cycle)*Compansate_Cnt;
        Accel_Swing_L = (fabs(Now_Param.Y.Swing_Left_Leg-Now_Param.Y.Start_Swing)/Repeat_Compansate_Cycle)*Compansate_Cnt;

        Accel_Entire_Time = (fabs(Now_Param.Entire_Time-Now_Param.Start_Entire_Time)/Repeat_Compansate_Cycle)*Compansate_Cnt;

        Now_Param.Sink_Entire_Time = Now_Param.Start_Entire_Time+Accel_Entire_Time;


        if(Accel_Rise_R>fabs(Now_Param.Z.Rise_Right_Leg-Now_Param.Z.Start_Rise)) Accel_Rise_R=fabs(Now_Param.Z.Rise_Right_Leg-Now_Param.Z.Start_Rise);
        if(Accel_Rise_L>fabs(Now_Param.Z.Rise_Left_Leg-Now_Param.Z.Start_Rise))  Accel_Rise_L=fabs(Now_Param.Z.Rise_Left_Leg-Now_Param.Z.Start_Rise);
        if(Accel_Swing_R>fabs(Now_Param.Y.Swing_Right_Leg-Now_Param.Y.Start_Swing)) Accel_Swing_R=fabs(Now_Param.Y.Swing_Right_Leg-Now_Param.Y.Start_Swing);
        if(Accel_Swing_L>fabs(Now_Param.Y.Swing_Left_Leg-Now_Param.Y.Start_Swing))  Accel_Swing_L=fabs(Now_Param.Y.Swing_Left_Leg-Now_Param.Y.Start_Swing);

        if(Accel_Entire_Time>fabs(Now_Param.Entire_Time-Now_Param.Start_Entire_Time)) Accel_Entire_Time = fabs(Now_Param.Entire_Time-Now_Param.Start_Entire_Time);


        cout<<endl<<"**************Compansate_Walking*****************"<<endl<<endl;
        //        cout<<"Now_Param.Entire_Time : "<<Now_Param.Entire_Time<<endl;
        //        cout<<"Now_Param.Sink_Entire_Time : "<<Now_Param.Sink_Entire_Time<<endl;
        //        cout<<"Accel_Entire_Time"<<Accel_Entire_Time<<endl;



        //        cout<<"Accel_Rise_R : "<<Accel_Rise_R<<endl;
        //        cout<<"Accel_Rise_L : "<<Accel_Rise_L<<endl;

        //        cout<<"N_R : "<<Now_Param.Z.Rise_Right_Leg<<endl;
        //        cout<<"S_R : "<<Now_Param.Z.Start_Rise<<endl;

        //        cout<<"Accel_Swing_R : "<<Accel_Swing_R<<endl;
        //        cout<<"Accel_Swing_L : "<<Accel_Swing_L<<endl;



        Smooth_Flag = true;

        //RIGHT--------------------------------------------------------------------------------------
        Kinetic_X_R = Step_Pattern.result(Time_Right_Leg_Start)*(Now_Param.X.Tuning_X) + Now_Param.X.Default_X_Right;
        Amount_of_Change_X = Kinetic_X_R - Past_Kinetic_X_R;
        Past_Kinetic_X_R = Kinetic_X_R;

        Kinetic_Y_R = -Start_COM_Pattern.result(Time_Right_Leg_Start)*(Now_Param.Y.Start_Swing+Accel_Swing_R)+Now_Param.Y.Default_Y_Right;
        Amount_of_Change_Y = Kinetic_Y_R - Past_Kinetic_Y_R;
        Past_Kinetic_Y_R = Kinetic_Y_R;

        Kinetic_Z_R = Start_Rise_Pattern.result(Time_Right_Leg_Start)*(Now_Param.Z.Start_Rise+Accel_Rise_R)+Now_Param.Z.Default_Z_Right;

        Kinetic_Yaw_R = Turn_Pattern.result(Time_Right_Leg)*(Now_Param.Yaw_R.Tuning_Yaw);

        //LEFT--------------------------------------------------------------------------------------
        Kinetic_X_L = Step_Pattern.result(Time_Left_Leg_Start)*(Now_Param.X.Tuning_X) + Now_Param.X.Default_X_Left;

        Kinetic_Y_L = Start_COM_Pattern.result(Time_Left_Leg_Start)*(Now_Param.Y.Start_Swing+Accel_Swing_L) + Now_Param.Y.Default_Y_Left;

        Kinetic_Z_L = Start_Rise_Pattern.result(Time_Left_Leg_Start)*(Now_Param.Z.Start_Rise+Accel_Rise_L) + Now_Param.Z.Default_Z_Left;

        Kinetic_Yaw_L = Turn_Pattern.result(Time_Left_Leg)*(Now_Param.Yaw_L.Tuning_Yaw);

    }

    else if(End_Flag)
    {
        cout<<endl<<"**************End_Walking*****************"<<endl<<endl;
        //RIGHT-------------------------------------------------------------------------------------
        Kinetic_X_R = Now_Param.X.Default_X_Right;
        Amount_of_Change_X = Kinetic_X_R - Past_Kinetic_X_R;
        Past_Kinetic_X_R = Kinetic_X_R;

        Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg_End)*(Now_Param.Y.End_Swing)+Now_Param.Y.Default_Y_Right;
        Amount_of_Change_Y = Kinetic_Y_R - Past_Kinetic_Y_R;
        Past_Kinetic_Y_R = Kinetic_Y_R;

        Kinetic_Z_R = Rise_Pattern.result(Time_Right_Leg_End)*(Now_Param.Z.End_Rise)+Now_Param.Z.Default_Z_Right;

        Kinetic_Yaw_R = 0.0;

        //LEFT--------------------------------------------------------------------------------------
        Kinetic_X_L = Now_Param.X.Default_X_Left;

        Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg_End)*(Now_Param.Y.End_Swing) + Now_Param.Y.Default_Y_Left;

        Kinetic_Z_L = Rise_Pattern.result(Time_Left_Leg_End)*(Now_Param.Z.End_Rise) + Now_Param.Z.Default_Z_Left;

        Kinetic_Yaw_L = 0.0;
    }
    else
    {
        cout<<endl<<"**************Normal_Walking*****************"<<endl<<endl;
        cout<<"Time_Right_Leg"<<Time_Right_Leg<<endl;
        cout<<"Kinetic_Z_R"<<Kinetic_Z_R<<endl;
        //cout<<"Accel_Entire_Time"<<Accel_Entire_Time<<endl;

        //RIGHT-------------------------------------------------------------------------------------
        Kinetic_X_R = Step_Pattern.result(Time_Right_Leg)*(Now_Param.X.Tuning_X + Now_Param.X.X /*+ Past_Param.X.Compansate_X*/) + Now_Param.X.Default_X_Right;
        Amount_of_Change_X = Kinetic_X_R - Past_Kinetic_X_R;
        Past_Kinetic_X_R = Kinetic_X_R;

        if(Now_Param.Y.Side < 0.0)
        {
            //RIGHT-------------------------------------------------------------------------------------
            Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg)*(Now_Param.Y.Swing_Right_Leg - (Now_Param.Y.Tuning_Side + Now_Param.Y.Side)*0.3 - (negative_position(Now_Param.X.X*0.05) + positive_position(Now_Param.X.X*0.05))) + Now_Param.Y.Default_Y_Right;

            Kinetic_Y_R += ((Now_Param.Y.Tuning_Side+ Now_Param.Y.Side )*Step_Pattern.result(Time_Right_Leg));
            //LEFT--------------------------------------------------------------------------------------
            Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg)*(Now_Param.Y.Swing_Left_Leg + (Now_Param.Y.Tuning_Side + Now_Param.Y.Side)*0.15 - (negative_position(Now_Param.X.X*0.05) + positive_position(Now_Param.X.X*0.05))) + Now_Param.Y.Default_Y_Left;

            Kinetic_Y_L += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side)*Step_Pattern.result(Time_Left_Leg) );
        }

        else if(Now_Param.Y.Side >= 0.0)
        {
            //RIGHT-------------------------------------------------------------------------------------
            Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg)*(Now_Param.Y.Swing_Right_Leg - (Now_Param.Y.Tuning_Side + Now_Param.Y.Side)*0.3 - (negative_position(Now_Param.X.X*0.05) + positive_position(Now_Param.X.X*0.012))) + Now_Param.Y.Default_Y_Right;

            Kinetic_Y_R += ((Now_Param.Y.Tuning_Side+ Now_Param.Y.Side )*Step_Pattern.result(Time_Right_Leg));
            //LEFT--------------------------------------------------------------------------------------
            Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg)*(Now_Param.Y.Swing_Left_Leg + (Now_Param.Y.Tuning_Side + Now_Param.Y.Side)*0.3 - (negative_position(Now_Param.X.X*0.05) + positive_position(Now_Param.X.X*0.012))) + Now_Param.Y.Default_Y_Left;

            Kinetic_Y_L += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side)*Step_Pattern.result(Time_Left_Leg) );
        }

        Amount_of_Change_Y = Kinetic_Y_R - Past_Kinetic_Y_R;
        Past_Kinetic_Y_R = Kinetic_Y_R;

        Kinetic_Z_R = Rise_Pattern.result(Time_Right_Leg)*(Now_Param.Z.Rise_Right_Leg /*+ Past_Param.Z.Compansate_Z*/) + Now_Param.Z.Default_Z_Right;

        Kinetic_Yaw_R = Turn_Pattern.result(Time_Right_Leg)*(Now_Param.Yaw_R.Tuning_Yaw+ Now_Param.Yaw_R.Yaw);

        Kinetic_Shoulder_X_R = Step_Pattern.result(Time_Right_Leg)*(Now_Param.Shoulder.Swing_Right_Shoulder /*-3.0*Imu_Balance.Pitch_ADD_Angle*/);

        Kinetic_Shoulder_Y_R = Step_Pattern.result(Time_Right_Leg)*(Now_Param.Y.Side);
        //LEFT--------------------------------------------------------------------------------------
        Kinetic_X_L = Step_Pattern.result(Time_Left_Leg)*(Now_Param.X.Tuning_X + Now_Param.X.X /*+ Past_Param.X.Compansate_X*/) + Now_Param.X.Default_X_Left;

        Kinetic_Z_L = Rise_Pattern.result(Time_Left_Leg)*(Now_Param.Z.Rise_Left_Leg /*+ Past_Param.Z.Compansate_Z*/) + Now_Param.Z.Default_Z_Left;

        Kinetic_Yaw_L = Turn_Pattern.result(Time_Left_Leg)*(Now_Param.Yaw_L.Tuning_Yaw + Now_Param.Yaw_L.Yaw);

        Kinetic_Shoulder_X_L = Step_Pattern.result(Time_Left_Leg)*(Now_Param.Shoulder.Swing_Left_Shoulder /*-3.0*Imu_Balance.Pitch_ADD_Angle*/);

        Kinetic_Shoulder_Y_L = Step_Pattern.result(Time_Left_Leg)*(Now_Param.Y.Side);

        if(Time_Right_Leg == 0.0)
        {
            Ikcoordinate_Info.X = (int)Now_Param.X.X;
            Ikcoordinate_Info.Y = (int)Now_Param.Y.Side;
            Ikcoordinate_Pub.publish(Ikcoordinate_Info);
        }
    }
    if(Balancing_Param.Balance.Balance_Pitch_Flag)
    {
        if(Zmp_Info.Right_Foot)
        {
            Zmp_Balance.PID_Zmp_Pitch_Balancing(Zmp_Info.Right_Y_zmp,Balancing_Param.Balance.Balance_Pitch_GP,Balancing_Param.Balance.Balance_Pitch_GI,Balancing_Param.Balance.Balance_Pitch_GD,Balancing_Param.Balance.Balance_Pitch_ELIMIT,Balancing_Param.Balance.Balance_Pitch_OLIMIT,Balancing_Param.Balance.Balance_Pitch_Neg_Target,Balancing_Param.Balance.Balance_Pitch_Pos_Target);

            Balancing_Param.Balance.Support_Con = 1;
        }
        else if(Zmp_Info.Left_Foot)
        {
            Zmp_Balance.PID_Zmp_Pitch_Balancing(Zmp_Info.Left_Y_zmp,Balancing_Param.Balance.Balance_Pitch_GP,Balancing_Param.Balance.Balance_Pitch_GI,Balancing_Param.Balance.Balance_Pitch_GD,Balancing_Param.Balance.Balance_Pitch_ELIMIT,Balancing_Param.Balance.Balance_Pitch_OLIMIT,Balancing_Param.Balance.Balance_Pitch_Neg_Target,Balancing_Param.Balance.Balance_Pitch_Pos_Target);

            Balancing_Param.Balance.Support_Con = -1;
        }
        else if(Zmp_Info.Both_Feet)
        {
            Zmp_Balance.PID_Zmp_Pitch_Balancing(Zmp_Info.Total_Y_zmp,Balancing_Param.Balance.Balance_Pitch_GP,Balancing_Param.Balance.Balance_Pitch_GI,Balancing_Param.Balance.Balance_Pitch_GD,Balancing_Param.Balance.Balance_Pitch_ELIMIT,Balancing_Param.Balance.Balance_Pitch_OLIMIT,Balancing_Param.Balance.Balance_Pitch_Neg_Target,Balancing_Param.Balance.Balance_Pitch_Pos_Target);

            Balancing_Param.Balance.Support_Con = 0;
        }

    }
    if(Balancing_Param.Balance.Balance_Angle_Pitch_Flag)
    {
//        if(IK.Signal_Robot_Support_Foot_Data.Right_Foot)
//        {
//            Zmp_Balance.PID_Zmp_Pitch_Angle_Balancing(Zmp_Info.Right_Y_zmp,Balancing_Param.Balance.Balance_Angle_Pitch_GP,Balancing_Param.Balance.Balance_Angle_Pitch_GI,Balancing_Param.Balance.Balance_Angle_Pitch_GD,Balancing_Param.Balance.Balance_Angle_Pitch_ELIMIT,Balancing_Param.Balance.Balance_Angle_Pitch_OLIMIT,Balancing_Param.Balance.Balance_Angle_Pitch_Neg_Target,Balancing_Param.Balance.Balance_Angle_Pitch_Pos_Target);
//        }
//        else if(IK.Signal_Robot_Support_Foot_Data.Left_Foot)
//        {
//            Zmp_Balance.PID_Zmp_Pitch_Angle_Balancing(Zmp_Info.Left_Y_zmp,Balancing_Param.Balance.Balance_Angle_Pitch_GP,Balancing_Param.Balance.Balance_Angle_Pitch_GI,Balancing_Param.Balance.Balance_Angle_Pitch_GD,Balancing_Param.Balance.Balance_Angle_Pitch_ELIMIT,Balancing_Param.Balance.Balance_Angle_Pitch_OLIMIT,Balancing_Param.Balance.Balance_Angle_Pitch_Neg_Target,Balancing_Param.Balance.Balance_Angle_Pitch_Pos_Target);
//        }
//        else if(IK.Signal_Robot_Support_Foot_Data.Both_Feet)
//        {
//            Zmp_Balance.PID_Zmp_Pitch_Angle_Balancing(Zmp_Info.Total_Y_zmp,Balancing_Param.Balance.Balance_Angle_Pitch_GP,Balancing_Param.Balance.Balance_Angle_Pitch_GI,Balancing_Param.Balance.Balance_Angle_Pitch_GD,Balancing_Param.Balance.Balance_Angle_Pitch_ELIMIT,Balancing_Param.Balance.Balance_Angle_Pitch_OLIMIT,Balancing_Param.Balance.Balance_Angle_Pitch_Neg_Target,Balancing_Param.Balance.Balance_Angle_Pitch_Pos_Target);
//        }

    }

    if(Balancing_Param.Balance.Balance_Roll_Flag)
    {
        if(Zmp_Info.Right_Foot)
        {
            Zmp_Balance.PID_Zmp_Roll_Balancing(Zmp_Info.Right_X_zmp,Balancing_Param.Balance.Balance_Roll_GP,Balancing_Param.Balance.Balance_Roll_GI,Balancing_Param.Balance.Balance_Roll_GD,Balancing_Param.Balance.Balance_Roll_ELIMIT,Balancing_Param.Balance.Balance_Roll_OLIMIT,Balancing_Param.Balance.Balance_Roll_Neg_Target,Balancing_Param.Balance.Balance_Roll_Pos_Target);

            Balancing_Param.Balance.Support_Con = 1;

        }
        else if(Zmp_Info.Left_Foot)
        {
            Zmp_Balance.PID_Zmp_Roll_Balancing(Zmp_Info.Left_X_zmp,Balancing_Param.Balance.Balance_Roll_GP,Balancing_Param.Balance.Balance_Roll_GI,Balancing_Param.Balance.Balance_Roll_GD,Balancing_Param.Balance.Balance_Roll_ELIMIT,Balancing_Param.Balance.Balance_Roll_OLIMIT,Balancing_Param.Balance.Balance_Roll_Neg_Target,Balancing_Param.Balance.Balance_Roll_Pos_Target);

            Balancing_Param.Balance.Support_Con = -1;
        }
        else if(Zmp_Info.Both_Feet)
        {

            Zmp_Balance.PID_Zmp_Roll_Balancing(Zmp_Info.Total_X_zmp,Balancing_Param.Balance.Balance_Roll_GP,Balancing_Param.Balance.Balance_Roll_GI,Balancing_Param.Balance.Balance_Roll_GD,Balancing_Param.Balance.Balance_Roll_ELIMIT,Balancing_Param.Balance.Balance_Roll_OLIMIT,Balancing_Param.Balance.Balance_Roll_Neg_Target,Balancing_Param.Balance.Balance_Roll_Pos_Target);
            Balancing_Param.Balance.Support_Con = 0;
        }


    }

    if (!Balancing_Param.Balance.Balance_Angle_Pitch_Flag)
    {
        Zmp_Balance.Target_Pitch_Angle = 0;

    }
    if (!Balancing_Param.Balance.Balance_Pitch_Flag)
    {
        Zmp_Balance.Target_X = 0;

    }
    if(!Balancing_Param.Balance.Balance_Roll_Flag)
    {
        Zmp_Balance.Target_Roll_Angle = 0;

    }

    if(!Balancing_Param.Balance.Balance_Angle_Pitch_Flag && !Balancing_Param.Balance.Balance_Pitch_Flag && !Balancing_Param.Balance.Balance_Roll_Flag)
    {
        Balancing_Param.Balance.Support_Con = 99;
    }

    if(Balancing_Param.Balance.Balance_Pitch_Flag_imu){
        Imu_Balance.PD_Pitch_control(Imu_Info.pitch,Balancing_Param.Balance.Balance_Pitch_GP_imu,Balancing_Param.Balance.Balance_Pitch_GI_imu,Balancing_Param.Balance.Balance_Pitch_GD_imu,Balancing_Param.Balance.Balance_Pitch_ELIMIT_imu,Balancing_Param.Balance.Balance_Pitch_OLIMIT_imu,Balancing_Param.Balance.Balance_Pitch_Neg_Target_imu,Balancing_Param.Balance.Balance_Pitch_Pos_Target_imu);
        //Past_Param.X.Compansate_X = sqrt(((-Past_Param.Z.Default_Z_Right * sin(Imu_Balance.Pitch_ADD_Angle*PI/180))*(-Past_Param.Z.Default_Z_Right * sin(Imu_Balance.Pitch_ADD_Angle*PI/180))) + ((-Past_Param.Z.Default_Z_Right + Past_Param.Z.Default_Z_Right * cos(Imu_Balance.Pitch_ADD_Angle*PI/180))*(-Past_Param.Z.Default_Z_Right + Past_Param.Z.Default_Z_Right * cos(Imu_Balance.Pitch_ADD_Angle*PI/180)))) * cos(Imu_Balance.Pitch_ADD_Angle*PI/180/2.0);
//        Past_Param.X.Compansate_X = -Past_Param.Z.Default_Z_Right*sin(Imu_Balance.Pitch_ADD_Angle*PI/180);
//        Past_Param.Z.Compansate_Z = -5.0*Past_Param.Z.Default_Z_Right*sin(Imu_Balance.Pitch_ADD_Angle*PI/180) * tan(Imu_Balance.Pitch_ADD_Angle*PI/180);
//        if(abs(Past_Param.X.Compansate_X) >= 15.0)
//        {
//            Past_Param.X.Compansate_X = 15.0;
//        }
//        if(abs(Past_Param.Z.Compansate_Z) >= 10.0)
//        {
//            Past_Param.Z.Compansate_Z = 10.0;
//        }
//        cout<<"Imu_ Pitch ================                    "<<Imu_Info.pitch<<endl;
        cout<<"Imu_Balance.Pitch_ADD_Angle :                  "<<Imu_Balance.Pitch_ADD_Angle<<endl;
//        cout<<"Past_Param.X.Compansate_X :                  "<<Past_Param.X.Compansate_X<<endl;
//        cout<<"Past_Param.Z.Compansate_Z :                  "<<Past_Param.Z.Compansate_Z<<endl;
//        cout<<"Past_Param.X.Compansate_X_2 :                  "<<-Past_Param.Z.Default_Z_Right*sin(Imu_Balance.Pitch_ADD_Angle*PI/180)<<endl;

        //        cout<<"Balancing_Param.Balance.Balance_Pitch_GP_imu  "<<Balancing_Param.Balance.Balance_Pitch_GP_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Pitch_GI_imu  "<<Balancing_Param.Balance.Balance_Pitch_GI_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Pitch_GD_imu  "<<Balancing_Param.Balance.Balance_Pitch_GD_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Pitch_ELIMIT_imu  "<<Balancing_Param.Balance.Balance_Pitch_ELIMIT_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Pitch_OLIMIT_imu  "<<Balancing_Param.Balance.Balance_Pitch_OLIMIT_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Pitch_Neg_Target_imu  "<<Balancing_Param.Balance.Balance_Pitch_Neg_Target_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Pitch_Pos_Target_imu  "<<Balancing_Param.Balance.Balance_Pitch_Pos_Target_imu<<endl<<endl;
    }
    if(!Balancing_Param.Balance.Balance_Pitch_Flag_imu){
        Imu_Balance.Pitch_ADD_Angle=0;
        Past_Param.X.Compansate_X = 0;
    }
    if(Balancing_Param.Balance.Balance_Roll_Flag_imu){
        Imu_Balance.PD_Roll_control(Imu_Info.roll,Balancing_Param.Balance.Balance_Roll_GP_imu,Balancing_Param.Balance.Balance_Roll_GI_imu,Balancing_Param.Balance.Balance_Roll_GD_imu,Balancing_Param.Balance.Balance_Roll_ELIMIT_imu,Balancing_Param.Balance.Balance_Roll_OLIMIT_imu,Balancing_Param.Balance.Balance_Roll_Neg_Target_imu,Balancing_Param.Balance.Balance_Roll_Pos_Target_imu);
        //        cout<<"Imu_Balance.Roll_ADD_Angle :                    "<<Imu_Balance.Roll_ADD_Angle<<endl;
        //        cout<<"Imu_ Roll ================                      "<<Imu_Info.roll<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Roll_GP_imu  "<<Balancing_Param.Balance.Balance_Roll_GP_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Roll_GI_imu  "<<Balancing_Param.Balance.Balance_Roll_GI_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Roll_GD_imu  "<<Balancing_Param.Balance.Balance_Roll_GD_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Roll_ELIMIT_imu  "<<Balancing_Param.Balance.Balance_Roll_ELIMIT_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Roll_OLIMIT_imu  "<<Balancing_Param.Balance.Balance_Roll_OLIMIT_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Roll_Neg_Target_imu  "<<Balancing_Param.Balance.Balance_Roll_Neg_Target_imu<<endl<<endl;
        //        cout<<"Balancing_Param.Balance.Balance_Roll_Pos_Target_imu  "<<Balancing_Param.Balance.Balance_Roll_Pos_Target_imu<<endl<<endl;
    }
    if(!Balancing_Param.Balance.Balance_Roll_Flag_imu){
        Imu_Balance.Roll_ADD_Angle=0;
    }





    if(Now_Param.IK_Flag)
    {
        Now_Param.Z.R_Rise_Condition = Rise_Pattern.result(Time_Right_Leg)*(Now_Param.Z.Rise_Right_Leg);
        Now_Param.Z.L_Rise_Condition = Rise_Pattern.result(Time_Left_Leg)*(Now_Param.Z.Rise_Left_Leg);

        IK.Balance_Control_Body_Upright(Zmp_Balance.Target_X,Now_Param.Z.Default_Z_Right,Time_Right_Leg,Now_Param.Z.R_Rise_Condition,Zmp_Balance.Target_Pitch_Angle,Balancing_Param.Balance.Balance_Value_0,Balancing_Param.Balance.Balance_Value_1,Zmp_Info.Left_Y_zmp,Zmp_Info.Right_Y_zmp,Zmp_Balance.Target_Roll_Angle,Balancing_Param.Balance.Balance_Value_2,Balancing_Param.Balance.Balance_Value_3, Model_Data.Link2Link, Zmp_Info.Left_X_zmp, Zmp_Info.Right_X_zmp, Balancing_Param.Balance.Support_Con);

        IK.solve(Kinetic_X_R,Kinetic_Y_R,Kinetic_Z_R,Kinetic_Yaw_R,Kinetic_X_L,Kinetic_Y_L,Kinetic_Z_L,Kinetic_Yaw_L,Leg,Kinetic_Shoulder_X_R,Kinetic_Shoulder_X_L,Kinetic_Shoulder_Y_R,Kinetic_Shoulder_Y_L,IK.Now_Balance_Theta.Theta1,IK.Now_Balance_Theta.Theta2,IK.Now_Balance_Theta.Theta3, Now_Param.Z.R_Rise_Condition, Now_Param.Z.L_Rise_Condition, Now_Param.Z.Rise_Right_Leg, Now_Param.Z.Rise_Left_Leg);

        walk_pattern_Info.xrpattern = Step_Pattern.result(Time_Right_Leg);
        walk_pattern_Info.yrpattern = -COM_Pattern.result(Time_Right_Leg);
        walk_pattern_Info.zrpattern = Rise_Pattern.result(Time_Right_Leg);
        walk_pattern_Info.trpattern = Turn_Pattern.result(Time_Right_Leg);

        walk_pattern_Info.xlpattern = Step_Pattern.result(Time_Left_Leg);
        walk_pattern_Info.ylpattern = -COM_Pattern.result(Time_Left_Leg);
        walk_pattern_Info.zlpattern = Rise_Pattern.result(Time_Left_Leg);
        walk_pattern_Info.tlpattern = Turn_Pattern.result(Time_Left_Leg);




        walk_pattern_Pub.publish(walk_pattern_Info);

        cout << "===============IKflag_true====================" << endl;
        cout << "X : "<<Kinetic_X_R<<endl;

        //        cout << "Kinetic_X_R  >> " << Kinetic_X_R  << endl;
        //        cout << "Kinetic_Y_R  >> " << Kinetic_Y_R  << endl;
        //        cout<< "timer : "<<Time_Right_Leg<<endl;
        //        cout<<" end_timer : "<<Time_Right_Leg_End<<endl;
        //        cout << "Kinetic_X_L  >> " << Kinetic_X_L  << endl;
        //        cout << "Kinetic_Y_L  >> " << Kinetic_Y_L  << endl;
        //        cout << "Kinetic_Z_R  >> " << Kinetic_Z_R  << endl;
        //        cout << "Kinetic_Z_L  >> " << Kinetic_Z_L  << endl;
        //        cout << "Kinetic_Yaw_R >> "  << Kinetic_Yaw_R << endl;
        //        cout << "Kinetic_Yaw_L >> "  << Kinetic_Yaw_L << endl;
        //        cout << "ShoulderR"<<Kinetic_Shoulder_X_R<<endl;
        //        cout << "ShoulderL"<<Kinetic_Shoulder_L<<endl;


    }

    Ik_Flag_Past = Now_Param.IK_Flag;
}



void timer_callback(const ros::TimerEvent&)
{
    static int Milli_Second = 0;
    Milli_Second++;


    if(Milli_Second >= Now_Param.Frequency)
    {

        Timer_Time +=Now_Param.Frequency;



        //if(Ready_To_Start)
        //if(Start_Flag)
        Timer_Time_Start +=Now_Param.Frequency;

        if(End_Flag)Timer_Time_End +=Now_Param.Frequency;


        Milli_Second=0;
        if(Timer_Time >=Now_Param.Entire_Time)
        {
            Timer_Time = 0;
        }


        if(Timer_Time_Start >=Now_Param.Sink_Entire_Time)
        {
            Timer_Time_Start = 0;
        }
        if(Timer_Time_End >=Now_Param.End_Entire_Time)
        {
            Timer_Time_End = 0;
        }

        if(!Smooth_Flag && !Start_Flag && !End_Flag)
        {
            Timer_Time_Find +=Now_Param.Frequency;
        }
        if(Timer_Time_Find >=Now_Param.Entire_Time)
        {
            Timer_Time_Find = 0;
        }



        Walk.Walk_Start_End(Now_Param);

        if(Now_Param.Check_ratio.Ratio_Flag != Past_Param.Check_ratio.Ratio_Flag)
        {
//            cout<<"safsgrwgfsadzhwefajrqwdCFR3WEQ"<<endl;
            Walk.Generate_Pattern(Now_Param);
        }

        Now_Param.Z.Z_com = 330.0 - Model_Data.Init_Z_Up;// Straight_Robot's Z_com - Model_Data.Init_Z_Up
        COM_Info.X_com = (Now_Param.Z.Z_com/9.81)*((Walk.Amount_of_Change_X)/(Now_Param.Entire_Time*0.01)) + Zmp_Info.Total_Y_zmp;
        COM_Info.Y_com = (Now_Param.Z.Z_com/9.81)*((Walk.Amount_of_Change_Y)/(Now_Param.Entire_Time*0.01)) + Zmp_Info.Total_X_zmp;

//        cout << "Change_X : "<<Walk.Amount_of_Change_X<<endl;
//        cout << "Change_Y : "<<Walk.Amount_of_Change_Y<<endl;
//        cout << "COM_Info.X_com : "<<COM_Info.X_com<<endl;
//        cout << "COM_Info.Y_com : "<<COM_Info.Y_com<<endl;

        Com_Pub.publish(COM_Info);
    }
}

void get_parameters()
{
    std::ifstream is;
    is.open("/home/robit/catkin_ws/src/tune_walk/work/2021_0714_double_3_speed_up");


    is >> Past_Param.Entire_Time;
    //Now_Param.Entire_Time = Past_Param.Entire_Time;
    is >> Past_Param.Frequency;
    is >> Balancing_Param.Balance.Ratio_Check_Flag;
    is >> Past_Param.X.Default_X_Right;
    is >> Past_Param.X.Default_X_Left;
    is >> Past_Param.Y.Default_Y_Right;
    is >> Past_Param.Y.Default_Y_Left;
    is >> Past_Param.Z.Default_Z_Right;
    is >> Past_Param.Z.Default_Z_Left;
    is >> IK.Past_Motor_Angle.Motor_Angle_10;
    is >> IK.Past_Motor_Angle.Motor_Angle_11;
    is >> IK.Past_Motor_Angle.Motor_Angle_12;
    is >> IK.Past_Motor_Angle.Motor_Angle_13;
    is >> IK.Past_Motor_Angle.Motor_Angle_14;
    is >> IK.Past_Motor_Angle.Motor_Angle_15;
    is >> IK.Past_Motor_Angle.Motor_Angle_16;
    is >> IK.Past_Motor_Angle.Motor_Angle_17;
    is >> IK.Past_Motor_Angle.Motor_Angle_18;
    is >> IK.Past_Motor_Angle.Motor_Angle_19;
    is >> IK.Past_Motor_Angle.Motor_Angle_20;
    is >> IK.Past_Motor_Angle.Motor_Angle_21;
    is >> Past_Param.Y.Swing_Right_Leg;
    is >> Past_Param.Y.Swing_Left_Leg;
    is >> Past_Param.Shoulder.Swing_Right_Shoulder;
    is >> Past_Param.Shoulder.Swing_Left_Shoulder;
    is >> Past_Param.Z.Rise_Right_Leg;
    is >> Past_Param.Z.Rise_Left_Leg;
    is >> Now_Param.Start_Entire_Time;
    is >> Now_Param.Y.Start_Swing;
    is >> Now_Param.Z.Start_Rise;
    is >> Now_Param.End_Entire_Time;
    is >> Now_Param.Y.End_Swing;
    is >> Now_Param.Z.End_Rise;
    is >> Past_Param.X.Tuning_X;
    is >> Past_Param.Y.Tuning_Side;
    is >> Past_Param.Yaw_R.Tuning_Yaw;
    is >> Balancing_Param.Balance.Balance_Value_0;
    is >> Balancing_Param.Balance.Balance_Pitch_GP;
    is >> Balancing_Param.Balance.Balance_Pitch_GI;
    is >> Balancing_Param.Balance.Balance_Pitch_GD;
    is >> Balancing_Param.Balance.Balance_Pitch_ELIMIT;
    is >> Balancing_Param.Balance.Balance_Pitch_OLIMIT;
    is >> Balancing_Param.Balance.Balance_Pitch_Neg_Target;
    is >> Balancing_Param.Balance.Balance_Pitch_Pos_Target;

    is >> Balancing_Param.Balance.Balance_Value_1;
    is >> Balancing_Param.Balance.Balance_Angle_Pitch_GP;
    is >> Balancing_Param.Balance.Balance_Angle_Pitch_GI;
    is >> Balancing_Param.Balance.Balance_Angle_Pitch_GD;
    is >> Balancing_Param.Balance.Balance_Angle_Pitch_ELIMIT;
    is >> Balancing_Param.Balance.Balance_Angle_Pitch_OLIMIT;
    is >> Balancing_Param.Balance.Balance_Angle_Pitch_Neg_Target;
    is >> Balancing_Param.Balance.Balance_Angle_Pitch_Pos_Target;

    is >> Balancing_Param.Balance.Balance_Value_2;
    is >> Balancing_Param.Balance.Balance_Roll_GP;
    is >> Balancing_Param.Balance.Balance_Roll_GI;
    is >> Balancing_Param.Balance.Balance_Roll_GD;
    is >> Balancing_Param.Balance.Balance_Roll_ELIMIT;
    is >> Balancing_Param.Balance.Balance_Roll_OLIMIT;
    is >> Balancing_Param.Balance.Balance_Roll_Neg_Target;
    is >> Balancing_Param.Balance.Balance_Roll_Pos_Target;

    is >> Balancing_Param.Balance.Balance_Value_3;



    is >> Balancing_Param.Balance.Balance_Value_4;
    is >> Balancing_Param.Balance.Balance_Value_5;
    is >> Balancing_Param.Balance.Balance_Pitch_GP_imu;
    is >> Balancing_Param.Balance.Balance_Pitch_GI_imu;
    is >> Balancing_Param.Balance.Balance_Pitch_GD_imu;
    is >> Balancing_Param.Balance.Balance_Pitch_ELIMIT_imu;
    is >> Balancing_Param.Balance.Balance_Pitch_OLIMIT_imu;
    is >> Balancing_Param.Balance.Balance_Pitch_Neg_Target_imu;
    is >> Balancing_Param.Balance.Balance_Pitch_Pos_Target_imu;

    is >> Balancing_Param.Balance.Balance_Roll_GP_imu;
    is >> Balancing_Param.Balance.Balance_Roll_GI_imu;
    is >> Balancing_Param.Balance.Balance_Roll_GD_imu;
    is >> Balancing_Param.Balance.Balance_Roll_ELIMIT_imu;
    is >> Balancing_Param.Balance.Balance_Roll_OLIMIT_imu;
    is >> Balancing_Param.Balance.Balance_Roll_Neg_Target_imu;
    is >> Balancing_Param.Balance.Balance_Roll_Pos_Target_imu;
    is >> Model_Data.Center2Leg;
    is >> Model_Data.Link2Link;
    is >> Model_Data.Init_Z_Up;

    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_10;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_11;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_12;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_13;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_14;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_15;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_16;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_17;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_18;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_19;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_20;
    is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_21;




    //Balance flag on/off
    Balancing_Param.Balance.Balance_Angle_Pitch_Flag = false;
    Balancing_Param.Balance.Balance_Pitch_Flag = true;
    Balancing_Param.Balance.Balance_Roll_Flag = true;
    Balancing_Param.Balance.Balance_Pitch_Flag_imu = true;
    Balancing_Param.Balance.Balance_Roll_Flag_imu = false;

    //Default//
    Now_Param.Y.Default_Y_Right = -(Model_Data.Center2Leg);
    Now_Param.Y.Default_Y_Left = Model_Data.Center2Leg;
    Now_Param.Z.Default_Z_Right = -(2*Model_Data.Link2Link - Model_Data.Init_Z_Up);
    Now_Param.Z.Default_Z_Left = -(2*Model_Data.Link2Link - Model_Data.Init_Z_Up);

    //Offset & % of IK init//
    IK.Now_Motor_Angle = IK.Past_Motor_Angle;
    IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;

//        cout<<"Past_Param.Entire_Time   "<<Past_Param.Entire_Time<<endl<<endl;
//        cout<<"Past_Param.Frequency  "<<Past_Param.Frequency<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Ratio_Check_Flag  "<<Balancing_Param.Balance.Ratio_Check_Flag<<endl<<endl;
//        cout<<"Past_Param.X.Default_X_Right  "<<Past_Param.X.Default_X_Right<<endl<<endl;
//        cout<<"Past_Param.X.Default_X_Left  "<<Past_Param.X.Default_X_Left<<endl<<endl;
//        cout<<"Past_Param.Y.Default_Y_Right  "<<Past_Param.Y.Default_Y_Right<<endl<<endl;
//        cout<<"Past_Param.Y.Default_Y_Left  "<<Past_Param.Y.Default_Y_Left<<endl<<endl;
//        cout<<"Past_Param.Z.Default_Z_Right  "<<Past_Param.Z.Default_Z_Right<<endl<<endl;
//        cout<<"Past_Param.Z.Default_Z_Left  "<<Past_Param.Z.Default_Z_Left<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_10  "<<IK.Past_Motor_Angle.Motor_Angle_10<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_11  "<<IK.Past_Motor_Angle.Motor_Angle_11<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_12  "<<IK.Past_Motor_Angle.Motor_Angle_12<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_13  "<<IK.Past_Motor_Angle.Motor_Angle_13<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_14  "<<IK.Past_Motor_Angle.Motor_Angle_14<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_15  "<<IK.Past_Motor_Angle.Motor_Angle_15<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_16  "<<IK.Past_Motor_Angle.Motor_Angle_16<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_17  "<<IK.Past_Motor_Angle.Motor_Angle_17<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_18  "<<IK.Past_Motor_Angle.Motor_Angle_18<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_19  "<<IK.Past_Motor_Angle.Motor_Angle_19<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_20  "<<IK.Past_Motor_Angle.Motor_Angle_20<<endl<<endl;
//        cout<<"IK.Past_Motor_Angle.Motor_Angle_21  "<<IK.Past_Motor_Angle.Motor_Angle_21<<endl<<endl;


//        cout<<"Past_Param.Y.Swing_Right_Leg  "<<Past_Param.Y.Swing_Right_Leg<<endl<<endl;
//        cout<<"Past_Param.Y.Swing_Left_Leg  "<<Past_Param.Y.Swing_Left_Leg<<endl<<endl;
//        cout<<"Past_Param.Shoulder.Swing_Right_Shoulder  "<<Past_Param.Shoulder.Swing_Right_Shoulder<<endl<<endl;
//        cout<<"Past_Param.Shoulder.Swing_Left_Shoulder  "<<Past_Param.Shoulder.Swing_Left_Shoulder<<endl<<endl;
//        cout<<"Past_Param.Z.Rise_Right_Leg  "<<Past_Param.Z.Rise_Right_Leg<<endl<<endl;
//        cout<<"Past_Param.Z.Rise_Left_Leg  "<<Past_Param.Z.Rise_Left_Leg<<endl<<endl;
//        cout<<"Now_Param.Start_Entire_Time  "<<Now_Param.Start_Entire_Time<<endl<<endl;
//        cout<<"Now_Param.Y.Start_Swing  "<<Now_Param.Y.Start_Swing<<endl<<endl;
//        cout<<"Now_Param.Z.Start_Rise  "<<Now_Param.Z.Start_Rise<<endl<<endl;
//        cout<<"Now_Param.End_Entire_Time  "<<Now_Param.End_Entire_Time<<endl<<endl;
//        cout<<"Now_Param.Y.End_Swing  "<<Now_Param.Y.End_Swing<<endl<<endl;
//        cout<<"Now_Param.Z.End_Rise  "<<Now_Param.Z.End_Rise<<endl<<endl;
//        cout<<"Past_Param.X.Tuning_X  "<<Past_Param.X.Tuning_X<<endl<<endl;
//        cout<<"Past_Param.Y.Tuning_Side  "<<Past_Param.Y.Tuning_Side<<endl<<endl;
//        cout<<"Past_Param.Yaw_R.Tuning_Yaw  "<<Past_Param.Yaw_R.Tuning_Yaw<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Value_0  "<<Balancing_Param.Balance.Balance_Value_0<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_GP  "<<Balancing_Param.Balance.Balance_Pitch_GP<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_GI  "<<Balancing_Param.Balance.Balance_Pitch_GI<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_GD  "<<Balancing_Param.Balance.Balance_Pitch_GD<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_ELIMIT  "<<Balancing_Param.Balance.Balance_Pitch_ELIMIT<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_OLIMIT  "<<Balancing_Param.Balance.Balance_Pitch_OLIMIT<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_Neg_Target  "<<Balancing_Param.Balance.Balance_Pitch_Neg_Target<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_Pos_Target  "<<Balancing_Param.Balance.Balance_Pitch_Pos_Target<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Value_1  "<<Balancing_Param.Balance.Balance_Value_1<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Angle_Pitch_GP  "<<Balancing_Param.Balance.Balance_Angle_Pitch_GP<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Angle_Pitch_GI  "<<Balancing_Param.Balance.Balance_Angle_Pitch_GI<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Angle_Pitch_GD  "<<Balancing_Param.Balance.Balance_Angle_Pitch_GD<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Angle_Pitch_ELIMIT  "<<Balancing_Param.Balance.Balance_Angle_Pitch_ELIMIT<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Angle_Pitch_OLIMIMT  "<<Balancing_Param.Balance.Balance_Angle_Pitch_OLIMIT<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Angle_Pitch_Neg_Target  "<<Balancing_Param.Balance.Balance_Angle_Pitch_Neg_Target<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Angle_Pitch_Pos_Target  "<<Balancing_Param.Balance.Balance_Angle_Pitch_Pos_Target<<endl<<endl;

//        cout<<"Balancing_Param.Balance.Balance_Value_2  "<<Balancing_Param.Balance.Balance_Value_2<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_GP  "<<Balancing_Param.Balance.Balance_Roll_GP<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_GI  "<<Balancing_Param.Balance.Balance_Roll_GI<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_GD  "<<Balancing_Param.Balance.Balance_Roll_GD<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_ELIMIT  "<<Balancing_Param.Balance.Balance_Roll_ELIMIT<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_OLIMIT  "<<Balancing_Param.Balance.Balance_Roll_OLIMIT<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_Neg_Target  "<<Balancing_Param.Balance.Balance_Roll_Neg_Target<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_Pos_Target  "<<Balancing_Param.Balance.Balance_Roll_Pos_Target<<endl<<endl;

//        cout<<"Balancing_Param.Balance.Balance_Value_3  "<<Balancing_Param.Balance.Balance_Value_3<<endl<<endl;


//        cout<<"Balancing_Param.Balance.Balance_Value_4  "<<Balancing_Param.Balance.Balance_Value_4<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Value_5  "<<Balancing_Param.Balance.Balance_Value_5<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_GP_imu  "<<Balancing_Param.Balance.Balance_Pitch_GP_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_GI_imu  "<<Balancing_Param.Balance.Balance_Pitch_GI_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_GD_imu  "<<Balancing_Param.Balance.Balance_Pitch_GD_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_ELIMIT_imu  "<<Balancing_Param.Balance.Balance_Pitch_ELIMIT_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_OLIMIT_imu  "<<Balancing_Param.Balance.Balance_Pitch_OLIMIT_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_Neg_Target_imu  "<<Balancing_Param.Balance.Balance_Pitch_Neg_Target_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Pitch_Pos_Target_imu  "<<Balancing_Param.Balance.Balance_Pitch_Pos_Target_imu<<endl<<endl;

//        cout<<"Balancing_Param.Balance.Balance_Roll_GP_imu  "<<Balancing_Param.Balance.Balance_Roll_GP_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_GI_imu  "<<Balancing_Param.Balance.Balance_Roll_GI_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_GD_imu  "<<Balancing_Param.Balance.Balance_Roll_GD_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_ELIMIT_imu  "<<Balancing_Param.Balance.Balance_Roll_ELIMIT_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_OLIMIT_imu  "<<Balancing_Param.Balance.Balance_Roll_OLIMIT_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_Neg_Target_imu  "<<Balancing_Param.Balance.Balance_Roll_Neg_Target_imu<<endl<<endl;
//        cout<<"Balancing_Param.Balance.Balance_Roll_Pos_Target_imu  "<<Balancing_Param.Balance.Balance_Roll_Pos_Target_imu<<endl<<endl;
//        cout<<"Model_Data.Center2Leg  "<<Model_Data.Center2Leg<<endl<<endl;
//        cout<<"Model_Data.Link2Link  "<<Model_Data.Link2Link<<endl<<endl;
//        cout<<"Model_Data.Init_Z_Up  "<<Model_Data.Init_Z_Up<<endl<<endl;

//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_10   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_10<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_11   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_11<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_12   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_12<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_13   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_13<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_14   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_14<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_15   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_15<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_16   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_16<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_17   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_17<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_18   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_18<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_19   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_19<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_20   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_20<<endl<<endl;
//        cout<<"IK.Past_Percentage_of_IK_Motor.Motor_Multiple_21   "<<IK.Past_Percentage_of_IK_Motor.Motor_Multiple_21<<endl<<endl;


}
void master2ik_callback(const msg_generate::ik_msg::ConstPtr& msg)
{
    Past_Param.IK_Flag = msg->flag;
    Past_Param.X.X=msg->X_length;
    Past_Param.Y.Side = msg->Y_length;
    Past_Param.Yaw_R.Yaw = msg->Yaw;
    Past_Param.Yaw_L.Yaw = msg->Yaw;

    if(Past_Param.X.X >= X_LIMIT)    Past_Param.X.X = X_LIMIT;
    else if(Past_Param.X.X <= -X_LIMIT)  Past_Param.X.X = -X_LIMIT;

    if(Past_Param.Y.Side >= Y_LIMIT) Past_Param.Y.Side = Y_LIMIT;
    else if(Past_Param.Y.Side <= -Y_LIMIT) Past_Param.Y.Side = -Y_LIMIT;

    if(Past_Param.Yaw_R.Yaw >= YAW_LIMIT)   Past_Param.Yaw_R.Yaw = YAW_LIMIT;
    else if(Past_Param.Yaw_R.Yaw <= -YAW_LIMIT)   Past_Param.Yaw_R.Yaw = -YAW_LIMIT;

    if(Past_Param.Yaw_L.Yaw >= YAW_LIMIT)   Past_Param.Yaw_L.Yaw = YAW_LIMIT;
    else if(Past_Param.Yaw_L.Yaw <= -YAW_LIMIT)   Past_Param.Yaw_L.Yaw = -YAW_LIMIT;
}





double avg (double x)
{

    unsigned char i;

    double sum = 0, average;

    for (i = 0; i < FILTERDATA - 1; i++)
        data[i] = data[i+1];

    data[FILTERDATA - 1] = x;


    for (i = 0; i < 10; i++)


        sum += data[i];


    average = sum / FILTERDATA;

//     sort(data,data+FILTERDATA);//

    return /*data[4];*/ average;
}

double negative_position(double x)
{
    if(x<=0.0)
    {
        return x;
    }
    else
    {
        return 0;
    }
}

double positive_position(double x)
{
    if(x>=0.0)
    {
        return x;
    }
    else
    {
        return 0;
    }
}

void imuCallback(const msg_generate::imu_msg::ConstPtr &msg)
{
//    Imu_Info.pitch     = msg->pitch;
    Imu_Info.roll      = msg->roll;
    Imu_Info.yaw       = msg->yaw;
    Imu_Info.roll_acc  = msg->roll_acc;
    Imu_Info.pitch_acc = msg->pitch_acc;
    Imu_Info.yaw_acc   = msg->yaw_acc;
    Imu_Info.pitch = avg(msg->pitch);

}
void tune2walkCallback(const msg_generate::tune2walk::ConstPtr& msg)
{
    Past_Param.IK_Flag = msg ->IK_Flag;
    Past_Param.Entire_Time = msg ->Entire_Time;
    Past_Param.Frequency = msg->Frequency;
    Balancing_Param.Balance.Ratio_Check_Flag = msg -> Ratio_Check_Flag;

    Past_Param.X.X = msg->Test_X;
    Past_Param.X.Tuning_X = msg->Tuning_X;
    Past_Param.X.Default_X_Right = msg->Default_X_Right;
    Past_Param.X.Default_X_Left = msg->Default_X_Left;

    Past_Param.Y.Side = msg->Test_Side;
    Past_Param.Y.Tuning_Side = msg->Tuning_Side;
    Past_Param.Y.Default_Y_Right = msg->Default_Y_Right;
    Past_Param.Y.Default_Y_Left = msg->Default_Y_Left;
    Past_Param.Y.Swing_Right_Leg = msg->Swing_Right_Leg;
    Past_Param.Y.Swing_Left_Leg = msg->Swing_Left_Leg;

    Past_Param.Z.Default_Z_Right = msg->Default_Z_Right;
    Past_Param.Z.Default_Z_Left = msg->Default_Z_Left;
    Past_Param.Z.Rise_Right_Leg = msg->Rise_Right_Leg;
    Past_Param.Z.Rise_Left_Leg = msg->Rise_Left_Leg;
    Past_Param.Shoulder.Swing_Right_Shoulder = msg->Swing_Right_Shoulder;
    Past_Param.Shoulder.Swing_Left_Shoulder = msg->Swing_Left_Shoulder;

    Past_Param.Yaw_R.Yaw = msg->Test_Yaw;
    Past_Param.Yaw_R.Tuning_Yaw = msg->Tuning_Yaw;

    Past_Param.Yaw_L.Yaw = msg->Test_Yaw;
    Past_Param.Yaw_L.Tuning_Yaw = msg->Tuning_Yaw;

    IK.Past_Motor_Angle.Motor_Angle_10 = msg-> Offset_10_Motor;
    IK.Past_Motor_Angle.Motor_Angle_11 = msg-> Offset_11_Motor;
    IK.Past_Motor_Angle.Motor_Angle_12 = msg-> Offset_12_Motor;
    IK.Past_Motor_Angle.Motor_Angle_13 = msg-> Offset_13_Motor;
    IK.Past_Motor_Angle.Motor_Angle_14 = msg-> Offset_14_Motor;
    IK.Past_Motor_Angle.Motor_Angle_15 = msg-> Offset_15_Motor;
    IK.Past_Motor_Angle.Motor_Angle_16 = msg-> Offset_16_Motor;
    IK.Past_Motor_Angle.Motor_Angle_17 = msg-> Offset_17_Motor;
    IK.Past_Motor_Angle.Motor_Angle_18 = msg-> Offset_18_Motor;
    IK.Past_Motor_Angle.Motor_Angle_19 = msg-> Offset_19_Motor;
    IK.Past_Motor_Angle.Motor_Angle_20 = msg-> Offset_20_Motor;
    IK.Past_Motor_Angle.Motor_Angle_21 = msg-> Offset_21_Motor;

    IK.Now_Motor_Angle=IK.Past_Motor_Angle;


    Now_Param.Start_Entire_Time = msg ->Start_Entire_Time;


    Now_Param.Y.Start_Swing = msg->Start_Swing;
    Now_Param.Z.Start_Rise = msg->Start_Rise;
    Now_Param.End_Entire_Time = msg->End_Entire_Time;
    Now_Param.Y.End_Swing = msg->End_Swing;
    Now_Param.Z.End_Rise = msg->End_Rise;
    Balancing_Param.Balance.Balance_Value_0 = msg -> Balance_Value_0;
    Balancing_Param.Balance.Balance_Value_1 = msg -> Balance_Value_1;
    Balancing_Param.Balance.Balance_Value_2 = msg -> Balance_Value_2;
    Balancing_Param.Balance.Balance_Value_3 = msg -> Balance_Value_3;
    Balancing_Param.Balance.Balance_Pitch_GP = msg -> Balance_Pitch_GP;
    Balancing_Param.Balance.Balance_Pitch_GI = msg -> Balance_Pitch_GI;
    Balancing_Param.Balance.Balance_Pitch_GD = msg -> Balance_Pitch_GD;
    Balancing_Param.Balance.Balance_Pitch_ELIMIT = msg -> Balance_Pitch_ELIMIT;
    Balancing_Param.Balance.Balance_Pitch_OLIMIT = msg -> Balance_Pitch_OLIMIT;
    Balancing_Param.Balance.Balance_Pitch_Neg_Target = msg -> Balance_Pitch_Neg_Target;
    Balancing_Param.Balance.Balance_Pitch_Pos_Target = msg -> Balance_Pitch_Pos_Target;

    Balancing_Param.Balance.Balance_Angle_Pitch_GP = msg -> Balance_Angle_Pitch_GP;
    Balancing_Param.Balance.Balance_Angle_Pitch_GI = msg -> Balance_Angle_Pitch_GI;
    Balancing_Param.Balance.Balance_Angle_Pitch_GD = msg -> Balance_Angle_Pitch_GD;
    Balancing_Param.Balance.Balance_Angle_Pitch_ELIMIT = msg -> Balance_Angle_Pitch_ELIMIT;
    Balancing_Param.Balance.Balance_Angle_Pitch_OLIMIT = msg -> Balance_Angle_Pitch_OLIMIT;
    Balancing_Param.Balance.Balance_Angle_Pitch_Neg_Target = msg -> Balance_Angle_Pitch_Neg_Target;
    Balancing_Param.Balance.Balance_Angle_Pitch_Pos_Target = msg -> Balance_Angle_Pitch_Pos_Target;
    Balancing_Param.Balance.Balance_Roll_GP = msg -> Balance_Roll_GP;
    Balancing_Param.Balance.Balance_Roll_GI = msg -> Balance_Roll_GI;
    Balancing_Param.Balance.Balance_Roll_GD = msg -> Balance_Roll_GD;
    Balancing_Param.Balance.Balance_Roll_ELIMIT = msg -> Balance_Roll_ELIMIT;
    Balancing_Param.Balance.Balance_Roll_OLIMIT = msg -> Balance_Roll_OLIMIT;
    Balancing_Param.Balance.Balance_Roll_Neg_Target = msg -> Balance_Roll_Neg_Target;
    Balancing_Param.Balance.Balance_Roll_Pos_Target = msg -> Balance_Roll_Pos_Target;

    Balancing_Param.Balance.Balance_Pitch_GP_imu = msg -> Balance_Pitch_GP_imu;
    Balancing_Param.Balance.Balance_Pitch_GI_imu = msg -> Balance_Pitch_GI_imu;
    Balancing_Param.Balance.Balance_Pitch_GD_imu = msg -> Balance_Pitch_GD_imu;
    Balancing_Param.Balance.Balance_Pitch_ELIMIT_imu = msg -> Balance_Pitch_ELIMIT_imu;
    Balancing_Param.Balance.Balance_Pitch_OLIMIT_imu = msg -> Balance_Pitch_OLIMIT_imu;
    Balancing_Param.Balance.Balance_Pitch_Neg_Target_imu = msg -> Balance_Pitch_Neg_Target_imu;
    Balancing_Param.Balance.Balance_Pitch_Pos_Target_imu = msg -> Balance_Pitch_Pos_Target_imu;

    Balancing_Param.Balance.Balance_Roll_GP_imu = msg -> Balance_Roll_GP_imu;
    Balancing_Param.Balance.Balance_Roll_GI_imu = msg -> Balance_Roll_GI_imu;
    Balancing_Param.Balance.Balance_Roll_GD_imu = msg -> Balance_Roll_GD_imu;
    Balancing_Param.Balance.Balance_Roll_ELIMIT_imu = msg -> Balance_Roll_ELIMIT_imu;
    Balancing_Param.Balance.Balance_Roll_OLIMIT_imu = msg -> Balance_Roll_OLIMIT_imu;
    Balancing_Param.Balance.Balance_Roll_Neg_Target_imu = msg -> Balance_Roll_Neg_Target_imu;
    Balancing_Param.Balance.Balance_Roll_Pos_Target_imu = msg -> Balance_Roll_Pos_Target_imu;

    Balancing_Param.Balance.Balance_Pitch_Flag = msg -> Balance_Pitch_Flag;
    Balancing_Param.Balance.Balance_Angle_Pitch_Flag = msg -> Balance_Angle_Pitch_Flag;
    Balancing_Param.Balance.Balance_Roll_Flag = msg -> Balance_Roll_Flag;
    Balancing_Param.Balance.Balance_Pitch_Flag_imu = msg -> Balance_Pitch_Flag_imu;
    Balancing_Param.Balance.Balance_Roll_Flag_imu = msg -> Balance_Roll_Flag_imu;

    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_10 = msg ->Percentage_of_IK_10_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_11 = msg ->Percentage_of_IK_11_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_12 = msg ->Percentage_of_IK_12_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_13 = msg ->Percentage_of_IK_13_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_14 = msg ->Percentage_of_IK_14_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_15 = msg ->Percentage_of_IK_15_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_16 = msg ->Percentage_of_IK_16_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_17 = msg ->Percentage_of_IK_17_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_18 = msg ->Percentage_of_IK_18_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_19 = msg ->Percentage_of_IK_19_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_20 = msg ->Percentage_of_IK_20_Motor;
    IK.Past_Percentage_of_IK_Motor.Motor_Multiple_21 = msg ->Percentage_of_IK_21_Motor;

    IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;
//    Model_Data.Center2Leg = msg -> Center2Leg;
//    Model_Data.Link2Link = msg -> Link2Link;
//    Model_Data.Init_Z_Up = msg -> Init_Z_Up;

    //Default//
//    Past_Param.Y.Default_Y_Right = -(Model_Data.Center2Leg);
//    Past_Param.Y.Default_Y_Left = Model_Data.Center2Leg;
//    Past_Param.Z.Default_Z_Right = -(2*Model_Data.Link2Link - Model_Data.Init_Z_Up);
//    Past_Param.Z.Default_Z_Left = -(2*Model_Data.Link2Link - Model_Data.Init_Z_Up);

    //cout<<"Balancing_Param.Balance.Ratio_Check_Flag  "<<Balancing_Param.Balance.Ratio_Check_Flag<<endl<<endl;


}
void zmpcallback(const msg_generate::zmp_msg::ConstPtr& msg)
{
    Zmp_Info.Left_X_zmp  = msg->Left_X_zmp;
    Zmp_Info.Left_Y_zmp  = msg->Left_Y_zmp;
    Zmp_Info.Right_X_zmp = msg->Right_X_zmp;
    Zmp_Info.Right_Y_zmp = msg->Right_Y_zmp;
    Zmp_Info.Total_X_zmp = msg->Total_X_zmp;
    Zmp_Info.Total_Y_zmp = msg->Total_Y_zmp;
    Zmp_Info.Left_Foot = msg->Left_Foot;
    Zmp_Info.Right_Foot = msg->Right_Foot;
    Zmp_Info.Both_Feet = msg->Both_Feet;

//    cout<<"Zmp_Info.Left_X_zmp   "<<Zmp_Info.Left_X_zmp<<endl<<endl;
//    cout<<"Zmp_Info.Left_Y_zmp   "<<Zmp_Info.Left_Y_zmp<<endl<<endl;
//    cout<<"Zmp_Info.Right_X_zmp  "<<Zmp_Info.Right_X_zmp<<endl<<endl;
//    cout<<"Zmp_Info.Right_Y_zmp  "<<Zmp_Info.Right_Y_zmp<<endl<<endl;
//    cout<<"Zmp_Info.Total_X_zmp   "<<Zmp_Info.Total_X_zmp<<endl<<endl;
//    cout<<"Zmp_Info.Total_Y_zmp   "<<Zmp_Info.Total_Y_zmp<<endl<<endl;
//    cout<<"Zmp_Info.Left_Foot  "<<Zmp_Info.Left_Foot<<endl<<endl;
//    cout<<"Zmp_Info.Right_Foot   "<<Zmp_Info.Right_Foot<<endl<<endl;
//    cout<<"Zmp_Info.Both_Feet   "<<Zmp_Info.Both_Feet<<endl<<endl;

}
