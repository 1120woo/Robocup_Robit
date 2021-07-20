#include "../include/master_yongpyong/master_yongpyong.hpp"

using namespace std;

GameState gameInfo;
VisionMsg visionInfo;
ImuMsg imuInfo;
UdpMsg udpInfo;
LocalMsg localInfo;

//Tracking parameter initial=========================
int Tracking_point_x = trackingDefaultPointX;
int Tracking_point_y = trackingDefaultPointY;

double Tracking_thing_x = trackingDefaultPointX;
double Tracking_thing_y = trackingDefaultPointY;

int Tilt_Max = defaultTiltMax;
int Tilt_Min = defaultTiltMin;
int Pan_Max = defaultPanMax;
int Pan_Min = defaultPanMin;

int scan_pan_range = defaultScanPanRange;
int scan_tilt_range = defaultScanTiltRange;

double PAN_POSITION = 0.0;
double TILT_POSITION = 0.0;

double temp_pan_position = 0.0;
double temp_tilt_position = 0.0;
//===================================================



int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_sydney");
    ros::NodeHandle n;

    //PID init======================================================================
    double kP = 0.3;//0.035
    double kI = 0.0;
    double kD = 0.001;

    PID_Control_init(&Tracking_pid_pan, kP, kI, kD, 150, 90);
    PID_Control_init(&Tracking_pid_tilt, kP, kI, kD, 150, 90);


    //subscriber=====================================================================
    gameSub = n.subscribe("gamecontroller", 100, gameCallback);
    visionSub = n.subscribe("robocup2019_vision", 100, visionCallback);
    imuSub = n.subscribe("imu", 100, imuCallback);
    motionSub = n.subscribe("motion_end", 100, motionCallback);
    udpSub = n.subscribe("udp_order", 100, udpCallback);
    localSub = n.subscribe("position", 100, localCallback);
    ikendSub = n.subscribe("ikend", 100, ikendCallback);
    //Publish========================================================================
    pantiltPub = n.advertise<msg_generate::pan_tilt_msg>("pantilt", 100);
    ikPub = n.advertise<msg_generate::ik_msg>("master2ik", 100);
    motionPub = n.advertise<msg_generate::motionNum_msg>("Motion", 100);
    udpPub = n.advertise<msg_generate::udp_msg>("udp_data", 100);

    //timer
    ros::Timer robocupTimer = n.createTimer(ros::Duration(0.01), robocupCallback);

    ros::spin();

    return 0;
}


void robocupCallback(const ros::TimerEvent&)
{
    //    cout<<"1.  target_block = "<<target_block<<endl;
    //    cout<<"2.  visionInfo.ballD   ======   "<<visionInfo.ballD<<endl;
    //    cout<<"3.  PAN_POSITION ===== "<<PAN_POSITION<<endl;
    //    cout<<"4.  wasPenalty ===== "<<wasPenalty<<endl;
    //    cout<<"5.  MoveDone ===== "<<MoveDone<<endl;
    //    cout<<"6.  my_turn_cnt ===== "<<my_turn_cnt<<endl;
    //    cout<<"7.  your_turn_cnt ===== "<<your_turn_cnt<<endl;
    //    cout<<"8.  isKeepGoing ===== "<<isKeepGoing<<endl;
    //    cout<<"9.  isSet    ========     "<<isSet<<endl<<endl;
    //    cout<<"9.  PAN_POSITION    ========     "<<PAN_POSITION<<endl;
    int TRacking_flag_C = Tracking(Tracking_thing_x, Tracking_point_x, Tracking_thing_y, Tracking_point_y);


    if(gameInfo.firstHalf)
        GOALPOST = gameInfo.firstside ? RIGHTSIDE : LEFTSIDE; // FIRSTHALF_GOAL;
    else
        GOALPOST = gameInfo.firstside ? LEFTSIDE : RIGHTSIDE; // SECONDHALF_GOAL;

    cout << "GOALPOST = " << GOALPOST << endl;

    /*For Test*/
    gameInfo.state = STATE_PLAYING;
    GOALPOST = FIRSTHALF_GOAL;
    //robocup_case = MODE_TEST;


    //get up=========================

    if(imuInfo.pitch > 60.0)
    {
        PAN_POSITION = 0;
        TILT_POSITION = -40;
        pantiltPub.publish(ptMsg);
    }
    if(imuInfo.pitch < -60.0)
    {
        PAN_POSITION = 0;
        TILT_POSITION = 0;
        pantiltPub.publish(ptMsg);
    }

    if(imuInfo.pitch > 60.0)
    {
        WALK_STOP;
        DXL_Motion(STAND_F);
        isFalldown = true;
    }
    else if(imuInfo.pitch < -60.0)
    {
        WALK_STOP;
        DXL_Motion(STAND_B);
        isFalldown = true;
    }
//    else if(imuInfo.roll > 70.0)
//    {
//        WALK_STOP;
//        DXL_Motion(STAND_R);
//        isFalldown = true;
//    }
//    else if(imuInfo.roll < -70.0)
//    {
//        WALK_STOP;
//        DXL_Motion(STAND_L);
//        isFalldown = true;
//    }

    //    if(robocup_case != temp_case)   //initialize
    //    {
    //        tracking_cnt = 0;
    //        no_ball_cnt = 0;
    //    }

    if(gameInfo.penalty)
    {
        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, ballTrackingPointX, ballTrackingPointY);
        wasPenalty = true;

        WALK_STOP;

        robocup_case = MODE_BALL_DETECT;
        isFalldown = false;
    }
    else
    {
        if(isFalldown)
        {
            cout<<"FALL DOWN!!!!!!!!!!!!!!!!!!!!"<<endl<<endl;
            if(motion_end)
            {
                robocup_case = temp_case;
                isFalldown = false;
                motion_end = 0;
            }
        }
        else{  // is not Falldown
            switch (gameInfo.state)
            {
            case STATE_INITIAL:
            {
                cout << "********************\n";
                cout << "   INITIAL\n";
                cout << "********************\n";

                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, 0, 0);

                isGone = false;
                isDone = false;
                isPlaying = false;

                wasInitial = true;

                break;
            }
            case STATE_READY:
            {
                cout << "********************\n";
                cout << "   READY\n";
                cout << "********************\n";

                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, 0, 0);


                static int targetYaw = 0;

                cout << "================ isGone  = " << isGone << endl;
                cout << "================ isDone  = " << isDone << endl;
                cout << "================ local   = " << local  << endl;

                ikMsg.flag = true;

                if(isPlaying)
                {
                    cout << "scored" << endl;

                    if(local)
                    {
                        if(!isDone)
                        {
                            if(!isGone)
                            {
                                static bool isRotated = false;
                                int nCol = (localInfo.robot_area - 1) % 6;

                                targetYaw = (-1) * GOALPOST;

                                if(GOALPOST > 0)
                                {
                                    if(nCol < 4)
                                    {
                                        if(!isRotated)
                                        {
                                            WALK_START(0, 0, targetYaw - localInfo.yaw);

                                            if(abs(ikMsg.Yaw) < 10)
                                            {
                                                WALK_START(0, 0, 0);
                                                isRotated = true;
                                            }
                                        }
                                        else
                                        {
                                            ikMsg.Yaw = abs(targetYaw - localInfo.yaw) > 15 ? (targetYaw - localInfo.yaw) : 0;

                                            ikMsg.X_length = abs(ikMsg.Yaw) > 15 ? 40 : 40;
                                            ikMsg.Y_length = 0;

                                            //                                            WALK_START(40, 0, 0);
                                        }
                                    }
                                    else
                                    {
                                        WALK_START(0, 0, 0);

                                        isGone = true;
                                        isRotated = false;
                                    }
                                }
                                else
                                {
                                    if(nCol > 1)
                                    {
                                        if(!isRotated)
                                        {
                                            WALK_START(0, 0, targetYaw - localInfo.yaw);

                                            if(abs(ikMsg.Yaw) < 10)
                                            {
                                                WALK_START(0, 0, 0);
                                                isRotated = true;
                                            }
                                        }
                                        else
                                        {
                                            ikMsg.Yaw = abs(targetYaw - localInfo.yaw) > 15 ? (targetYaw - localInfo.yaw) : 0;

                                            ikMsg.X_length = abs(ikMsg.Yaw) > 15 ? 40 : 40;
                                            ikMsg.Y_length = 0;
                                        }
                                    }
                                    else
                                    {
                                        WALK_START(0, 0, 0);

                                        isGone = true;
                                        isRotated = false;
                                    }
                                }

                                //                                if(!isRotated)
                                //                                {
                                //                                    WALK_START(0, 0, targetYaw - localInfo.yaw);

                                //                                    if(abs(ikMsg.Yaw) < 10)
                                //                                    {
                                //                                        WALK_START(0, 0, 0);
                                //                                        isRotated = true;
                                //                                    }
                                //                                }
                                //                                else
                                //                                {
                                //                                    int nCol = (localInfo.robot_area - 1) % 6;

                                //                                    if(GOALPOST > 0)
                                //                                    {
                                //                                        if(nCol < 4)
                                //                                        {
                                //                                            ikMsg.Yaw = abs(targetYaw - localInfo.yaw) > 15 ? (targetYaw - localInfo.yaw) : 0;

                                //                                            ikMsg.X_length = abs(ikMsg.Yaw) > 15 ? 20 : 40;
                                //                                            ikMsg.Y_length = 0;

                                ////                                            WALK_START(40, 0, 0);
                                //                                        }
                                //                                        else
                                //                                        {
                                //                                            WALK_START(0, 0, 0);

                                //                                            isGone = true;
                                //                                            isRotated = false;
                                //                                        }
                                //                                    }
                                //                                    else
                                //                                    {
                                //                                        if(nCol > 1)
                                //                                        {
                                //                                            ikMsg.Yaw = abs(targetYaw - localInfo.yaw) > 15 ? (targetYaw - localInfo.yaw) : 0;

                                //                                            ikMsg.X_length = abs(ikMsg.Yaw) > 15 ? 20 : 40;
                                //                                            ikMsg.Y_length = 0;
                                //                                        }
                                //                                        else
                                //                                        {
                                //                                            WALK_START(0, 0, 0);

                                //                                            isGone = true;
                                //                                            isRotated = false;
                                //                                        }
                                //                                    }
                                //                                }

                                //                                cout << "now block = " << localInfo.robot_area << endl;
                                //                                int target = GET_RETURN_BLOCK(localInfo.robot_area);

                                //                                cout << "return to : " << target << endl;

                                //                                if( MOVE(target) )  isGone = true;
                                //                                if(GOALPOST > 0)
                                //                                {
                                //                                    if(MOVE(17))    isGone = true;
                                //                                }
                                //                                else
                                //                                {
                                //                                    if(MOVE(14))    isGone = true;
                                //                                }
                                //                                if(gameInfo.kickoffTeam == ROBIT)
                                //                                {
                                //                                    if(GOALPOST > 0)
                                //                                    {
                                //                                        if(MOVE(16))    isGone = true;
                                //                                    }
                                //                                    else
                                //                                    {
                                //                                        if(MOVE(15))    isGone = true;
                                //                                    }
                                //                                }
                                //                                else
                                //                                {
                                //                                    if(GOALPOST > 0)
                                //                                    {
                                //                                        if(MOVE(17))    isGone = true;
                                //                                    }
                                //                                    else
                                //                                    {
                                //                                        if(MOVE(14))    isGone = true;
                                //                                    }
                                //                                }
                            }
                            else
                            {
                                targetYaw = GOALPOST;

                                WALK_START(0, 0, targetYaw - localInfo.yaw);

                                if(abs(ikMsg.Yaw) < 10)
                                {
                                    WALK_START(0, 0, 0);
                                    isDone = true;
                                }
                            }
                        }
                        else
                            ikMsg.flag = false;
                    }
                    else
                    {
                        if(gameInfo.firstHalf)
                        {
                            //                    if(gameInfo.kickoffTeam == ROBIT)
                            //                        targetYaw = FIRSTHALF_GOAL;
                            //                    else
                            //                        targetYaw = SECONDHALF_GOAL;
                            targetYaw = GOALPOST;
                        }
                        else
                        {
                            //                    if(gameInfo.kickoffTeam == ROBIT)
                            //                        targetYaw = SECONDHALF_GOAL;
                            //                    else
                            //                        targetYaw = FIRSTHALF_GOAL;
                            targetYaw = GOALPOST;
                        }

                        ikMsg.Y_length = 0;
                        ikMsg.Yaw = (targetYaw - localInfo.yaw);

                        ikMsg.X_length = (abs(ikMsg.Yaw) > 10 ? 15 : 40);
                    }
                }
                else
                {
                    if(local)
                    {
                        if(!isDone)
                        {
                            if(!isGone)
                            {
                                int target = ( (GOALPOST > 0) ? 16 : 15 )/*( (abs(localInfo.yaw) < 90 ) ? ( (GOALPOST > 0) ? 10 : 9 ) : ( (GOALPOST > 0) ? 22 : 21 ) )*/;

                                if( (MOVE(target) || gameInfo.readyTime < 5) && gameInfo.readyTime )
                                {
                                    isGone = true;
                                    isSelect = false;
                                    isSetYaw = false;
                                    isSamerow = false;
                                }
                                //                            if(GOALPOST > 0)
                                //                            {
                                //                                if(MOVE(16))    isGone = true;
                                //                            }
                                //                            else
                                //                            {
                                //                                if(MOVE(15))    isGone = true;
                                //                            }
                                //                            if(gameInfo.kickoffTeam == ROBIT)
                                //                            {
                                //                                if(GOALPOST > 0)
                                //                                {
                                //                                    if(MOVE(16))    isGone = true;
                                //                                }
                                //                                else
                                //                                {
                                //                                    if(MOVE(15))    isGone = true;
                                //                                }
                                //                            }
                                //                            else
                                //                            {
                                //                                if(GOALPOST > 0)
                                //                                {
                                //                                    if(MOVE(17))    isGone = true;
                                //                                }
                                //                                else
                                //                                {
                                //                                    if(MOVE(14))    isGone = true;
                                //                                }
                                //                            }
                            }
                            else
                            {
                                targetYaw = GOALPOST/* + 10 * ((abs(GOALPOST) < abs(localInfo.yaw)) ? )*/;

                                int turnYaw = ( abs(localInfo.yaw) < 90 ? (targetYaw - localInfo.yaw)
                                                                        : (localInfo.yaw*targetYaw < 0 ? (localInfo.yaw - targetYaw) : (targetYaw - localInfo.yaw)) );

                                cout << "turn Yaw = " << turnYaw << endl;

                                WALK_START(0, 0, turnYaw);

                                if(abs(turnYaw) < 10)
                                {
                                    WALK_START(0, 0, 0);
                                    isDone = true;
                                }
                            }
                        }
                        else
                            ikMsg.flag = false;
                    }
                    else
                    {
                        ikMsg.X_length = 40;
                        ikMsg.Y_length = 0;
                        ikMsg.Yaw = (-1) * (abs(localInfo.yaw) < 90 ? localInfo.yaw : (localInfo.yaw > 0 ? localInfo.yaw - 180 : localInfo.yaw + 180));

                        if(gameInfo.readyTime < 5)
                        {
                            ikMsg.X_length = 30;
                            ikMsg.Y_length = 0;

                            //                        if(gameInfo.firstHalf)
                            //                        {
                            //                            cout << "right" << endl;
                            //                            targetYaw = FIRSTHALF_GOAL;
                            //                        }
                            //                        else
                            //                        {
                            //                            cout << "left" << endl;
                            //
                            targetYaw = SECONDHALF_GOAL;
                            //                        }

                            targetYaw = GOALPOST;

                            ikMsg.Yaw = (targetYaw - localInfo.yaw);
                        }
                    }
                }

                if(ikMsg.X_length > 40) ikMsg.X_length = 40;

                if(ikMsg.Yaw > 10)        ikMsg.Yaw = 10;
                else if(ikMsg.Yaw < -10)  ikMsg.Yaw = -10;

                cout << "X = " << ikMsg.X_length << endl;
                cout << "Y = " << ikMsg.Y_length << endl;
                cout << "Yaw = " << ikMsg.Yaw << endl;

                break;
            }
            case STATE_SET:
            {
                cout << "********************\n";
                cout << "   SET\n";
                cout << "********************\n";

                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);

                ikMsg.flag = false;
                isGone = false;
                isDone = false;

                wasSet = true;

                if(gameInfo.kickoffTeam == ROBIT)
                {
                    motion_what = SHOOT_L;
                    robocup_case = MODE_KICK;
                }
                else
                    robocup_case = MODE_BALL_DETECT;

                no_ball_cnt = 0;
                tracking_cnt = 0;

                break;
            }
            case STATE_PLAYING:
            {
                cout << "********************\n";
                cout << "   PLAYING\n";
                cout << "********************\n";

                if(/*!isPlaying && */gameInfo.kickoffTeam != ROBIT && gameInfo.readyTime && !gameInfo.secondState)
                {
                    TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);


                    break;
                }

                isPlaying = true;


                //Robocup case=========================================================================================================

                //theta_ball_real
                theta_ball_real = localInfo.yaw + PAN_POSITION;
                if(theta_ball_real > 180) theta_ball_real -= 360;
                else if(theta_ball_real <= -180) theta_ball_real += 360;

                cout << "freekick =============== " << gameInfo.secondInfo[1] << endl;

                if(gameInfo.secondState != 0 && gameInfo.secondState != 2 && !gameInfo.secondInfo[1])
                {
                    TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);

                    WALK_STOP;

                    if(gameInfo.secondInfo[0] == ROBIT)
                        robocup_case = MODE_BALL_DETECT;
                    else
                        robocup_case = MODE_FREEKICK;
                }
                else if(gameInfo.secondState != 0 && gameInfo.secondState != 2 && gameInfo.secondInfo[1] == 2)
                {
                    TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);

                    WALK_STOP;
                }
                else
                {
                    switch (robocup_case)
                    {
                    case MODE_TEST:

                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_TEST  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);
                        cout<<"visionInfo.ballD: "<<visionInfo.ballD<<endl;
                        cout<<"no_ball_cnt: "<<no_ball_cnt<<endl;
                        cout<<"tracking_cnt: "<<tracking_cnt<<endl;

                        //TILT_POSITION = -20;
                        //WALK_START(0,0,0);
                        //DXL_Motion(SHOOT_L_SIDE);

//                        if(!TRacking_flag_C)
//                        {
//                            no_ball_cnt = 0;
//                            tracking_cnt++;


//                        }
//                        else if(TRacking_flag_C)
//                        {
//                            no_ball_cnt++;
//                            SET_YAW_PAN();
//                            //tracking_cnt = 0;
//                        }
//                        if(no_ball_cnt >= 100)
//                        {
//                            WALK_STOP;
//                            //no_ball_cnt = 0;
//                        }
//                        if(tracking_cnt>=150)
//                        {
//                            WALK_START(20,0,0);
//                            SET_YAW_PAN();
//                            if(visionInfo.ballD < 250)
//                            {
//                                cout<<"stop!!!!!!!!!!!!!"<<endl;
//                                WALK_STOP;
//                                tracking_cnt = 0;
//                                robocup_case = MODE_SET_Y;
//                            }
//                            //tracking_cnt = 0;
//                        }




//                        if(no_ball_cnt > 1000)
//                            WALK_STOP;





                        //                        if(!TRacking_flag_C)
                        //                        WALK_START(visionInfo.ballD*0.01,0,0);
                        //                        else
                        //                        WALK_STOP;
                        //                        TILT_POSITION = 20;
                        //                        PAN_POSITION = 0;
                        //                    int ball = 0;
                        //                    cin>>ball;
                        //                    if(localInfo.robot_area != 0)
                        //                         cout<<"""GET_RETURN_BLOCK(ball) =========== "<<GET_TARGET_BLOCK(ball)<<endl;

                        //WALK_START(40, 0, 6);


                        //                if(localInfo.robot_area != 0)
                        //                {
                        //                    if(MOVE(7))
                        //                    {
                        //                        robocup_case = MODE_WAIT;
                        //                    }
                        //                }
                        //robocup_case = MODE_BALL_DETECT;

                        break;
                    }

                    case MODE_BALL_DETECT:
                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_BALL_DETECT  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);
                        WALK_STOP;
                        isNoball = false;
                        noball_time = 0;

                        if(TRacking_flag_C == 0) //See Ball...............................
                            tracking_cnt++;
                        else //Don't See Ball
                        {
                            tracking_cnt = 0;
                            no_ball_cnt++;
                            cout<<"noballcnt == "<<no_ball_cnt<<endl;

                            if(no_ball_cnt > DETECT2NO_BALL_CNT && tracking_cnt < FIND_BALL_CNT-30) //go to MODE_NO_BALL
                            {
                                robocup_case = MODE_NO_BALL;
                                no_ball_cnt = 0;
                            }
                        }

                        if(tracking_cnt > FIND_BALL_CNT)   //Find Ball...............................
                        {
                            //Initialize
                            tracking_cnt = 0;
                            no_ball_cnt = 0;
                            //Change Case
                            robocup_case = MODE_BALL_WALK;
                        }


                        break;
                    }

                    case MODE_NO_BALL:
                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_NO_BALL  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                        cout<<"isTeamplay ==   "<<isTeamplay<<endl;
                        Matrix rb_area = ROBOT_WHERE(localInfo.robot_area);
                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);

                        static int Turn_cnt = 0;
                        static int Forward_cnt = 0;

                        static int dir = 0;


                        if(!TRacking_flag_C) // See ball!!!
                        {
                            //WALK_STOP;
                            WALK_START(0,0,0);
                            tracking_cnt++;
                        }
                        else
                        {
                            tracking_cnt = 0;
                            no_ball_cnt++;
                        }

                        if(wasPenalty || wasInitial || wasSet)
                        {
                            WALK_START(10,0,0);
                            Forward_cnt++;

                            if(Forward_cnt > 2000)
                            {
                                WALK_START(0,0,0);
                                Forward_cnt = 0;
                                wasPenalty = false; wasInitial = false; wasSet = false;
                            }
                        }

                        if(isScanfinish && (!wasPenalty && !wasInitial && !wasSet) && tracking_cnt < FIND_BALL_CNT)
                        {
                            if(noball_time < 4000)
                            {
                                isNoball = true;
                                WALK_START(0,0,8);
                                noball_time++;
                            }
                            else if(noball_time >= 4000)
                            {
                                isNoball = false;
                                if(isScanfinish)
                                {
                                    WALK_START(0,0,8);
                                    Turn_cnt++;
                                }
                                if(isScanfinish && Turn_cnt > TURN_CNT)
                                {
                                    WALK_START(5,0,0);
                                    isScanfinish = false;
                                    Turn_cnt = 0;
                                }
                            }


                            //                            WALK_START(0,0,8);
                            //                            Turn_cnt++;
                            //                            noball_time++;


                            //                            if(noball_time < 4000 && Turn_cnt > TURN_CNT)
                            //                            {
                            //                                WALK_STOP;
                            //                                //WALK_START(0,0,0);
                            //                                isScanfinish = false;
                            //                                Turn_cnt = 0;
                            //                            }

                            //                            if(noball_time >= 4000 && Turn_cnt > TURN_CNT)
                            //                            {
                            //                                WALK_START(40,0,0);
                            //                                isScanfinish = false;
                            //                                Turn_cnt = 0;

                            //                            }
                        }


                        if(tracking_cnt > FIND_BALL_CNT) // Find Ball................................
                        {
                            //WALK_STOP;
                            WALK_START(0,0,0);
                            tracking_cnt = 0;
                            no_ball_cnt = 0;
                            Forward_cnt = 0;
                            Turn_cnt = 0;
                            wasPenalty = false; wasInitial = false; wasSet = false;
                            robocup_case = MODE_BALL_WALK;
                            isNoball = false;
                        }

                        break;
                    }


                    case MODE_BALL_WALK:
                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_BALL_WALK  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);

                        int goal_yaw = GOALPOST; /*GET_GOALPOST_DR(localInfo.robot_area);*//*GOALPOST;*/
                        Goal_zeroyaw = localInfo.yaw - goal_yaw;
                        if(Goal_zeroyaw > 180) Goal_zeroyaw -= 360;
                        else if(Goal_zeroyaw <= -180) Goal_zeroyaw += 360;

                        wasInitial = false; wasSet = false; wasPenalty = false;


                        if(TRacking_flag_C)// IF LOST BALL
                        {
                            no_ball_cnt++;

                            if(no_ball_cnt >= LOST_BALL_CNT)
                            {
                                WALK_STOP;
                                no_ball_cnt = 0;
                                isSet = false;
                                robocup_case = MODE_BALL_DETECT;
                            }
                        }

                        if(visionInfo.ballD != 0)
                        {
                            no_ball_cnt = 0;
                            if((isSet||abs(Goal_zeroyaw) < GOALPOST_ERROR)&&visionInfo.ballD <= Walk_BallD) //TURN FINISH
                            {
                                isSet = true;

                                if(visionInfo.ballD < DesireBallD_SHOOT && visionInfo.ballD > 0 && visionInfo.ballX != 0)//KICK
                                {
                                    cout<<endl<<"+++++++++TURN FINISH++++++++++"<<endl<<endl;
                                    lost_ball_count = 0;
                                    isSet = false;
                                    robocup_case = MODE_SET_Y;    //MODE_SET_Y
                                }
                                else if(visionInfo.ballD > DesireBallD_SHOOT && visionInfo.ballX != 0)
                                {
                                    WALK_START(5, 0, 0);

                                    if(visionInfo.ballD < DesireBallD_SHOOT+80 && visionInfo.ballD > DesireBallD_SHOOT)
                                    {
                                        WALK_START(visionInfo.ballD*0.01, 0, 0);//0.08
                                    }
                                    PAN_POSITION_ERROR = 0;
                                    SET_YAW_PAN();

                                }
                            }
                            else //TURN TO SET GOALPOST DIRECTION
                            {
                                lost_ball_count = 0;

                                if(visionInfo.ballD > Walk_BallD)
                                {
                                    WALK_START(10,0,0);
                                }
                                else
                                {
                                    //turn
                                    if(abs(theta_ball_real - goal_yaw) < 10 && PAN_POSITION > 30)
                                    {
                                        WALK_START(0,0,0);
                                        desire_yaw = goal_yaw;
                                        SET_YAW_YAW();
                                    }
                                    else if(abs(Goal_zeroyaw) > GOALPOST_ERROR)
                                    {
                                        if(Goal_zeroyaw <= 0)    //+
                                            WALK_START(0,-15,0);//-20 3
                                        else if(Goal_zeroyaw > 0)  //-
                                            WALK_START(0,15,-0);//17 -4
                                    }

                                    //SET BALLDISTANCE
                                    if(visionInfo.ballD > DesireBallD_SHOOT+50)
                                        ikMsg.X_length =  visionInfo.ballD*0.01;
                                    else if(visionInfo.ballD < DesireBallD_SHOOT-50)
                                        ikMsg.X_length = -visionInfo.ballD*0.01;

                                }

                                //limit................................
                                if(ikMsg.X_length > X_LIMIT) ikMsg.X_length = X_LIMIT;
                                else if(ikMsg.X_length < -(X_LIMIT-5)) ikMsg.X_length = -(X_LIMIT-5);

                                PAN_POSITION_ERROR = 0;
                                SET_YAW_PAN();
                            }

                            if(visionInfo.ballD > DesireBallD + 400 && visionInfo.ballD != 0 || abs(Goal_zeroyaw) > 30)
                                isSet = false;
                        }

                    }
                        break;


                    case MODE_SET_Y:
                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_SET_Y  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);
                        isNoball = false;
                        double PANPOS_correct = visionInfo.ballT + PAN_POSITION;
                        cout<<"PANPOS_correct   ==========  "<<PANPOS_correct<<endl;
                        cout<<"motion_what    =======       "<<motion_what<<endl;
                        static int set_cnt = 0;
                        set_cnt++;
                        cout<<"((((((((((((((((  SET CNT  )))))))))))))))       "<<set_cnt<<endl;

                        if(TRacking_flag_C) //Lost ball
                        {
                            if(no_ball_cnt++ >= LOST_BALL_CNT)
                            {
                                WALK_STOP;
                                no_ball_cnt = 0;
                                //robocup_case = MODE_BALL_DETECT;
                                robocup_case = MODE_TEST;
                            }
                        }

                        if(visionInfo.ballD > DesireBallD_SHOOT)
                        {
                            WALK_START(visionInfo.ballD*0.01, 0, 0);//0.08

                            PAN_POSITION_ERROR = 0;
                            SET_YAW_PAN();
                        }
                        else
                        {
                            if(PANPOS_correct >= -10 && visionInfo.ballD != 0) //BALL ROBOT
                            {
                                if(set_cnt > SET_CNT)
                                {
                                    motion_what = SHOOT_L;
                                    robocup_case = MODE_KICK;
                                    WALK_STOP;
                                    set_cnt = 0;
                                }
                                else if(abs(DesirePP_L - PAN_POSITION) < 13)
                                {
                                    motion_what = SHOOT_L;
                                    robocup_case = MODE_KICK;
                                    WALK_STOP;
                                    set_cnt = 0;
                                }
                                else if(PAN_POSITION < DesirePP_L - 13)
                                {
                                    WALK_START(0,-8,0);
                                }

                                else if(PAN_POSITION > DesirePP_L + 13)
                                {
                                    WALK_START(0,8,0);
                                }
                            }
                            else if(PANPOS_correct < -10  && visionInfo.ballD != 0)
                            {
                                if(set_cnt > SET_CNT)
                                {
                                    motion_what = SHOOT_R;     //SHOOT_R
                                    robocup_case = MODE_KICK;
                                    WALK_STOP;
                                    set_cnt = 0;

                                }
                                else if(abs(DesirePP_R - PAN_POSITION) < 13)
                                {
                                    motion_what = SHOOT_R;     //SHOOT_R
                                    robocup_case = MODE_KICK;
                                    WALK_STOP;
                                    set_cnt = 0;
                                }
                                else if(PAN_POSITION < DesirePP_R - 13)
                                {
                                    WALK_START(0,-8,0);
                                }
                                else if(PAN_POSITION > DesirePP_R + 13)
                                {
                                    WALK_START(0,8,0);
                                }
                            }
                        }



                        break;
                    }

                    case MODE_KICK:
                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_KICK  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                        //TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);
                        static int Kick_cnt = 0;
                        Kick_cnt++;
                        PAN_POSITION = 0;
                        TILT_POSITION = -60;
                        WALK_STOP;
                        //scan_start = true;
                        if((ik_end || Kick_cnt > 500))
                        {
                            DXL_Motion(motion_what);
                            robocup_case = MODE_STOP;
                            scan_start = true;
                            ik_end = 0;
                            Kick_cnt = 0;
                        }


                        break;
                    }

                    case MODE_MOVE:{

                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE MOVE  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                        cout<<"------------target block = "<<target_block <<endl;
                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);
                        isNoball = false;


                        if(TRacking_flag_C == 0) //See Ball...............................
                            tracking_cnt++;
                        else //Don't See Ball
                        {
                            tracking_cnt = 0;
                        }



                        if(visionInfo.ballD != 0 && tracking_cnt > FIND_BALL_CNT && visionInfo.ballD < MOVE_DISTANCE + 30) // Find Ball
                        {
                            if(visionInfo.ballD >= MOVE_DISTANCE)
                            {
                                tracking_cnt = 0;
                                MoveDone = true;
                                robocup_case = MODE_BALL_DETECT;
                            }
                            else if(visionInfo.ballD < MOVE_DISTANCE)
                            {
                                WALK_START(-10, 0, 0);
                            }
                        }
                        else
                        {
                            if(MOVE(target_block))
                            {
                                tracking_cnt = 0;
                                MoveDone = true;
                                robocup_case = MODE_BALL_DETECT;
                            }
                        }

                        if(isKeepGoing)
                            robocup_case = MODE_BALL_DETECT;

                        break;


                    }
                    case MODE_FAR_BALL:
                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_FAR_BALL  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);
                        isNoball = false;


                        if(TRacking_flag_C)
                        {
                            no_ball_cnt++;

                            if(no_ball_cnt >= LOST_BALL_CNT)
                            {
                                no_ball_cnt = 0;
                                isFar = true;
                                MoveDone = false;   //here
                                robocup_case = MODE_BALL_DETECT;
                                WALK_STOP;
                            }
                        }

                        if(visionInfo.ballD > 0)
                        {
                            if(abs(visionInfo.ballD - FAR_DISTANCE) < FAR_DISTANCE_ERROR)
                                WALK_STOP;
                            else if(visionInfo.ballD > FAR_DISTANCE + FAR_DISTANCE_ERROR)
                                WALK_START(10,0,0);
                            else if(visionInfo.ballD < FAR_DISTANCE - FAR_DISTANCE_ERROR)
                                WALK_START(-10, 0, 0);

                            PAN_POSITION_ERROR = 0;
                            SET_YAW_PAN();
                        }

                        if(isKeepGoing)
                            robocup_case = MODE_BALL_WALK;

                        break;
                    }



                    case MODE_WAIT:{
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_WAIT  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);
                        isNoball = false;


                        Matrix rb_area = ROBOT_WHERE(localInfo.robot_area);
                        Matrix target;

                        target.row = ((rb_area.row > 3) ? rb_area.row - 1 : rb_area.row + 1);
                        target.col = ((GOALPOST > 0) ? rb_area.col - 1 : rb_area.col + 1);
                        if(target.row <= 1 || target.row >= 6)
                            target.row = ((target.row <= 1) ? 1 : 6);

                        int target_ball_yaw = (int)(atan2((double)((6-target.row) - (6-rb_area.row)),(double)(target.col - rb_area.col)) * 180/3.141592) - 90;
                        if(target_ball_yaw >= 180)  target_ball_yaw -= 360;
                        else if(target_ball_yaw < -180) target_ball_yaw += 360;

                        int New_yaw_ball = localInfo.yaw - target_ball_yaw;
                        if(New_yaw_ball >= 180) New_yaw_ball -= 360;
                        else if(New_yaw_ball < -180) New_yaw_ball += 360;

                        target_ball_yaw = GOALPOST;


                        if(rb_area.row != 3 && abs(localInfo.yaw - target_ball_yaw) > YAW_ERROR && !isFar)
                        {
                            if(New_yaw_ball <= 0)    //+
                                WALK_START(0,0,GET_YAW(target_ball_yaw));
                            else if(New_yaw_ball >= 0)  //-
                                WALK_START(0,0,-GET_YAW(target_ball_yaw));
                        }
                        else
                            WALK_STOP;

                    }

                        break;

                    case MODE_STOP:
                    {
                        cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_STOP  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                        static int fuck_stop = 0;
                        fuck_stop++;
                        tracking_cnt=0;
                        if(motion_end || fuck_stop > 1000)
                        {
                            //robocup_case = MODE_BALL_DETECT;
                            robocup_case = MODE_TEST;
                            motion_end = 0;
                            fuck_stop = 0;
                        }

                        break;
                    }

                    case MODE_FREEKICK: {
                        // TODO
                        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.ballX, visionInfo.ballY);

                        if(TRacking_flag_C == 0) //See Ball...............................
                            tracking_cnt++;
                        else
                            tracking_cnt = 0;

                        if(tracking_cnt > FIND_BALL_CNT)   //Find Ball...............................
                        {
                            if(visionInfo.ballD != 0)
                            {
                                if(visionInfo.ballD > 1500)
                                    WALK_STOP;
                                else
                                    WALK_START(-20,0,0);
                            }
                        }

                        if(!gameInfo.secondState)
                        {
                            WALK_STOP;
                            tracking_cnt = 0;
                            robocup_case = MODE_BALL_DETECT;
                        }
                    }
                        break;

                    default:
                        break;

                    }

                }

                break;
            }
                //===============================================================================================================

            case STATE_FINISHED:
                cout << "********************\n";
                cout << "1. FINISHED\n";
                cout << "********************\n";

                isGone = false;
                isDone = false;
                isPlaying = false;

                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, 0, 0);

                WALK_STOP;

                break;
            default:
                break;

                //past-----------------------------------
                gameInfo.stateBefore = gameInfo.state;
                temp_case = robocup_case;
                yourCase_past = udpInfo.yourCase;
            }}

    }

    ptMsg.Angle_Yaw = PAN_POSITION;
    ptMsg.Angle_Pitch = TILT_POSITION;
    udpMsg.ballDist = visionInfo.ballD;
    udpMsg.field_area = localInfo.robot_area;
    udpMsg.ball_area = localInfo.ball_area;
    udpMsg.robocup_case = robocup_case;

    if(gameInfo.state == STATE_PLAYING && !gameInfo.penalty  && !isFalldown)
        udpMsg.robocup_case = robocup_case;
    else
        udpMsg.robocup_case = 0;

    //    if(abs(ikMsg.Y_length) > 10)
    //    {
    //        if(ikMsg.X_length > 15)    ikMsg.X_length = 15;
    //        else if(ikMsg.X_length < -10)   ikMsg.X_length = -10;
    //    }

    //    //    if(abs(ikMsg.Y_length) > 10)
    //    //    {
    //    //        if(ikMsg.Yaw >= 6) ikMsg.Yaw = 6;
    //    //        if(ikMsg.Yaw <= -6) ikMsg.Yaw = -6;

    //    //    }

    ////    if(abs(ikMsg.Yaw) >= 5)
    ////    {
    ////        if(ikMsg.Y_length >= 10) ikMsg.Y_length = 10;
    ////        else if(ikMsg.Y_length <= -10)  ikMsg.Y_length = -10;
    ////    }

    //    if(abs(ikMsg.X_length) >= 40)
    //    {
    //        if(ikMsg.Yaw >= 6) ikMsg.Yaw = 6;
    //        else if(ikMsg.Yaw <= -6)  ikMsg.Yaw = -6;
    //    }

    if(abs(ikMsg.X_length)>=15)
    {
        if(ikMsg.X_length>=15) ikMsg.X_length=15;
        else if(ikMsg.X_length<=-15) ikMsg.X_length = -15;

        if(ikMsg.Y_length>=15) ikMsg.Y_length=15;
        else if(ikMsg.Y_length<=-15) ikMsg.Y_length = -15;
    }

    if(abs(ikMsg.Y_length)>=20)
    {
        if(ikMsg.Y_length>=20) ikMsg.Y_length=20;
        else if(ikMsg.Y_length<=-20) ikMsg.Y_length = -20;
    }

    if(abs(ikMsg.Yaw)>=10)
    {
        if(ikMsg.Yaw>=10) ikMsg.Yaw=10;
        else if(ikMsg.Yaw<=-10) ikMsg.Yaw = -10;
    }

    pantiltPub.publish(ptMsg);
    ikPub.publish(ikMsg);
    udpPub.publish(udpMsg);
}


void SET_YAW_PAN()
{
    if(PAN_POSITION > PAN_POSITION_ERROR) //BALL ROBOT //ccw
    {
        ikMsg.Yaw = +(fabs(PAN_POSITION - PAN_POSITION_ERROR) * 0.3);
    }
    else if(PAN_POSITION < PAN_POSITION_ERROR) //ROBOT BALL //cw
    {
        ikMsg.Yaw = -(fabs(PAN_POSITION - PAN_POSITION_ERROR) * 0.3);
    }
    else
    {
        ikMsg.Yaw = 0;
    }

    //limit................................
    if(ikMsg.Yaw > YAW_LIMIT) ikMsg.Yaw = YAW_LIMIT;
    else if(ikMsg.Yaw < -YAW_LIMIT) ikMsg.Yaw = -YAW_LIMIT;
}

void SET_YAW_YAW()
{
    static int diff_yaw;

    diff_yaw = localInfo.yaw - desire_yaw;
    if(diff_yaw > 180)   diff_yaw -= 360;
    else if(diff_yaw <= -180)   diff_yaw += 360;

    if(diff_yaw > 0) //BALL ROBOT //ccw
    {
        ikMsg.Yaw = -(abs(localInfo.yaw - desire_yaw) * 0.4);
    }
    else if(diff_yaw <= 0) //ROBOT BALL //cw
    {
        ikMsg.Yaw = +(abs(localInfo.yaw - desire_yaw) * 0.4);
    }
    else
    {
        ikMsg.Yaw = 0;
    }

    //limit................................
    if(ikMsg.Yaw > YAW_LIMIT) ikMsg.Yaw = YAW_LIMIT;
    else if(ikMsg.Yaw < -YAW_LIMIT) ikMsg.Yaw = -YAW_LIMIT;
}


int GET_YAW(int yaw_target)
{
    int yaw_Diff = abs(yaw_target - localInfo.yaw);
    int yaw_return = 0;

    yaw_return = yaw_Diff*0.3;

    //limit................................
    if(yaw_return > YAW_LIMIT) yaw_return = YAW_LIMIT;
    else if(yaw_return < -YAW_LIMIT) yaw_return = -YAW_LIMIT;

    return yaw_return;
}


int GET_GOALPOST_DR(int rb_block)
{
    Matrix rb = ROBOT_WHERE(rb_block);
    int goalpost_dr = 0;

    if(rb.row == 1)
    {
        if(GOALPOST == RIGHTSIDE)
            goalpost_dr = 105;/*100;*//*(int)(180 - ((10 + (10 / rb.col)) * rb.col));*/
        else if(GOALPOST == LEFTSIDE)
            goalpost_dr = -105;/*-100;*//*(int)(-180 + ((10 + (10 / (7-rb.col))) * (7-rb.col)));*/
    }
    //    else if(rb.row == 2)
    //    {
    //        if(GOALPOST == FIRSTHALF_GOAL)
    //            goalpost_dr = 100;//goalpost_dr = (int)(180 - ((13 + (10 / rb.col)) * rb.col));
    //        else if(GOALPOST == SECONDHALF_GOAL)
    //            goalpost_dr = -100;//goalpost_dr = (int)(-180 + ((13 + (10 / (7-rb.col))) * (7-rb.col)));
    //    }
    //    else if(rb.row == 3)
    //    {
    //        goalpost_dr = GOALPOST;
    //    }
    //    else if(rb.row == 4)
    //    {
    //        if(GOALPOST == FIRSTHALF_GOAL)
    //            goalpost_dr = 80;//goalpost_dr = (int)((13 + (10 / rb.col)) * rb.col);
    //        else if(GOALPOST == SECONDHALF_GOAL)
    //            goalpost_dr = -80;//goalpost_dr = -(int)((13 + (10 / (7-rb.col))) * (7-rb.col));
    //    }
    else if(rb.row == 5)
    {
        if(GOALPOST == RIGHTSIDE)
            goalpost_dr = 75;/*80;*//*(int)((10 + (10 / rb.col)) * rb.col);*/
        else if(GOALPOST == LEFTSIDE)
            goalpost_dr = -75;/*-80;*//*-(int)((10 + (10 / (7-rb.col))) * (7-rb.col));*/
    }
    else
    {
        goalpost_dr = GOALPOST;
    }

    return goalpost_dr;
}

//===Team Play========================================
bool MOVE(int _target_block)
{
    if(localInfo.robot_area != 0)
    {
        Matrix start;
        Matrix target;
        Matrix now;

        static int target_yaw = 0;
        static int walk_cnt = 0;
        static bool isSave = false;
        cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$44targey    =="  <<_target_block<<endl<<endl;

        if(!isSave)
        {
            start = ROBOT_WHERE(localInfo.robot_area);
            isSave = true;
        }

        cout<<"isSamerow  ==  "<<isSamerow<<endl;
        cout<<"isSelect  ==  "<<isSelect<<endl;

        target = ROBOT_WHERE(_target_block);
        now = ROBOT_WHERE(localInfo.robot_area);
        cout<<"now.col = "<<now.col <<"   now.row = "<<now.row<<endl;
        cout<<"target.col = "<<target.col <<"   target.row = "<<target.row<<endl;



        //First, Get target yaw
        if(!isSelect && !isSamerow)
        {


            if(target.col == now.col)
            {
                target_yaw = ((target.row > now.row) ? -180 : 0);
            }
            else
            {
                target_yaw = (int)(atan2(((6-target.row) - (6-now.row)),(target.col - now.col)) * 180/3.141592) - 90;
                if(target_yaw > 180) target_yaw -= 360;
                else if(target_yaw < -180) target_yaw += 360;
            }


            if(now.row == target.row)
                isSamerow = true;

            isSelect = true;
        }
        else if(isSelect && !isSamerow)  //Second, Turn to target yaw
        {
            cout<<"target_yaw == "<<target_yaw<<endl;
            cout<<"localInfo.yaw == "<<localInfo.yaw<<endl;
            int New_yaw = localInfo.yaw - target_yaw;
            if(New_yaw < -180)
                New_yaw += 360;
            else if(New_yaw > 180)
                New_yaw -= 360;

            cout<<"New_yaw =============== "<<New_yaw<<endl;

            cout<<"abs(localInfo.yaw - target_yaw)  === "<<abs(localInfo.yaw - target_yaw)<<endl;
            if(abs(localInfo.yaw - target_yaw) > YAW_ERROR)
            {
                if(!isSetYaw)
                {
                    if(New_yaw <= 0)    //+
                        WALK_START(0,0,GET_YAW(target_yaw));
                    else if(New_yaw >= 0)  //-
                        WALK_START(0,0,-GET_YAW(target_yaw));
                }
                else
                {
                    if(New_yaw <= 0)    //+
                        ikMsg.Yaw = GET_YAW(target_yaw);
                    else if(New_yaw >= 0)  //-
                        ikMsg.Yaw = -GET_YAW(target_yaw);
                }
            }
            else
            {
                isSetYaw = true;
                ikMsg.Yaw = 0;
            }

            if(isSetYaw)
                WALK_START(40,0,ikMsg.Yaw);


            if(now.row == target.row)
            {
                if(walk_cnt++ > WALK_CNT)
                {
                    isSetYaw = false;
                    isSamerow = true;
                    walk_cnt = 0;
                }
            }


        }
        else //now.row == target.row
        {
            if(now.row != target.row)
            {
                isSelect =false;
                isSamerow = false;
            }

            if(now.col > target.col)
                target_yaw = 90;
            else if(now.col < target.col)
                target_yaw = -90;

            int New_yaw = localInfo.yaw - target_yaw;
            if(New_yaw < -180)
                New_yaw += 360;
            else if(New_yaw > 180)
                New_yaw -= 360;


            if(abs(localInfo.yaw - target_yaw) > YAW_ERROR)
            {
                if(!isSetYaw)
                {
                    if(New_yaw <= 0)    //+
                        WALK_START(0,0,GET_YAW(target_yaw));
                    else if(New_yaw >= 0)  //-
                        WALK_START(0,0,-GET_YAW(target_yaw));
                }
                else
                {
                    if(New_yaw <= 0)    //+
                        ikMsg.Yaw = GET_YAW(target_yaw);
                    else if(New_yaw >= 0)  //-
                        ikMsg.Yaw = -GET_YAW(target_yaw);
                }
            }
            else
            {
                isSetYaw = true;
                ikMsg.Yaw = 0;
            }

            if(isSetYaw)
                WALK_START(40,0,ikMsg.Yaw);
        }



        //-----------------------------------------------------------
        if(_target_block == localInfo.robot_area)
        {
            isSelect = false;
            isSamerow = false;
            isSetYaw = false;
        }
        if(_target_block == localInfo.robot_area
                /*||((now.row ==1 || now.row == 5 || now.col == 1 || now.col == 6) && (start.row != 1 && start.row != 5 && start.col != 1 && start.col != 6))*/)
            return true;
        else
            return false;
    }
    else
        return false;
}

int GET_TARGET_BLOCK(int ball_block)

{
    Matrix robot;
    Matrix ball;

    robot = ROBOT_WHERE(localInfo.robot_area);
    ball = ROBOT_WHERE(ball_block);

    if(((GOALPOST > 0) ? robot.col <= ball.col : robot.col >= ball.col)) //goal|robot|ball
    {
        if(robot.col == ball.col)
            robot.col = ((GOALPOST > 0) ? robot.col - 1 : robot.col + 1);


        //        if(abs(robot.row - ball.row) > 2)
        //            robot.row = (robot.row <= ball.row ?  robot.row -1 : robot.row + 1);

        //        else if(robot.row == ball.row)
        //            robot.row = ((robot.row >= 4) ? robot.row - 1 : robot.row + 1);
    }

    else if(((GOALPOST > 0) ? robot.col > ball.col : robot.col < ball.col))      //goal|ball|robot
    {
        if(abs(robot.col - ball.col) > 2)
            robot.col = ((GOALPOST > 0) ? robot.col - 2 : robot.col + 2);
    }

    //--limit
    if(robot.col <= 1 || robot.col >= 6)
        robot.col = ((robot.col <= 1) ? 2 : 5);
    if(robot.row <= 1 || robot.row >= 5)
        robot.row = ((robot.row <= 1) ? 2 : 4);

    return Matrix2Block(robot);
}



int GET_RETURN_BLOCK(int block)
{
    int nowBlock = block - 1;

    int nowRow = nowBlock / 6;
    int nowCol = nowBlock % 6;

    int targetRow = nowRow;
    int targetCol = ( (GOALPOST > 0) ? 4 : 1 );

    int targetBlock = targetRow * 6 + targetCol;

    return (targetBlock + 1);
}

Matrix ROBOT_WHERE(int block)
{
    Matrix robot_mx;

    robot_mx.row = (int)((block-1)/6) + 1;
    robot_mx.col = ((block%6 == 0)? 6 : block%6);

    cout<<"robot_mx.row == "<<robot_mx.row <<endl;
    cout<<"robot_mx.col == "<<robot_mx.col <<endl;

    return robot_mx;
}

int Matrix2Block(Matrix mat)
{
    int block = ((mat.row-1)*6) + mat.col;
    return block;
}

//Order function=============================================================================================================

void TRACKING_WHAT(int TrackingPoint_x, int TrackingPoint_y, double TrackingThing_x, double TrackingThing_y)
{
    Tracking_point_x = TrackingPoint_x;
    Tracking_point_y = TrackingPoint_y;
    Tracking_thing_x = TrackingThing_x;
    Tracking_thing_y = TrackingThing_y;
}

void WALK_START(int x, int y, int yaw)
{
    ikMsg.flag = true;

    ikMsg.X_length = x;
    ikMsg.Y_length = y;
    ikMsg.Yaw = yaw;
}

void DXL_Motion(unsigned char Motion_Num)
{
    msg_generate::motionNum_msg motion_msg;
    motion_msg.Motion_Mode = 1;
    motion_msg.Motion_Num = Motion_Num;
    motionPub.publish(motion_msg);
}


//Callback function===========================================================================================================

void gameCallback(const msg_generate::robocupcontroller::ConstPtr &msg)
{
    gameInfo.firstHalf = msg->firstHalf;
    gameInfo.firstside = msg->firstside;
    gameInfo.kickoffTeam = msg->kickoffTeam;
    gameInfo.state = msg->state;
    gameInfo.readyTime = msg->readyTime;
    gameInfo.penalty = msg->penalty;
    gameInfo.secondState    = msg->secondState;

    for(int i = 0; i < 2; i++)
        gameInfo.secondInfo[i] = msg->secondInfo[i];
}

void ikendCallback(const msg_generate::ikend_msg::ConstPtr &msg)
{
    ik_end = msg->ikend;
}

void localCallback(const msg_generate::position_msg::ConstPtr &msg)
{
    localInfo.robot_area = msg->robot_area;
    localInfo.ball_area = msg->ball_area;
    localInfo.yaw = msg->yaw;
}

void visionCallback(const msg_generate::robocupvision::ConstPtr &msg)
{
    visionInfo.ballX = msg->ballX;
    visionInfo.ballY = msg->ballY;
    visionInfo.ballD = msg->ballDist;
    visionInfo.ballT = msg->ballTheta;
}

void imuCallback(const msg_generate::imu_msg::ConstPtr &msg)
{
    imuInfo.pitch = msg->pitch;
    imuInfo.roll = msg->roll;
}

void motionCallback(const msg_generate::motion_end::ConstPtr &msg)
{
    motion_end = msg->motion_end;
}

void udpCallback(const msg_generate::udp_order::ConstPtr &msg)
{
    udpInfo.yourBalldist = msg->yourBalldist;
    udpInfo.yourArea = msg->yourArea;
    udpInfo.ballArea = msg->ball_area;
    udpInfo.yourCase = msg->yourCase;

    if(udpInfo.yourCase == 0)
    {
        isKeepGoing = true;
        MoveDone = false;
    }
    else
        isKeepGoing = false;



    if(udpInfo.yourCase != 0 && localInfo.robot_area != 0)
    {
        if(robocup_case >= MODE_NO_BALL && !wasPenalty && !wasSet) // more than MODE_NO_BALL
        {
            if(udpInfo.yourCase == robocup_case && robocup_case == MODE_BALL_WALK) // both walk
            {
                if(visionInfo.ballD <= udpInfo.yourBalldist) //my turn
                {
                    my_turn_cnt++;
                    your_turn_cnt = 0;
                }
                else
                {
                    your_turn_cnt++;
                    my_turn_cnt = 0;
                }

                if(my_turn_cnt > 50)
                {
                    cout<<"walk keep going~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl<<endl;
                    my_turn_cnt = 0;
                    your_turn_cnt = 0;
                }
                if(your_turn_cnt > 50)
                {
                    robocup_case = MODE_FAR_BALL;  //SAME MODE_BALL_WALK (Korea robocup)
                    my_turn_cnt = 0;
                    your_turn_cnt = 0;
                }
            }
            else if(robocup_case <= MODE_BALL_DETECT && udpInfo.yourCase >= MODE_BALL_WALK && !MoveDone)   //only me bbackdaegal
            {
                if(udpInfo.yourBalldist < 2000)
                    your_turn_cnt++;
                else
                    your_turn_cnt = 0;

                if(your_turn_cnt > 50)
                {
                    target_block = GET_TARGET_BLOCK(udpInfo.yourArea);  //need top change
                    robocup_case = MODE_MOVE;

                    your_turn_cnt = 0;
                }
            }
            else if(robocup_case >= MODE_BALL_WALK && udpInfo.yourCase >= MODE_BALL_WALK)   //both of us genious
            {
                if(robocup_case < udpInfo.yourCase)    //You're better
                {
                    if(udpInfo.yourBalldist < FAR_DISTANCE)
                    {
                        robocup_case = MODE_FAR_BALL;
                    }
                }
            }
        }

        yourCase_past = temp_udpcase;

        if((robocup_case <= MODE_WAIT || MoveDone) && (yourCase_past > MODE_BALL_DETECT) && (udpInfo.yourCase < MODE_BALL_WALK))  //Your case change
        {
            no_ball_cnt = 0;
            tracking_cnt = 0;
            isFar = false;
            MoveDone = false;
            robocup_case = MODE_BALL_DETECT;

            if(robocup_case == MODE_FAR_BALL)
                robocup_case = MODE_BALL_WALK;
        }

        temp_udpcase = udpInfo.yourCase;
    }
}

//Tracking====================================================================================================================
int Tracking(double now_x, double X_POINT_STANDARD, double now_y, double Y_POINT_STANDARD)
{
    int Tracking_flag = 1;
    double Pan_temp_glass;
    double Tilt_temp_glass;
    static int waist_flag = 0;
    int mode = 0;
    static int scan_flag = 0;
    static double scan_var = -0.6;
    static int missing_cnt = 0;

    static int PAN_Flag = 1;
    static int TILT_Flag = 0;
    static int TILT_UP_COUNT = 0;

    if(now_x == 0)
    {
        missing_cnt++;
        mode = 0;
    }
    else
    {
        missing_cnt = 0;
        mode = 1;
    }

    if(missing_cnt > 50)
    {
        mode = 2;

        if(isNoball)
            mode = 3;
        //if(isNoball && !isNoball_pan)
        //    mode = 3;
    }



    if(mode == 1)   // Tracking
    {
        Tracking_flag = 0;
        //        cout<<"now_X ============== "<<now_x<<endl;
        //        cout<<"X_POINT_STANDARD === " <<X_POINT_STANDARD<<endl;
        PID_Control_Float(&Tracking_pid_pan, X_POINT_STANDARD, now_x);
        PID_Control_Float(&Tracking_pid_tilt, Y_POINT_STANDARD, now_y);

        Pan_temp_glass = -Tracking_pid_pan.nowOutput;
        PAN_POSITION += Pan_temp_glass;

        Tilt_temp_glass = -Tracking_pid_tilt.nowOutput;
        TILT_POSITION += Tilt_temp_glass;

        scan_var = -1.0;
        scan_flag = 0;

        if(PAN_POSITION > Pan_Max)
        {
            PAN_POSITION = Pan_Max;
        }

        else if(PAN_POSITION < Pan_Min)
        {
            PAN_POSITION = Pan_Min;
        }

        if(TILT_POSITION > Tilt_Max)
        {
            TILT_POSITION = Tilt_Max;
        }

        else if(TILT_POSITION < Tilt_Min)
        {
            TILT_POSITION = Tilt_Min;
        }
    }

    else if(mode == 2)  // Scan
    {
        //        std::cout << "TILT_Flag = " << TILT_Flag << std::endl;
        //        std::cout << "PAN_Flag = " << PAN_Flag << std::endl;
        //        std::cout << "tiltupcount = " << TILT_UP_COUNT << std::endl;
        //        std::cout << "PAN_POSITION = " << PAN_POSITION << std::endl;
        //        std::cout << "TILT_POSITION = " << TILT_POSITION << std::endl;

        if(PAN_Flag == 1) // Go To Right
        {
            PAN_POSITION = PAN_POSITION - 0.5;

            if(PAN_POSITION < -ballScanPanRange)
            {
                TILT_Flag = 1;
                TILT_UP_COUNT++;
            }
        }
        else if(PAN_Flag == 2) // Go To Left
        {
            PAN_POSITION = PAN_POSITION + 0.5;

            if(PAN_POSITION > ballScanPanRange)
            {
                TILT_Flag = 1;
                TILT_UP_COUNT++;
            }
        }

        if(TILT_Flag == 1)
        {
            PAN_Flag = 0;
            TILT_POSITION = TILT_POSITION - 0.5;
            if(TILT_POSITION < defaultTiltMin + (4 - TILT_UP_COUNT) * fabs(defaultTiltMin/4))
            {
                if(TILT_UP_COUNT % 2 == 0)
                    PAN_Flag = 1;
                if(TILT_UP_COUNT % 2 == 1)
                    PAN_Flag = 2;
                TILT_Flag = 0;
            }
        }

        if(scan_start || TILT_UP_COUNT >= 4)
        {
            TILT_POSITION = defaultTiltMax; // 80
            TILT_UP_COUNT = 0;
            scan_start = false;
            isScanfinish = true;
        }

        if(TILT_POSITION > Tilt_Max)
        {
            TILT_POSITION = Tilt_Max;
        }

        else if(TILT_POSITION < Tilt_Min)
        {
            TILT_POSITION = Tilt_Min;
        }

    }

    else if(mode == 3)  // Only Tilt
    {
        std::cout << "TILT_Flag = " << TILT_Flag << std::endl;
        std::cout << "PAN_Flag = " << PAN_Flag << std::endl;
        std::cout << "tiltupcount = " << TILT_UP_COUNT << std::endl;
        std::cout << "PAN_POSITION = " << PAN_POSITION << std::endl;
        std::cout << "TILT_POSITION = " << TILT_POSITION << std::endl;

        PAN_POSITION = 0;

        if(!TILT_Flag)
        {
            TILT_POSITION = TILT_POSITION - 0.5;
            if(TILT_POSITION < defaultTiltMin)
            {
                TILT_Flag = 1;
            }
        }
        else if(TILT_Flag)
        {
            TILT_POSITION = TILT_POSITION + 0.5;
            if(TILT_POSITION >= defaultTiltMax)
            {
                TILT_Flag = 0;
            }
        }

        if(scan_start)
        {
            TILT_POSITION = 0; // 80
            TILT_UP_COUNT = 0;
            scan_start = false;
        }

        if(TILT_POSITION > Tilt_Max)
        {
            TILT_POSITION = Tilt_Max;
        }

        else if(TILT_POSITION < Tilt_Min)
        {
            TILT_POSITION = Tilt_Min;
        }


    }

    isNoball = false;

    return Tracking_flag;
}
