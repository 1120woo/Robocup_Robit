#ifndef MASTER_YONGPYONG_HPP
#define MASTER_YONGPYONG_HPP

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "pid_control_float.h"
#include "RoboCupGameControlData.h"

/*message headers*/
#include <msg_generate/imu_msg.h>
#include <msg_generate/motion_end.h>
#include <msg_generate/motionNum_msg.h>
#include <msg_generate/robocupvision.h>
#include <msg_generate/pan_tilt_msg.h>
#include <msg_generate/udp_order.h>
#include <msg_generate/ik_msg.h>
#include <msg_generate/position_msg.h>
#include <msg_generate/udp_msg.h>
#include <msg_generate/ikend_msg.h>
#include <msg_generate/robocupcontroller.h>

//MOTION=========================================================
#define SHOOT_R                         0x01
#define SHOOT_R_SIDE                    0x02
#define SHOOT_L                         0x03
#define SHOOT_L_SIDE                    0x04
#define STAND_F                         0x05
#define STAND_B                         0x06

//#define STAND_R                         0x08
//#define STAND_L                         0x06

//ROBOCUP MODE===================================================
#define MODE_TEST                       0

#define MODE_MOVE                       7
#define MODE_FAR_BALL                   8
#define MODE_WAIT                       9
#define MODE_NO_BALL                    10
#define MODE_BALL_DETECT                11
#define MODE_BALL_WALK                  12
#define MODE_BALL_STAND                 13
#define MODE_SET_Y                      14
#define MODE_KICK                       15
#define MODE_STOP                       16
#define MODE_FREEKICK                   17

//teamplay_case==================================================
#define KEEP_GOING                      0

//PARAMETER======================================================
#define ROBIT                            21

#define FIRSTHALF_GOAL                   90
#define SECONDHALF_GOAL                 -90

#define LEFTSIDE                        -90
#define RIGHTSIDE                         90

#define ATTACK                           1
#define DEFENSE                          2

//.............................................................
#define FIND_BALL_CNT                    40
#define LOST_BALL_CNT                    300
#define DETECT2NO_BALL_CNT               800
#define NO_BALL_CNT                      2000
#define NO_BALL_CNT_WAIT                 2500
#define SET_CNT                          800
#define WALK_CNT                         150
#define TURN_CNT                         700


#define DesireBallD                      350
#define DesireBallD_SHOOT                150
#define Walk_BallD                       680

#define YAW_ERROR                        4
#define YAW_LIMIT                        10
#define X_LIMIT                          15

#define DesirePP_R                      -15
#define DesirePP_L                       15

#define GOALPOST_ERROR                   7
#define GOALPOST_CONVERGENCE             20

#define PANPOS_ERROR_DEFAULT             40
#define PANPOS_ERRPR_RATIO               0.6
#define PANPOS_ERROR_LIMIT               40

//MODE_MOVE
#define MOVE_DISTANCE                    1000
#define FAR_DISTANCE                     1800
#define FAR_DISTANCE_ERROR               400


typedef struct _GameState
{
    int firstHalf;
    int firstside;
    int kickoffTeam;
    int state;
    int readyTime;
    int penalty;
    int secondState;
    int secondInfo[4];

    int stateBefore;

} GameState;

typedef struct _VisionMsg
{
    int ballX;
    int ballY;
    int ballD;
    double ballT;


} VisionMsg;

typedef struct _ImuMsg
{
    // Euler
    float roll;
    float pitch;
    float yaw;

}ImuMsg;

typedef struct _UdpMsg
{
    int yourArea;
    int ballArea;
    int yourCase;
    int yourBalldist;

}UdpMsg;

typedef struct _LocalMsg
{
    int robot_area;
    int ball_area;
    int yaw;

}LocalMsg;

typedef struct _Matrix
{
    int row;
    int col;

}Matrix;

PID Tracking_pid_pan;
PID Tracking_pid_tilt;

/** etc **/
int motion_end = 0;
int ik_end = 0;
int GOALPOST = FIRSTHALF_GOAL;
int ATTACK_MODE = ATTACK;

int PAN_POSITION_ERROR = 0;


int theta_ball_real = 0;
int desire_yaw = 0;

int goal_yaw = 0;
int Goal_zeroyaw = 0;

/** bool **/
bool motion_flag = false;
bool isFalldown = false;
bool isPlaying = false;
bool scan_start = false;
bool isNoball = false;
bool isSet = false;
bool local = true;
bool isPoint = false;
bool isTeamplay = false;

bool isDone = false;
bool isGone = false;

bool isKeepGoing = false;
bool isFar = false;

bool wasPenalty = false;
bool wasSet = false;
bool wasInitial = false;

bool isScanfinish = false;

bool MoveDone = false;

static int noball_time = 0;



/** CNT **/
int tracking_cnt = 0;
int no_ball_cnt = 0;
int my_turn_cnt = 0;
int your_turn_cnt = 0;
int lost_ball_count = 0;
//MESSAGE===================================================================
msg_generate::pan_tilt_msg ptMsg;
msg_generate::motionNum_msg motionMsg;
msg_generate::ik_msg ikMsg;
msg_generate::udp_msg udpMsg;

/** message subscriber **/
ros::Subscriber gameSub;
ros::Subscriber visionSub;
ros::Subscriber imuSub;
ros::Subscriber motionSub;
ros::Subscriber udpSub;
ros::Subscriber localSub;
ros::Subscriber ikendSub;

/** message publisher **/
ros::Publisher pantiltPub;
ros::Publisher ikPub;
ros::Publisher motionPub;
ros::Publisher udpPub;


//FUCTION====================================================================
void TRACKING_WHAT(int TrackingPoint_x, int TrackingPoint_y, double TrackingThing_x, double TrackingThing_y);
void WALK_START(int x, int y, int yaw);
bool MOVE(int _target_block);
void SET_YAW_PAN();
void SET_YAW_YAW();

int GET_YAW(int yaw_target);
int GET_GOALPOST_DR(int rb_block);
int GET_TARGET_BLOCK(int ball_block);
int GET_RETURN_BLOCK(int block);

Matrix ROBOT_WHERE(int block);
int Matrix2Block(Matrix mat);
#define WALK_STOP ikMsg.flag = 0

void robocupCallback(const ros::TimerEvent&);
void gameCallback(const msg_generate::robocupcontroller::ConstPtr&);
void visionCallback(const msg_generate::robocupvision::ConstPtr&);
void imuCallback(const msg_generate::imu_msg::ConstPtr&);
void motionCallback(const msg_generate::motion_end::ConstPtr&);
void udpCallback(const msg_generate::udp_order::ConstPtr&);
void localCallback(const msg_generate::position_msg::ConstPtr&);
void ikendCallback(const msg_generate::ikend_msg::ConstPtr&);


void DXL_Motion(unsigned char Motion_Num);
int Tracking(double now_x, double X_POINT_STANDARD, double now_y, double Y_POINT_STANDARD);





//TRACKING PARAMETER===========================================================
const int trackingDefaultPointX        = 320;
const int trackingDefaultPointY        = 240;
const int defaultPanMax                = 80;
const int defaultPanMin                = -80;
const int defaultTiltMax               = -10;
const int defaultTiltMin               = -90;
const int defaultScanPanRange          = 30;
const int defaultScanTiltRange         = -90;

// ball
const int ballTrackingPointX           = 320;
const int ballTrackingPointY           = 240;
const int ballPanMax                   = 55;
const int ballPanMin                   = -55;
const int ballTiltMax                  = 80;
const int ballTiltMin                  = 0;
const int ballScanPanRange             = ballPanMax;
const int ballScanTiltRange            = ballTiltMin;

const double ballDistanceTiltAngle          = -32;
const double ballPixelXRange                = 8.0;
const double ballPixelYRange                = 6.0;
const double ballDistancePanAngleRange      = 5.0;
const double ballDistanceForShootTiltAngle  = 5.0;

// etc
//int robocup_case = MODE_BALL_DETECT;
int robocup_case = MODE_TEST;
int yourCase_past = MODE_BALL_DETECT;
int motion_what = SHOOT_L;
int target_block = 0;
int temp_udpcase = MODE_BALL_DETECT;
int temp_case = MODE_BALL_DETECT;

//MOVE
bool isSamerow = false;
bool isSelect = false;     //if select target yaw => true
bool isSetYaw = false;


#endif // MASTER_YONGPYONH_HPP


