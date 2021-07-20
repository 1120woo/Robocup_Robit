/**
 * @file /include/load_cell/main_window.hpp
 *
 * @brief Qt based gui for load_cell.
 *
 * @date November 2010
 **/
#ifndef load_cell_MAIN_WINDOW_H
#define load_cell_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QApplication>
#include <QPainter>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>
#include "qnode.hpp"
#include "ui_main_window.h"
#include "/home/robit/catkin_ws/src/load_cell/include/load_cell/qcustomplot.hpp"

//#include "zmp_msg.h"

//#include <msg_generate/zmp_msg.h>
#include <msg_generate/Serial2LC_msg.h>
#include <msg_generate/R_LC_msg.h>
#include <msg_generate/L_LC_msg.h>
#include <msg_generate/com_msg.h>

#define load_cell_X_1  67.5
#define load_cell_Y_1 95.31

#define load_cell_X_2 -67.5
#define load_cell_Y_2 95.31

#define load_cell_X_3  67.5
#define load_cell_Y_3 -95.31

#define load_cell_X_4  -67.5
#define load_cell_Y_4 -95.31

#define total_load_cell_X_1 140.0
#define total_load_cell_Y_1 95.31

#define total_load_cell_X_2 -140.0
#define total_load_cell_Y_2 95.31

#define total_load_cell_X_3 140.0
#define total_load_cell_Y_3 -95.31

#define total_load_cell_X_4 -140.0
#define total_load_cell_Y_4 -95.31

#define median_cnt 5

#define PI 3.141592653589793

/////////////////////////////////////////

#define LC_NUM 8

#define ZERO_RESET    0
#define UNIT_RESET    1
#define SET_RefVALUE  2
#define VOID_STEP     99

#define UNIT_SET_1    0
#define UNIT_SET_2    1
#define UNIT_SET_3    2
#define UNIT_SET_4    3
#define UNIT_SET_5    4
#define UNIT_SET_6    5
#define UNIT_SET_7    6
#define UNIT_SET_8    7

#define LEFT_FOOT     0
#define RIGHT_FOOT    1


/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;

namespace load_cell {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
        ~MainWindow();

        QSerialPort *serial;
        msg_generate::zmp_msg zmp;

        bool open_serial();
        void paintEvent(QPaintEvent *event);

        QByteArray Tx_data;

        msg_generate::Serial2LC_msg LC_info;
        msg_generate::R_LC_msg r_lc_info;
        msg_generate::L_LC_msg l_lc_info;
        msg_generate::com_msg COM_info;


        int conversion_mode = -1;
        int unit_set_mode = 0;

        long int R_LC_Data[LC_NUM] = {0,};
        long int L_LC_Data[LC_NUM] = {0,};

        long int R_LC_Data_Filtering[LC_NUM] = {0,};
        long int L_LC_Data_Filtering[LC_NUM] = {0,};

        long int LC_data[LC_NUM] = {0,};
        long int LC_Zero_Value[LC_NUM] = {0,};
        double LC_Unit_Value[LC_NUM] = {0,};
        double LC_Pos_X_Value[LC_NUM] = {0,};
        double LC_Pos_Y_Value[LC_NUM] = {0,};

        double LC_T_Pos_X_Value[LC_NUM] = {0,};
        double LC_T_Pos_Y_Value[LC_NUM] = {0,};

        long int Zero_reset_LC_data[LC_NUM] = {0,};

        long int Above_4_LC_data[LC_NUM] = {0,};
        long int Above_4_init[LC_NUM] = {0,};

        long int Below_3_LC_data[LC_NUM] = {0,};

        int Cum_error[LC_NUM] = {0,};
        int error[LC_NUM] = {0,};

        long int add2zero[LC_NUM] = {0,};
        long int add2unit[LC_NUM] = {0,};

        long int LC_stable_value_2[LC_NUM] = {0,};
        long int LC_temp_data[LC_NUM] = {0,};

        bool Zero_flag = false;
        bool Unit_flag = false;

        int load_cell_median_buffer[8][median_cnt] = {{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,}};

        int g_z_lc[LC_NUM] = {0,};
        int g_u_lc[LC_NUM] = {0,};

        double R_Pos_X_Coordinate = 0.0;
        double R_Pos_Y_Coordinate = 0.0;
        double L_Pos_X_Coordinate = 0.0;
        double L_Pos_Y_Coordinate = 0.0;
        double T_Pos_X_Coordinate = 0.0;
        double T_Pos_Y_Coordinate = 0.0;

        double real_R_Pos_X_Coordinate = 0.0;
        double real_R_Pos_Y_Coordinate = 0.0;
        double real_L_Pos_X_Coordinate = 0.0;
        double real_L_Pos_Y_Coordinate = 0.0;
        double real_T_Pos_X_Coordinate = 0.0;
        double real_T_Pos_Y_Coordinate = 0.0;

        ////////////Low_pass_filter///////////
        long int Output = 0;
        long int data_old = 0;

        //////////Support_Link////////////
        bool Left_Foot = false;
        bool Right_Foot = false;
        bool Both_Feet = false;

        /////////COM_point/////////////
        double X_com = 0.0;
        double Y_com = 0.0;
        double real_X_com = 0.0;
        double real_Y_com = 0.0;

public Q_SLOTS:

        void LoadCell_Callback();
        void Zero_reset(int foot_what);
        void makePlot();
        void Plot_init();
        void update();
        void median(int data_1,int data_2,int data_3,int data_4,int data_5,int data_6,int data_7,int data_8);
        long int Low_pass_filter(long int initial_data);


private Q_SLOTS:
        void on_g_u_push_01_clicked();

        void on_g_u_push_02_clicked();

        void on_g_u_push_03_clicked();

        void on_g_u_push_04_clicked();

        void on_g_u_push_05_clicked();

        void on_g_u_push_06_clicked();

        void on_g_u_push_07_clicked();

        void on_g_u_push_08_clicked();

        void on_g_u_push_reset_clicked();

        void on_g_z_push_insert_clicked();

        void on_save_clicked();

        void on_open_clicked();

        void on_reset_clicked();

        void on_g_z_push_01_clicked();

        void on_g_z_push_02_clicked();

        void on_g_z_push_03_clicked();

        void on_g_z_push_04_clicked();

        void on_g_z_push_05_clicked();

        void on_g_z_push_06_clicked();

        void on_g_z_push_07_clicked();

        void on_g_z_push_08_clicked();

        void on_g_z_push_reset_clicked();

        void on_g_z_lc_01_textChanged(const QString &arg1);

        void on_g_z_lc_02_textChanged(const QString &arg1);

        void on_g_z_lc_03_textChanged(const QString &arg1);

        void on_g_z_lc_04_textChanged(const QString &arg1);

        void on_g_z_lc_05_textChanged(const QString &arg1);

        void on_g_z_lc_06_textChanged(const QString &arg1);

        void on_g_z_lc_07_textChanged(const QString &arg1);

        void on_g_z_lc_08_textChanged(const QString &arg1);

        void on_g_u_lc_01_textChanged(const QString &arg1);

        void on_g_u_lc_02_textChanged(const QString &arg1);

        void on_g_u_lc_03_textChanged(const QString &arg1);

        void on_g_u_lc_04_textChanged(const QString &arg1);

        void on_g_u_lc_05_textChanged(const QString &arg1);

        void on_g_u_lc_06_textChanged(const QString &arg1);

        void on_g_u_lc_07_textChanged(const QString &arg1);

        void on_g_u_lc_08_textChanged(const QString &arg1);

        void on_r_reset_bnt_clicked();

        void on_l_reset_bnt_clicked();

private:
        Ui::MainWindowDesign ui;
        QNode qnode;

Q_SIGNALS:
        void zmp_signal();
};

}  // namespace load_cell

#endif // load_cell_MAIN_WINDOW_H
