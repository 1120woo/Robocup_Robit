/**
 * @file /include/load_cell/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef load_cell_QNODE_HPP_
#define load_cell_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <msg_generate/zmp_msg.h>
#include <msg_generate/Serial2LC_msg.h>
#include <msg_generate/com_msg.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace std;

namespace load_cell {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
        QNode(int argc, char** argv );
        virtual ~QNode();
        bool init();
        void run();

        long int R_LC_Data[4];
        long int L_LC_Data[4];

        ros::Publisher Zmp_Pub;
        ros::Subscriber loadcell_sub;
        ros::Subscriber COM_sub;

        msg_generate::Serial2LC_msg LC_info;
        msg_generate::com_msg COM_info;

Q_SIGNALS:
        void rosShutdown();
        void LC_callback();

private:
        int init_argc;
        char** init_argv;

        void LoadCell_Callback(const msg_generate::Serial2LC_msg::ConstPtr &msg);
        void COM_Callback(const msg_generate::com_msg::ConstPtr &msg);

};

}  // namespace load_cell

#endif /* load_cell_QNODE_HPP_ */
