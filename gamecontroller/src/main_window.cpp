/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/gamecontroller/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gamecontroller {

using namespace std;
using namespace Qt;

extern ros::Publisher controller_pub;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
    MainWindow::ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    std::cout << "11111111111111111111111" << std::endl;

    qnode.init();
    initAddrAndPort();

    std::cout << "22222222222222222222222" << std::endl;

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow()
{
    delete &ui;
    delete m_pUdpSocket;
}

void MainWindow::initAddrAndPort()
{
    bool isOpen_Network = false;

    while(!isOpen_Network)
    {
        QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();

        for(int i=0;i<ipAddressesList.size();i++)
        {
            std::cout << i << " " << ipAddressesList.at(i).toString().toStdString() << std::endl;
            if(ipAddressesList.at(i) != QHostAddress::LocalHost && ipAddressesList.at(i).toIPv4Address())
            {
                if(ipAddressesList.at(i).toString().toStdString().find("192") != std::string::npos) // 192
                {
                    m_qstrIp = ipAddressesList.at(i).toString();
                    isOpen_Network = true;
                    break;
                }
            }
        }

        if(m_qstrIp.isEmpty())
        {
            m_qstrIp = QHostAddress(QHostAddress::LocalHost).toString();
        }
    }
    m_qSrcAddress.setAddress(m_qstrIp);

    m_iPort = 3838;

    ui.IP_Line->setText(m_qstrIp);
    ui.PORT_Line->setText(QString::number(m_iPort));
}

void MainWindow::initSocket()
{
    if(m_pUdpSocket==NULL)
    {
        m_pUdpSocket = new QUdpSocket(this);
    }
    if(m_pUdpSocket!=NULL && m_pUdpSocket->bind(m_iPort , QUdpSocket::ShareAddress))
    {
        m_bIsServerOpen=true;

        ui.IP_Line->setEnabled(false);
        ui.PORT_Line->setEnabled(false);
        ui.NUM_line->setEnabled(false);
        ui.checkBox->setEnabled(false);

        ui.Server_Open->setText("CLOSE");

        connect(m_pUdpSocket,SIGNAL(readyRead()),this,SLOT(readData()));
    }
}

void MainWindow::closeSocket()
{
    ui.IP_Line->setEnabled(true);
    ui.PORT_Line->setEnabled(true);
    ui.NUM_line->setEnabled(true);
    ui.checkBox->setEnabled(true);

    disconnect(m_pUdpSocket,SIGNAL(readyRead()),this,SLOT(readData()));
    ui.textEdit->clear();

    ui.Server_Open->setText("OPEN");

    m_pUdpSocket->close();

    m_bIsServerOpen = false;
}

void MainWindow::on_Server_Open_clicked()
{
    if(m_bIsServerOpen == false)
    {
        QString n = ui.NUM_line->text();
        playerNum = n.toInt() - 1;
        if(playerNum < 0) playerNum = 0;

        firstside = ui.checkBox->isChecked() ? 0 : 1;
        secondside = ui.checkBox->isChecked() ? 1 : 0;

        ui.NUM_line->setText(QString::number(playerNum + 1));

        initSocket();
    }
    else
    {
        closeSocket();
    }
}

void MainWindow::readData()
{
    QHostAddress senderAddress;
    quint16 senderPort;

    //robocup_controller::robocupController RCMsg;
    msg_generate::robocupcontroller RCmsg;

    QByteArray str;
    str.resize(m_pUdpSocket->bytesAvailable());

    m_pUdpSocket->readDatagram(str.data(),str.size(),&senderAddress,&senderPort);

    qDebug() << "From : "<<senderAddress.toString();
    qDebug() << "Port : "<<senderPort;
    qDebug() << "Message : "<<str.size();

    cout<<" ============= GAME INFO ============="<<endl;
    cout<<" SIDE : "<<firstside<<endl;
    cout<<" playerNum = "<<playerNum<<endl;

    if(str.size() == 688)
    {
        ui.textEdit->clear();

        memmove(&robocupData, str.data(), sizeof(RoboCupGameControlData));
        std::cout << "state = " << (int)robocupData.state << std::endl;
        std::cout << "penalty = " << (int)robocupData.teams[firstside].players[playerNum].penalty << std::endl;
        std::cout << "half = " << (int)robocupData.firstHalf << std::endl;
        std::cout << "kickoff team = " << (int)robocupData.kickOffTeam << std::endl;
        std::cout << "secondary = " << (int)robocupData.secondaryState << std::endl;
        cout<<"Team 0 Number"<<(int)robocupData.teams[0].teamNumber<<endl;
        cout<<"Team 1 Number"<<(int)robocupData.teams[1].teamNumber<<endl;
//        std::cout << "side = " << firstside << std::endl;
//        std::cout << "^side = " << secondside << std::endl;

        RCmsg.state = (int)robocupData.state;
        RCmsg.firstHalf = (int)robocupData.firstHalf;
        RCmsg.firstside = firstside;
        RCmsg.kickoffTeam = (int)robocupData.kickOffTeam;
        RCmsg.secondState = (int)robocupData.secondaryState;

        cout<<"CARD!!!!"<<(int)robocupData.teams[firstside].players[playerNum].redCardCount<<(int)robocupData.teams[firstside].players[playerNum].yellowCardCount<<(int)robocupData.teams[firstside].players[playerNum].numberOfWarnings<<endl;
//        RCmsg.readyTime = (int)robocupData.secondaryTime;

        for(int i = 0; i < 4; i++)
        {
            RCmsg.secondInfo.push_back((int)robocupData.secondaryStateInfo[i]);
            std::cout << "secondary info = " << RCmsg.secondInfo[i] << " ";
        }
        std::cout << endl << endl;

        switch (RCmsg.firstHalf)
        {
        case 1:
            ui.textEdit->append("1st Half");
            RCmsg.penalty = (int)robocupData.teams[firstside].players[playerNum].penalty;
            break;

        case 0:
            ui.textEdit->append("2nd Half");
            RCmsg.penalty = (int)robocupData.teams[secondside].players[playerNum].penalty;
            break;
        default:
            break;
        }

        switch (RCmsg.state)
        {
        case STATE_INITIAL:
            ui.textEdit->append("state : Initial");
            break;
        case STATE_READY:
            ui.textEdit->append("state : Ready");
            RCmsg.readyTime = (int)robocupData.secondaryTime;
            ui.textEdit->append(QString::number(RCmsg.readyTime));
            break;
        case STATE_SET:
            ui.textEdit->append("state : Set");
            break;
        case STATE_PLAYING:
            ui.textEdit->append("state : Play");
            RCmsg.readyTime = (int)robocupData.secondaryTime;
            ui.textEdit->append(QString::number(RCmsg.readyTime));
            break;
        case STATE_FINISHED:
            ui.textEdit->append("state : Finish");
            break;
        default:
            break;

        }

        switch (RCmsg.penalty)
        {
        case HL_BALL_MANIPULATION:
            ui.textEdit->append("penalty : BALL_MANIPULATION");
            break;
        case HL_PHYSICAL_CONTACT:
            ui.textEdit->append("penalty : PHYSICAL_CONTACT");
            break;
        case HL_ILLEGAL_ATTACK:
            ui.textEdit->append("penalty : ILLEGAL_ATTACK");
            break;
        case HL_ILLEGAL_DEFENSE:
            ui.textEdit->append("penalty : ILLEGAL_DEFENSE");
            break;
        case HL_PICKUP_OR_INCAPABLE:
            ui.textEdit->append("penalty : PICKUP_OR_INCAPABLE");
            break;
        case HL_SERVICE:
            ui.textEdit->append("penalty : SERVICE");
            break;
        default:
            ui.textEdit->append("Penalty : NONE");
            break;

        }

        ui.textEdit->append("CARD : " + QString::number((int)robocupData.teams[firstside].players[playerNum].redCardCount)
                            + " " + QString::number((int)robocupData.teams[firstside].players[playerNum].yellowCardCount)
                            + " " + QString::number((int)robocupData.teams[firstside].players[playerNum].numberOfWarnings));

        switch (RCmsg.secondState)
        {
        case STATE2_PENALTYSHOOT:
            ui.textEdit->append("secondary : PENALTYSHOOT");
            ui.textEdit->append("info : " + QString::number(RCmsg.secondInfo[0]) + " " + QString::number(RCmsg.secondInfo[1]));
            break;

        case STATE2_OVERTIME:
            ui.textEdit->append("secondary : OVERTIME");
            break;

        case STATE2_TIMEOUT:
            ui.textEdit->append("secondary : TIMEOUT");
            break;

        case STATE2_DIRECT_FREEKICK:
            ui.textEdit->append("secondary : DIRECT_FREEKICK");
            ui.textEdit->append("info : " + QString::number(RCmsg.secondInfo[0]) + " " + QString::number(RCmsg.secondInfo[1]));
            break;

        case STATE2_INDIRECT_FREEKICK:
            ui.textEdit->append("secondary : INDIRECT_FREEKICK");
            ui.textEdit->append("info : " + QString::number(RCmsg.secondInfo[0]) + " " + QString::number(RCmsg.secondInfo[1]));
            break;

        case STATE2_PENALTYKICK:
            ui.textEdit->append("secondary : PENALTYKICK");
            ui.textEdit->append("info : " + QString::number(RCmsg.secondInfo[0]) + " " + QString::number(RCmsg.secondInfo[1]));
            break;

        case STATE2_CORNER_KICK:
            ui.textEdit->append("secondary : CORNERKICK");
            ui.textEdit->append("info : " + QString::number(RCmsg.secondInfo[0]) + " " + QString::number(RCmsg.secondInfo[1]));
            break;

        case STATE2_GOAL_KICK:
            ui.textEdit->append("secondary : GOALKICK");
            ui.textEdit->append("info : " + QString::number(RCmsg.secondInfo[0]) + " " + QString::number(RCmsg.secondInfo[1]));
            break;

        case STATE2_THROW_IN:
            ui.textEdit->append("secondary : THROWIN");
            ui.textEdit->append("info : " + QString::number(RCmsg.secondInfo[0]) + " " + QString::number(RCmsg.secondInfo[1]));
            break;

        default:
            ui.textEdit->append("secondary : NORMAL");
            break;
        }
/*

#define HL_BALL_MANIPULATION                30
#define HL_PHYSICAL_CONTACT                 31
#define HL_ILLEGAL_ATTACK                   32
#define HL_ILLEGAL_DEFENSE                  33
#define HL_PICKUP_OR_INCAPABLE              34
#define HL_SERVICE                          35
  */

//        controller_pub.publish(RCMsg);
          controller_pub.publish(RCmsg);
    }

}


}  // namespace robocup_controller
