#ifndef ROS_THR_H
#define ROS_THR_H

#include <QObject>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sstream"
#include "CreatDataBuffer.h"
#include "geometry_msgs/Twist.h"
#include "mainwindow.h"
#include <QTimer>
#include "HelperFunctions.h"
#include "std_srvs/Empty.h"
#include "pid_controller.h"
#include "listnerthr.h"
#include "low_pass_filter.h"
#include <geometry_msgs/Pose2D.h>




using namespace ros;
using namespace std;

class listnerthr;

class MainWindow;

struct ControlCommand
{
    inline ControlCommand() {roll = pitch = yaw = gaz = 0;useHovering = 1;}
    inline ControlCommand(double roll, double pitch, double yaw, double gaz,bool useHovering)
    {
        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
        this->gaz = gaz;
        this->useHovering = useHovering;
    }
    double yaw, roll, pitch, gaz;
    bool useHovering;
};

class ROS_THR : public QObject
{
    Q_OBJECT
public:
    explicit ROS_THR(QObject *parent = 0);
    ~ROS_THR();
    NodeHandle n;
    CreatDataBuffer PRINT_Buff;
    MainWindow *gui;
    QTimer *MainTimer;
    listnerthr *img_thr;

    //controllers...........

    PID_Controller Height_Control;
    PID_Controller X_Control,Y_Control,Z_Control;
    PID_Controller YAW_Control;



    MultiThreadedSpinner *spinner;


    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    ros::Publisher vel_pub;
    ros::Publisher slop_pub;


    Subscriber takeoff_sub;
    Subscriber land_sub;
    ros::Subscriber vel_sub;

    ros::ServiceClient flattrim_srv;
    std_srvs::Empty flattrim_srv_srvs;
    ros::ServiceClient toggleCam_srv;
    std_srvs::Empty toggleCam_srv_srvs;


    unsigned int velCount;
    unsigned int velCount100ms;

    int slam_calib_counter;
    int send_slop_counter;
    int step_send;


    void send_Land_msg(void);
    void send_Takeoff_msg(void);
    void sendControlToDrone(ControlCommand cmd);

    void landCb(std_msgs::EmptyConstPtr);
    void takeoffCb(std_msgs::EmptyConstPtr);
    void velCb(const geometry_msgs::TwistConstPtr vel);
    void SLAM_Calibration(int _mode);

    void sendFlattrim();
    void sendToggleCam();


    void Drone_controll_operation(void);


private Q_SLOTS:
    void MainTimerEvent(void);


};

#endif // ROS_THR_H
