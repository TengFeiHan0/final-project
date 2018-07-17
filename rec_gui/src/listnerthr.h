#ifndef LISTNERTHR_H
#define LISTNERTHR_H

#include <QObject>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include <QThread>
#include <QTimer>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros_thr.h"
#include <boost/thread.hpp>
#include "CreatBuffer.h"
#include "ardrone_autonomy/Navdata.h"
#include "math.h"
#include<pcl_ros/point_cloud.h>
#include "mainwindow.h"


#define SUB_REC_NAV_POS_SETPOINT                   100
#define SUB_REC_NAV_YAW_SETPOINT                   200
#define SUB_REC_RE_CALIBRATION                     300
#define SLAM_CALIB_FINISHED                         20
#define _C1  0.58
#define _C2  17.8
#define _C3  10
#define _C4  35
#define _C5  10
#define _C6  25
#define _C7  1.4
#define _C8  1.0

using namespace ros;
using namespace std;

class ROS_THR;
class MainWindow;

struct AR_Data{
    float Batt;
    float Rx,Ry,Rz;
    float Rx_r,Ry_r,Rz_r;
    float height;
    float acc_x,acc_y,acc_z;
    float v_x,v_y,v_z;
    float acc_static_x,acc_static_y,acc_static_z;
    float acc_pure_x,acc_pure_y,acc_pure_z;

    float Global_acc_x,Global_acc_y,Global_acc_z;
    float Global_v_x,Global_v_y,Global_v_z;
};



class listnerthr : public QObject
{
    struct cam_pos_st{

        float X,Y,Z,PSI;
        float raw_PSI;
        int status;

        int SLAM_Calib_stage;
        cv::Point2f Calib_P0_z,Calib_P1_z;
        cv::Point2f Calib_P0_x,Calib_P0_y;
        float slope,Compensate_slope;
        float modified_x,modified_y,modified_z;

        float predicted_x,predicted_y,predicted_z;
        float predicted_PSI;
        float predicted_raw_PSI;

        pcl::PointXYZ Robot_Heading;



        void get_cam_modified_pos(void)
        {
            if(SLAM_Calib_stage == SLAM_CALIB_FINISHED)
            {
//                modified_x = slope*(X-Calib_P0_x.x) ;
//                modified_y = slope*(Y-Calib_P0_y.x) ;
//                modified_z = slope*(Z-Calib_P0_z.x) + Calib_P0_z.y;

                modified_x = slope*(X) ;
                modified_y = slope*(Y) ;
                modified_z = slope*(Z);
            }
        }
    };

    struct seg_str
    {
        pcl::PointXYZ SetPoint;
        pcl::PointXYZ modified_SetPoint;
        float admissible_Len;
        float yaw_setpoint;
        pcl::PointXYZ Robot_Heading_setpoint;


        void get_modified_setpoint(cam_pos_st drone_data)
        {
            if(drone_data.SLAM_Calib_stage == SLAM_CALIB_FINISHED)
            {
                modified_SetPoint.x = drone_data.slope*(SetPoint.x-drone_data.Calib_P0_x.x) ;
                modified_SetPoint.y = drone_data.slope*(SetPoint.y-drone_data.Calib_P0_y.x) ;
                modified_SetPoint.z = drone_data.slope*(SetPoint.z-drone_data.Calib_P0_z.x) + drone_data.Calib_P0_z.y;


            }
        }

    };

    Q_OBJECT
public:
    explicit listnerthr(QObject *parent = 0);
    ~listnerthr();

    ROS_THR *ros_bridg;
    MainWindow *gui;

    Subscriber sub_front_IMG,sub_bottom_IMG,sub_ORB_IMG;
    Subscriber sub_AR_data;
    ros::Subscriber ORB_sub;
    ros::Subscriber Seg_sub;

    cam_pos_st Camera_Posotion;
    seg_str Seg_OutPut;
    bool start_flag ;
    CreatBuffer IMG_Buff,ORB_IMG_Buff;
    AR_Data AR_Status;

    boost::thread *mythr;

    void CallBack_Grab_IMG(const sensor_msgs::ImageConstPtr& msg);
    void CallBack_ORB_Grab_IMG(const sensor_msgs::ImageConstPtr& msg);
    void thr_start();
    void AR_data_event(ardrone_autonomy::Navdata ARData);
    float RobotPointAngle(pcl::PointXYZ _robot, pcl::PointXYZ _vec);

signals:
    void once_run();

protected:
    void CallBack_Cam_POS(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud);
    void Segmentation_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud);

private Q_SLOTS:
    void run(void);
};

#endif // LISTNERTHR_H
