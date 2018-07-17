#ifndef LISTNERTHR_H
#define LISTNERTHR_H

#include <QObject>
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose2D.h>
#include<QDebug>
#include <QFile>
#include <QDataStream>
#include "seg_fcn.h"
#include "planning.h"
#include "viewer.h"
#include "captureviewer.h"

using namespace pcl;
using namespace std;
using namespace SEG;

class Planning;
class Viewer;
class CaptureViewer;

class listnerthr : public QObject
{
    Q_OBJECT
public:
    explicit listnerthr(QObject *parent = 0);
    ~listnerthr();
    ros::NodeHandle nh;
    Planning* planning_thr;
    Viewer* viewer_thr;
    CaptureViewer* capture_thr;

    QTimer *MainTimer;
    PointCloudT::Ptr cloud;
    int check_sim;
    float cloud_scale;
    cv::Point2f Calib_P0_z,Calib_P0_x,Calib_P0_y;

    PointT Robot_POS;
    PointT Robot_ANGLE;
    PointT Robot_Heading;
    bool Scale_flag;
    cv::Mat SLAM_IMG,RAW_IMG;
    int only_grab_map;

    ros::Subscriber cloud_sub;
    ros::Subscriber robot_pos_sub;
    ros::Subscriber scale_sub;
    ros::Subscriber SLAM_IMG_sub;
    ros::Subscriber RAW_IMG_sub;

    void thr_start();
    void cloud_callback(const PointCloudT::ConstPtr& _cloud);
    void robot_pos_callback(const PointCloudT::ConstPtr& _cloud);
    void cloud_scale_callback(const PointCloudT::ConstPtr& _cloud);
    void CallBack_ORB_Grab_IMG(const sensor_msgs::ImageConstPtr &msg);
    void CallBack_ORB_Grab_RAW_IMG(const sensor_msgs::ImageConstPtr &msg);
    void LoadCloudFromFile(QByteArray name);
    void OnlyGrabCloud(void);
    void SimulationMode(void);

signals:
    void thr_start_signal();
protected slots:
    void Run_THR(void);
    void MainTimerEvent();
};

#endif // LISTNERTHR_H
