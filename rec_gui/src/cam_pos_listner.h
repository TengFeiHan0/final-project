#ifndef CAM_POS_LISTNER_H
#define CAM_POS_LISTNER_H

#include <QObject>
#include "ros/ros.h"
#include "../msg_gen/cpp/include/rec_gui/Cam_POS.h"
#include "ros_thr.h"
#include <QDebug>
#include <opencv2/opencv.hpp>
#include<pcl_ros/point_cloud.h>

#define SLAM_CALIB_FINISHED          20

using namespace ros;
using namespace std;
using namespace cv;


class ROS_THR;

class CAM_POS_Listner : public QObject
{
    struct seg_str
    {
        pcl::PointXYZ SetPoint;
        pcl::PointXYZ modified_SetPoint;
        float admissible_Len;
        float yaw_setpoint;
        pcl::PointXYZ Robot_Heading_setpoint;

    };

    struct cam_pos_st{

        float X,Y,Z,PSI;
        float raw_PSI;
        int status;

        int SLAM_Calib_stage;
        cv::Point2f Calib_P0_z,Calib_P1_z;
        cv::Point2f Calib_P0_x,Calib_P0_y;
        float slope;
        float modified_x,modified_y,modified_z;

        float predicted_x,predicted_y,predicted_z;
        float predicted_PSI;
        float predicted_raw_PSI;

        pcl::PointXYZ Robot_Heading;



        void get_cam_modified_pos(void)
        {
            if(SLAM_Calib_stage == SLAM_CALIB_FINISHED)
            {
                modified_x = slope*(X-Calib_P0_x.x) ;
                modified_y = slope*(Y-Calib_P0_y.x) ;
                modified_z = slope*(Z-Calib_P0_z.x) + Calib_P0_z.y;
            }
        }
    };

    Q_OBJECT
public:
    explicit CAM_POS_Listner(QObject *parent = 0);
    ~CAM_POS_Listner();

    ROS_THR *ros_bridg;
    ros::Subscriber ORB_sub;
    ros::Subscriber Seg_sub;

    cam_pos_st Camera_Posotion;
    seg_str Seg_OutPut;





    void thr_start(void);
    float RobotPointAngle(pcl::PointXYZ _robot, pcl::PointXYZ _vec);


private:
    void CallBack_Cam_POS(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud);
    void Segmentation_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud);
signals:
    void once_run();

private Q_SLOTS:
    void run(void);


signals:

public slots:
};

#endif // CAM_POS_LISTNER_H
