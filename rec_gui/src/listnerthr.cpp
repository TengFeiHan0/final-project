#include "listnerthr.h"

listnerthr::listnerthr(QObject *parent) : QObject(parent)
{
    start_flag = 0;

    IMG_Buff.initBuffer(2);
    ORB_IMG_Buff.initBuffer(2);
    Camera_Posotion.X = 0;
    Camera_Posotion.Y = 0;
    Camera_Posotion.Z = 0;
    Camera_Posotion.PSI = 0;
    Camera_Posotion.status = 50;

    Camera_Posotion.SLAM_Calib_stage = 0;
    Camera_Posotion.modified_x = 0;
    Camera_Posotion.modified_y = 0;
    Camera_Posotion.modified_z = 0;
    Camera_Posotion.slope = 1;
    Camera_Posotion.Compensate_slope = 1;

    Seg_OutPut.SetPoint = pcl::PointXYZ(0,0,0);
    Seg_OutPut.admissible_Len = 0;

    Seg_OutPut.Robot_Heading_setpoint = pcl::PointXYZ(0,0,1);


    connect(this,SIGNAL(once_run()),this,SLOT(run()));


}

listnerthr::~listnerthr()
{

}

void listnerthr::run()
{
    sub_front_IMG = ros_bridg->n.subscribe("ardrone/front/image_raw",5,&listnerthr::CallBack_Grab_IMG,this);
    sub_bottom_IMG = ros_bridg->n.subscribe("ardrone/bottom/image_raw",5,&listnerthr::CallBack_Grab_IMG,this);
    sub_ORB_IMG = ros_bridg->n.subscribe("ORB_SLAM/Frame",5,&listnerthr::CallBack_ORB_Grab_IMG,this);

    sub_AR_data = ros_bridg->n.subscribe("ardrone/navdata",10,&listnerthr::AR_data_event,this);
    ORB_sub = ros_bridg->n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("ORB_SLAM/TO_Drone",5,&listnerthr::CallBack_Cam_POS,this);
    Seg_sub = ros_bridg->n.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("ROS_EXP/OUTPUT", 10,&listnerthr::Segmentation_callback,this);

     ros::spin();
}

void listnerthr::CallBack_Grab_IMG(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat im;


    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(!cv_ptr->image.empty())
    {
        cv_ptr->image.copyTo(im);
        //        qDebug("Grabing IMG");
        IMG_Buff.Write2BufferOut(im);
    }

}

void listnerthr::CallBack_ORB_Grab_IMG(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat im;


    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(!cv_ptr->image.empty())
    {
        cv_ptr->image.copyTo(im);
        //        qDebug("Grabing IMG");
        ORB_IMG_Buff.Write2BufferOut(im);
    }
}

void listnerthr::thr_start()
{
    emit once_run();
}

void listnerthr::AR_data_event(ardrone_autonomy::Navdata ARData)
{

    //    int batt = (int)(ARData.batteryPercent);
    //        ROS_INFO("Battery: %02d%%",batt);
    //        ROS_INFO("Rx:%0.2f Ry:%0.2f Rz:%0.2f",ARData.rotX,ARData.rotY,ARData.rotZ);
    //        ROS_INFO("Height: %d cm",((ARData.altd))/10);

    AR_Status.Batt = ARData.batteryPercent;

    AR_Status.Rx = ARData.rotX;
    AR_Status.Ry = ARData.rotY;
    AR_Status.Rz = ARData.rotZ;

    float Rx = AR_Status.Rx*(3.1415/180.0);
    float Ry = AR_Status.Ry*(3.1415/180.0);
//    float Rz = Camera_Posotion.raw_PSI;
    float Rz = AR_Status.Rz*(3.1415/180.0);

    AR_Status.Rx_r = AR_Status.Rx*(3.1415/180.0);
    AR_Status.Ry_r = AR_Status.Ry*(3.1415/180.0);
    AR_Status.Rz_r = AR_Status.Rz*(3.1415/180.0);

    AR_Status.acc_x = ARData.ax - 0.0283721;
    AR_Status.acc_y = ARData.ay + 0.113757;
    AR_Status.acc_z = ARData.az;


    AR_Status.acc_static_x = sin(-Ry);
    AR_Status.acc_static_y = sin(Rx)*cos(Ry);
    AR_Status.acc_static_z = sqrt(pow(1.02,2)-pow(AR_Status.acc_static_x,2)-pow(AR_Status.acc_static_y,2));

    AR_Status.acc_pure_x = AR_Status.acc_x-AR_Status.acc_static_x;
    AR_Status.acc_pure_y = AR_Status.acc_y-AR_Status.acc_static_y;
    AR_Status.acc_pure_z = AR_Status.acc_z-AR_Status.acc_static_z;

    AR_Status.v_x = ARData.vx/10;
    AR_Status.v_y = ARData.vy/10;
    AR_Status.v_z = ARData.vz/10;

    float PSI = Camera_Posotion.predicted_raw_PSI*(3.1415/180.0);

    AR_Status.Global_v_x = cos(PSI)*AR_Status.v_x - sin(PSI)*AR_Status.v_y;
    AR_Status.Global_v_y = sin(PSI)*AR_Status.v_x + cos(PSI)*AR_Status.v_y;
    AR_Status.Global_v_z = AR_Status.v_z;

    AR_Status.Global_acc_x = cos(PSI)*AR_Status.acc_pure_x - sin(PSI)*AR_Status.acc_pure_y;
    AR_Status.Global_acc_y = sin(PSI)*AR_Status.acc_pure_x + cos(PSI)*AR_Status.acc_pure_y;



    AR_Status.height = (float)((ARData.altd))/10;

}

float listnerthr::RobotPointAngle(pcl::PointXYZ _robot, pcl::PointXYZ _vec)
{
    float Hp_V_dot = _vec.z*_robot.x - _robot.z*_vec.x;
    float H_V_dot = _vec.x*_robot.x + _robot.z*_vec.z;

    float attitude = (atan2(H_V_dot,Hp_V_dot)*(180/3.1415) - 90);

    if(attitude > 180)
        attitude = attitude - 360;
    else if(attitude < -180)
        attitude = attitude + 360;

    return attitude;
}

void listnerthr::CallBack_Cam_POS(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud)
{
    //    qDebug()<<data->status<<data->Cam_x<<data->Cam_y<<data->Cam_z;

    Camera_Posotion.X = _cloud->points[0].z;
    Camera_Posotion.Y = -_cloud->points[0].x;
    Camera_Posotion.Z = -_cloud->points[0].y;
//    Camera_Posotion.PSI = -_cloud->points[1].x;

    Camera_Posotion.Robot_Heading.x = -_cloud->points[1].x;
    Camera_Posotion.Robot_Heading.y = 0;
    Camera_Posotion.Robot_Heading.z = _cloud->points[1].z;

    Camera_Posotion.PSI = -(3.1415/180.0)*RobotPointAngle(Seg_OutPut.Robot_Heading_setpoint,Camera_Posotion.Robot_Heading);
    Camera_Posotion.raw_PSI = -(3.1415/180.0)*RobotPointAngle(pcl::PointXYZ(0,0,1),Camera_Posotion.Robot_Heading);



    Camera_Posotion.status = _cloud->points[2].x;

    Camera_Posotion.get_cam_modified_pos();

    //    qDebug()<<Camera_Posotion.Robot_Heading.x<<Camera_Posotion.Robot_Heading.z << Camera_Posotion.raw_PSI;
}

void listnerthr::Segmentation_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud)
{
    int _type = 0;
    if(_cloud->points[0].x == _cloud->points[0].y && _cloud->points[0].y == _cloud->points[0].z)
        _type = (int)_cloud->points[0].x;

    if(_cloud->points.size() == 3)
    {
        if(_type == SUB_REC_NAV_POS_SETPOINT)
        {
            Seg_OutPut.SetPoint.x = _cloud->points[1].z*(100.0);
            Seg_OutPut.SetPoint.y = -_cloud->points[1].x*(100.0);
            Seg_OutPut.SetPoint.z = -_cloud->points[1].y*(100.0);

            Seg_OutPut.modified_SetPoint.x = Seg_OutPut.SetPoint.x/(Camera_Posotion.Compensate_slope);
            Seg_OutPut.modified_SetPoint.y = Seg_OutPut.SetPoint.y/(Camera_Posotion.Compensate_slope);
            Seg_OutPut.modified_SetPoint.z = Seg_OutPut.SetPoint.z/(Camera_Posotion.Compensate_slope);

//            Seg_OutPut.get_modified_setpoint(Camera_Posotion);
//            Seg_OutPut.modified_SetPoint.x = (Seg_OutPut.SetPoint.z*100.0)-(Camera_Posotion.slope*Camera_Posotion.Calib_P0_y.x) ;
//            Seg_OutPut.modified_SetPoint.y = (Seg_OutPut.SetPoint.x*100.0)-(Camera_Posotion.slope*Camera_Posotion.Calib_P0_x.x) ;
//            Seg_OutPut.modified_SetPoint.z = (-Seg_OutPut.SetPoint.y*100.0)-(Camera_Posotion.slope*Camera_Posotion.Calib_P0_z.x) + Camera_Posotion.Calib_P0_z.y;

        }
        else if(_type == SUB_REC_NAV_YAW_SETPOINT)
        {
//            Seg_OutPut.yaw_setpoint = _cloud->points[1].x;
            Seg_OutPut.Robot_Heading_setpoint.x =  _cloud->points[1].x;
            Seg_OutPut.Robot_Heading_setpoint.y =  _cloud->points[1].y;
            Seg_OutPut.Robot_Heading_setpoint.z =  _cloud->points[1].z;
        }else if(_type == SUB_REC_RE_CALIBRATION)
        {
            Camera_Posotion.SLAM_Calib_stage = 0;
            gui->currentControlSource = CONTROL_SLAM_RE_CALIB;
        }

        Seg_OutPut.admissible_Len = _cloud->points[2].x;



//        qDebug()<<"Set_point"<<Seg_OutPut.modified_SetPoint.x<<Seg_OutPut.modified_SetPoint.y<<Seg_OutPut.modified_SetPoint.z<<Seg_OutPut.admissible_Len;
    }
}

