#include "cam_pos_listner.h"

CAM_POS_Listner::CAM_POS_Listner(QObject *parent) : QObject(parent)
{

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

    Seg_OutPut.SetPoint = pcl::PointXYZ(0,0,0);
    Seg_OutPut.admissible_Len = 0;

    Seg_OutPut.Robot_Heading_setpoint = pcl::PointXYZ(0,0,1);




    connect(this,SIGNAL(once_run()),this,SLOT(run()));

}

CAM_POS_Listner::~CAM_POS_Listner()
{

}

void CAM_POS_Listner::thr_start()
{
    emit once_run();
}



void CAM_POS_Listner::CallBack_Cam_POS(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud)
{
    //    qDebug()<<data->status<<data->Cam_x<<data->Cam_y<<data->Cam_z;

    Camera_Posotion.X = _cloud->points[0].x;
    Camera_Posotion.Y = _cloud->points[0].z;
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

float CAM_POS_Listner::RobotPointAngle(pcl::PointXYZ _robot, pcl::PointXYZ _vec)
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

void CAM_POS_Listner::Segmentation_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud)
{
    int _type = 0;
    if(_cloud->points[0].x == _cloud->points[0].y && _cloud->points[0].y == _cloud->points[0].z)
        _type = (int)_cloud->points[0].x;

    if(_cloud->points.size() == 3)
    {
        if(_type == 100)
        {
            Seg_OutPut.SetPoint = _cloud->points[1];

            Seg_OutPut.modified_SetPoint.x = (Seg_OutPut.SetPoint.x*100.0)-(Camera_Posotion.slope*Camera_Posotion.Calib_P0_x.x) ;
            Seg_OutPut.modified_SetPoint.y = (Seg_OutPut.SetPoint.z*100.0)-(Camera_Posotion.slope*Camera_Posotion.Calib_P0_y.x) ;
            Seg_OutPut.modified_SetPoint.z = (-Seg_OutPut.SetPoint.y*100.0)-(Camera_Posotion.slope*Camera_Posotion.Calib_P0_z.x) + Camera_Posotion.Calib_P0_z.y;

        }
        else if(_type == 200)
        {
//            Seg_OutPut.yaw_setpoint = _cloud->points[1].x;
            Seg_OutPut.Robot_Heading_setpoint.x =  _cloud->points[1].x;
            Seg_OutPut.Robot_Heading_setpoint.y =  _cloud->points[1].y;
            Seg_OutPut.Robot_Heading_setpoint.z =  _cloud->points[1].z;
        }

        Seg_OutPut.admissible_Len = _cloud->points[2].x;



        qDebug()<<"Set_point"<<Seg_OutPut.modified_SetPoint.x<<Seg_OutPut.modified_SetPoint.y<<Seg_OutPut.modified_SetPoint.z<<Seg_OutPut.admissible_Len;
    }
}

void CAM_POS_Listner::run()
{

    ORB_sub = ros_bridg->n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("ORB_SLAM/TO_Drone",5,&CAM_POS_Listner::CallBack_Cam_POS,this);
    Seg_sub = ros_bridg->n.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("ROS_EXP/OUTPUT", 10,&CAM_POS_Listner::Segmentation_callback,this);

}

