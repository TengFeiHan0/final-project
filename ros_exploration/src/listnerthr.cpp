#include "listnerthr.h"

listnerthr::listnerthr(QObject *parent) : QObject(parent)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    Robot_POS = PointT(0,0,0);
    Robot_Heading = PointT(0,0,1);
    check_sim = 0;
    cloud_scale = 1;
    Scale_flag = 0;
    only_grab_map = 0;

    MainTimer = new QTimer(this);

    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    connect(this,SIGNAL(thr_start_signal()),this,SLOT(Run_THR()));
}

listnerthr::~listnerthr()
{

}
void listnerthr::thr_start()
{
    emit thr_start_signal();
}

void listnerthr::cloud_callback(const PointCloudT::ConstPtr& _cloud)
{
    if(check_sim != (int)(_cloud->points[0].x))
    {
        if(Scale_flag == 1)
        {
            Robot_POS.x = cloud_scale*_cloud->points[1].x;
            Robot_POS.y = cloud_scale*_cloud->points[1].y;
            Robot_POS.z = cloud_scale*_cloud->points[1].z;

//            Robot_ANGLE.x = _cloud->points[2].x;
//            Robot_ANGLE.y = _cloud->points[2].y;
//            Robot_ANGLE.z = _cloud->points[2].z;
//            Robot_Heading = PointT(sin(Robot_ANGLE.z),0,cos(Robot_ANGLE.z));

            Robot_Heading.x = -_cloud->points[2].x;
            Robot_Heading.y = 0;
            Robot_Heading.z = _cloud->points[2].z;
//            Robot_Heading = PointT(sin(Robot_ANGLE.z),0,cos(Robot_ANGLE.z));

            if(planning_thr->start_nav == -1)
                planning_thr->start_nav = 0;

            int dif_size = abs((int)(cloud->points.size()-_cloud->points.size()));

            //            qDebug()<<check_sim<<dif_size;
            if(dif_size > 50)
            {
                while(planning_thr->busy_flag == 1 || viewer_thr->busy_flag == 1)
                {
                    qDebug("Busy");
                }
                cloud->points.clear();
                PointT tmp_point;
                for(int k=3;k<_cloud->points.size();k++)
                {
                    tmp_point.x = _cloud->points[k].x * cloud_scale;
                    tmp_point.y = _cloud->points[k].y * cloud_scale;
                    tmp_point.z = _cloud->points[k].z * cloud_scale;

                    cloud->points.push_back(tmp_point);
                }
                //                remove_outlier(cloud,cloud,5,0.5);
            }

        }


    }
    check_sim = (int)(_cloud->points[0].x);
}

void listnerthr::robot_pos_callback(const PointCloudT::ConstPtr& _cloud)
{
    //    qDebug()<<"Listner"<<QThread::currentThreadId();
    Robot_POS.x = cloud_scale*_cloud->points[0].x;
    Robot_POS.y = cloud_scale*_cloud->points[0].y;
    Robot_POS.z = cloud_scale*_cloud->points[0].z;
}


void listnerthr::cloud_scale_callback(const PointCloudT::ConstPtr& _cloud)
{

    if(_cloud->points.size() == 1)
    {
        cloud_scale = (_cloud->points[0].x/100);

//        Calib_P0_x.x = (_cloud->points[1].x);
//        Calib_P0_x.y = (_cloud->points[1].y/100);

//        Calib_P0_y.x = (_cloud->points[2].x);
//        Calib_P0_y.y = (_cloud->points[2].y/100);

//        Calib_P0_z.x = (_cloud->points[3].x);
//        Calib_P0_z.y = (_cloud->points[3].y/100);

        Scale_flag = 1;
    }
}

void listnerthr::CallBack_ORB_Grab_IMG(const sensor_msgs::ImageConstPtr &msg)
{


    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        //        ROS_ERROR("cv_bridge exception: %s", e.what());
        //        return;
    }


    if(!cv_ptr->image.empty())
    {
        //        while(capture_thr->busy_flag == 1)
        //        {
        //            qDebug("SLAM_Busy");
        //        }
        //        SLAM_IMG = cv_ptr->image.clone();
        capture_thr->SLAM_IMG_buff.Write2BufferIn(cv_ptr->image.clone());
    }
}

void listnerthr::CallBack_ORB_Grab_RAW_IMG(const sensor_msgs::ImageConstPtr &msg)
{


    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        //        ROS_ERROR("cv_bridge exception: %s", e.what());
        //        return;
    }

    if(!cv_ptr->image.empty())
    {
        //        while(capture_thr->busy_flag == 1)
        //        {
        //            qDebug("Raw_Busy");
        //        }
        //        RAW_IMG = cv_ptr->image.clone();
        capture_thr->Raw_IMG_buff.Write2BufferIn(cv_ptr->image.clone());
    }
}

void listnerthr::LoadCloudFromFile(QByteArray name)
{
    only_grab_map = 2;
    if(name != "NON")
    {
        LoadFromeFile(cloud,name);
//        remove_outlier(cloud,cloud,5,0.5);
        //    generate_sample_pointcloud(cloud,PointT(-1.5,0.2,1.5),PointT(1.5,-0.4,1.5),400,0.05);
        //        generate_sample_pointcloud(cloud,PointT(-1.2,-0.1,2.2),PointT(-0.5,-1.5,2.2),200,0.05);
    }
    else
    {
        generate_corridor_pointcloud(cloud,PointT(2.5,2.5,7),PointT(0,-0.75,0),PointT(0,0,1),1000,0.05);
//        generate_sample_pointcloud(cloud,PointT(-1.25,-2,7),PointT(-1.25,0.5,9),300,0.05);
        generate_sample_pointcloud(cloud,PointT(-1.25,0.5,7),PointT(1.25,0.5,9),500,0.05);
        generate_sample_pointcloud(cloud,PointT(-1.25,-2,9),PointT(1.25,0.5,9),500,0.05);
        generate_sample_pointcloud(cloud,PointT(-1.25,-2,3),PointT(-0.3,0.5,3),400,0.05);
        generate_sample_pointcloud(cloud,PointT(0.3,-2,4),PointT(1.25,0.5,4),400,0.05);
        generate_sample_pointcloud(cloud,PointT(-1.25,-0.4,5),PointT(1.25,0.4,5),400,0.05);
//        generate_sample_pointcloud(cloud,PointT(1.5,-0.4,7),PointT(1.5,0.4,9),400,0.05);
        generate_corridor_pointcloud(cloud,PointT(5,2.5,2),PointT(1.25,-0.75,8),PointT(1,0,0),600,0.05);
        generate_corridor_pointcloud(cloud,PointT(5,2.5,2),PointT(-0.6,-0.75,8),PointT(-1,0,0),600,0.05);
    }

    planning_thr->start_nav = 0;
    cloud_scale = 1;
}

void listnerthr::OnlyGrabCloud()
{
    only_grab_map = 1;
    cloud_scale = 1;
    planning_thr->start_nav = -2;
    Scale_flag = 1;
//    generate_sample_pointcloud(cloud,PointT(-5,1,-2),PointT(5,1,8),50000,0);
//    generate_sample_pointcloud(cloud,PointT(-0.5,-0.5,3.5),PointT(2,0.5,3.5),500,0);

}

void listnerthr::SimulationMode()
{
    cloud_scale = 1;
    planning_thr->start_nav = -1;
    Scale_flag = 1;
}

void listnerthr::Run_THR()
{
    if(only_grab_map < 2)
        cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("ORB_SLAM/pcl_cloud",10,&listnerthr::cloud_callback,this);
    scale_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("GUI/cloud_sacle", 10, &listnerthr::cloud_scale_callback,this);
    SLAM_IMG_sub = nh.subscribe("ORB_SLAM/Frame",5,&listnerthr::CallBack_ORB_Grab_IMG,this);
    RAW_IMG_sub = nh.subscribe("ardrone/front/image_raw",5,&listnerthr::CallBack_ORB_Grab_RAW_IMG,this);
    //    robot_pos_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("ORB_SLAM/TO_Drone",10,&listnerthr::robot_pos_callback,this);

    MainTimer->start(6);

}

void listnerthr::MainTimerEvent()
{
    if(nh.ok())  // We may check also for the other NodeHandle state
        ros::spinOnce();

}

