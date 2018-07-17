#include "ros_thr.h"

#define LOOP_DURATION_SEC       0.03
#define LOOP_DURATION_MSEC      LOOP_DURATION_SEC*1000

ROS_THR::ROS_THR(QObject *parent) : QObject(parent)
{
    PRINT_Buff.initBuffer(100);
    MainTimer = new QTimer(this);
    MainTimer->start(LOOP_DURATION_MSEC);

    slam_calib_counter = 0;

    // Height Controller
    Height_Control.Kp = 0.02;
    Height_Control.Ki = 0.0001;
    Height_Control.Kd = 0.003;
    Height_Control.DT = LOOP_DURATION_SEC;
    Height_Control.P_limit = 0.7;
    Height_Control.I_limit = 0.3;
    Height_Control.D_limit = 0.4;
    Height_Control.Out_limit = 1.0;
    Height_Control.diff_err_filter.F_pass = 10;

    // X Controller
    X_Control.Kp = 0.002;
    X_Control.Ki = 0.0004;
    X_Control.Kd = 0.006;
    X_Control.DT = LOOP_DURATION_SEC;
    X_Control.P_limit = 0.7;
    X_Control.I_limit = 0.1;
    X_Control.D_limit = 0.8;
    X_Control.Out_limit = 0.4;
    X_Control.diff_err_filter.F_pass = 8;

    // Y Controller
    Y_Control.Kp = 0.002;
    Y_Control.Ki = 0.0004;
    Y_Control.Kd = 0.006;
    Y_Control.DT = LOOP_DURATION_SEC;
    Y_Control.P_limit = 0.7;
    Y_Control.I_limit = 0.1;
    Y_Control.D_limit = 0.8;
    Y_Control.Out_limit = 0.4;
    Y_Control.diff_err_filter.F_pass = 8;

    // Z Controller
    Z_Control.Kp = 0.02;
    Z_Control.Ki = 0;
    Z_Control.Kd = 0.003;
    Z_Control.DT = LOOP_DURATION_SEC;
    Z_Control.P_limit = 0.8;
    Z_Control.I_limit = 0.3;
    Z_Control.D_limit = 0.5;
    Z_Control.Out_limit = 1.0;
    Z_Control.diff_err_filter.F_pass = 10;

    // YAW Controller
    YAW_Control.Kp = 0.02;
    YAW_Control.Ki = 0.0005;
    YAW_Control.Kd = 0.002;
    YAW_Control.DT = LOOP_DURATION_SEC;
    YAW_Control.P_limit = 0.7;
    YAW_Control.I_limit = 0.3;
    YAW_Control.D_limit = 0.4;
    YAW_Control.Out_limit = 0.2;
    YAW_Control.diff_err_filter.F_pass = 10;

    send_slop_counter = 0;

    //Publishers............
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land",1);
    takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);
    vel_pub	= n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    //    slop_pub = n.advertise<geometry_msgs::Pose2D>("GUI/cloud_sacle",1);
    slop_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("GUI/cloud_sacle", 10);

    //Subscribers...........
    takeoff_sub	 = n.subscribe("ardrone/takeoff",1, &ROS_THR::takeoffCb, this);
    land_sub = n.subscribe("ardrone/land",1, &ROS_THR::landCb, this);
    vel_sub	= n.subscribe("cmd_vel",50, &ROS_THR::velCb, this);


    //Services..............
    flattrim_srv         = n.serviceClient<std_srvs::Empty>("ardrone/flattrim",1);
    toggleCam_srv        = n.serviceClient<std_srvs::Empty>("ardrone/togglecam",1);


    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    step_send = 0;
}

ROS_THR::~ROS_THR()
{

}

void ROS_THR::MainTimerEvent()
{
    Drone_controll_operation();

    if(img_thr->Camera_Posotion.SLAM_Calib_stage == SLAM_CALIB_FINISHED)
    {
        if( (send_slop_counter%10) == 0)
        {

            pcl::PointCloud<pcl::PointXYZ> pub_cloud;
            pcl::PointXYZ tmp_point;
            tmp_point.x = img_thr->Camera_Posotion.slope;
            tmp_point.y = img_thr->Camera_Posotion.slope;
            tmp_point.z = img_thr->Camera_Posotion.slope;
            pub_cloud.push_back(tmp_point);

            //            tmp_point.x = -img_thr->Camera_Posotion.Calib_P0_y.x;
            //            tmp_point.y = -img_thr->Camera_Posotion.Calib_P0_y.y;
            //            tmp_point.z = 0;
            //            pub_cloud.push_back(tmp_point);

            //            tmp_point.x = -img_thr->Camera_Posotion.Calib_P0_z.x;
            //            tmp_point.y = -img_thr->Camera_Posotion.Calib_P0_z.y;
            //            tmp_point.z = 0;
            //            pub_cloud.push_back(tmp_point);

            //            tmp_point.x = img_thr->Camera_Posotion.Calib_P0_x.x;
            //            tmp_point.y = img_thr->Camera_Posotion.Calib_P0_x.y;
            //            tmp_point.z = 0;
            //            pub_cloud.push_back(tmp_point);

            slop_pub.publish(pub_cloud);
        }
    }

    //    float PSI = (img_thr->Camera_Posotion.raw_PSI);
    float PSI = img_thr->Camera_Posotion.predicted_raw_PSI*(3.1415/180.0);
    //    float p_control = Y_Control.out*cos(PSI)+X_Control.out*sin(PSI);
    //    float r_control = X_Control.out*cos(PSI)-Y_Control.out*sin(PSI);

    float p_control = -X_Control.out*cos(PSI) - Y_Control.out *sin(PSI);
    float r_control =  X_Control.out*sin(PSI) - Y_Control.out *cos(PSI);

    // if nothing on /cmd_vel, repeat!
    if(velCount100ms == 0)
    {
        ControlCommand c;
        switch(gui->currentControlSource)
        {
        case CONTROL_NONE:
            sendControlToDrone(ControlCommand(0,0,0,0,1));
            break;
        case CONTROL_KB:
            sendControlToDrone(gui->calcKBControl());
            break;
        case CONTROL_HEIGHT:
            c = gui->calcKBControl();
            c.gaz = Height_Control.out;
            sendControlToDrone(c);
            //            qDebug()<<"is pr:"<<c.roll<<c.pitch<<c.yaw<<c.gaz<<c.useHovering;
            break;
        case CONTROL_SLAM_CALIB:
            c = gui->calcKBControl();
            c.gaz = Height_Control.out;
            sendControlToDrone(c);
            break;
        case CONTROL_POS:
            c = gui->calcKBControl();
            c.pitch = c.pitch + p_control;
            c.roll = c.roll + r_control;
            c.gaz = c.gaz + Z_Control.out;
            c.yaw = YAW_Control.out;
            //            qDebug() << c.pitch << c.roll << c.gaz << c.yaw;
            sendControlToDrone(c);
            break;
        case CONTROL_Auto:
            c = gui->calcKBControl();
            c.pitch = c.pitch + p_control;
            c.roll = c.roll + r_control;
            c.gaz = c.gaz + Z_Control.out;
            c.yaw = YAW_Control.out;
            sendControlToDrone(c);
            break;

        }
    }
    velCount100ms = 0;


    slam_calib_counter++;
    send_slop_counter++;

}

void ROS_THR::send_Land_msg()
{
    land_pub.publish(std_msgs::Empty());
}

void ROS_THR::send_Takeoff_msg()
{
    takeoff_pub.publish(std_msgs::Empty());
}

void ROS_THR::sendControlToDrone(ControlCommand cmd)
{
    // TODO: check converstion (!)
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = -cmd.yaw;
    cmdT.linear.z = cmd.gaz;
    cmdT.linear.x = -cmd.pitch;
    cmdT.linear.y = -cmd.roll;

    cmdT.angular.x = cmdT.angular.y = cmd.useHovering ? 0 : 1;


    vel_pub.publish(cmdT);
}

void ROS_THR::landCb(std_msgs::EmptyConstPtr)
{
    PRINT_Buff.Write2BufferOut("Sent: Landing Command");
}

void ROS_THR::takeoffCb(std_msgs::EmptyConstPtr)
{
    PRINT_Buff.Write2BufferOut("Sent: Taking off Command");
}

void ROS_THR::velCb(const geometry_msgs::TwistConstPtr vel)
{
    velCount++;
    velCount100ms++;

}

void ROS_THR::SLAM_Calibration(int _mode)
{
    //        qDebug()<<img_thr->Camera_Posotion.status;
    if(img_thr->Camera_Posotion.status == 1)
    {
        //            qDebug("calib");
        send_slop_counter = 0;
        if(img_thr->Camera_Posotion.SLAM_Calib_stage == 0)
        {
            Height_Control.setpoint = 40;
            img_thr->Camera_Posotion.SLAM_Calib_stage = 1;
            qDebug("Calib_Stage 1");

        }
        else if(img_thr->Camera_Posotion.SLAM_Calib_stage == 1)
        {
            if(!Height_Control.get_Hyst_flag_status())
            {
                img_thr->Camera_Posotion.SLAM_Calib_stage = 2;
                slam_calib_counter = 0;
                qDebug("Calib_Stage 2");
            }

        }
        else if(img_thr->Camera_Posotion.SLAM_Calib_stage == 2)
        {
            if(slam_calib_counter > (1.5/0.03))
            {

                img_thr->Camera_Posotion.Calib_P0_x.x = img_thr->Camera_Posotion.X;
                img_thr->Camera_Posotion.Calib_P0_y.x = img_thr->Camera_Posotion.Y;
                img_thr->Camera_Posotion.Calib_P0_z.x = img_thr->Camera_Posotion.Z;
                img_thr->Camera_Posotion.Calib_P0_z.y = img_thr->AR_Status.height;
                Height_Control.setpoint = 120;
                img_thr->Camera_Posotion.SLAM_Calib_stage = 3;

                qDebug("Calib_Stage 2");
            }

        }
        else if(img_thr->Camera_Posotion.SLAM_Calib_stage == 3)
        {
            if(!Height_Control.get_Hyst_flag_status())
            {
                img_thr->Camera_Posotion.SLAM_Calib_stage = 4;
                slam_calib_counter = 0;
                qDebug()<<"Calib_Stage 3 slope::"<<img_thr->Camera_Posotion.slope;
            }
        }
        else if(img_thr->Camera_Posotion.SLAM_Calib_stage == 4)
        {
            if(slam_calib_counter > (1.5/0.03))
            {
                img_thr->Camera_Posotion.Calib_P1_z.x = img_thr->Camera_Posotion.Z;
                img_thr->Camera_Posotion.Calib_P1_z.y = img_thr->AR_Status.height;
                Height_Control.setpoint = 50;
                img_thr->Camera_Posotion.SLAM_Calib_stage = 5;
                if(_mode == CONTROL_SLAM_CALIB )
                {
                    img_thr->Camera_Posotion.slope = (img_thr->Camera_Posotion.Calib_P1_z.y - img_thr->Camera_Posotion.Calib_P0_z.y)/
                            (img_thr->Camera_Posotion.Calib_P1_z.x - img_thr->Camera_Posotion.Calib_P0_z.x);
                    img_thr->Camera_Posotion.Compensate_slope = 1;

                }else if(_mode == CONTROL_SLAM_RE_CALIB)
                {
                    Height_Control.setpoint = 80;
                    img_thr->Camera_Posotion.Compensate_slope = (img_thr->Camera_Posotion.Calib_P1_z.y - img_thr->Camera_Posotion.Calib_P0_z.y)/
                            (img_thr->Camera_Posotion.Calib_P1_z.x - img_thr->Camera_Posotion.Calib_P0_z.x);

                }
            }

        }
        else if(img_thr->Camera_Posotion.SLAM_Calib_stage == 5)
        {
            if(!Height_Control.get_Hyst_flag_status())
            {
                img_thr->Camera_Posotion.SLAM_Calib_stage = SLAM_CALIB_FINISHED;

                if(_mode == CONTROL_SLAM_CALIB )
                    gui->setControlSource(CONTROL_KB);
                else if(_mode == CONTROL_SLAM_RE_CALIB)
                    gui->currentControlSource = CONTROL_Auto;
            }
        }


        Height_Control.point = img_thr->AR_Status.height;
        Height_Control.Run_control(MODIFIED_PID,WITH_HYST,8);
    }
}

void ROS_THR::sendFlattrim()
{
    flattrim_srv.call(flattrim_srv_srvs);
}

void ROS_THR::sendToggleCam()
{
    toggleCam_srv.call(toggleCam_srv_srvs);
}

void ROS_THR::Drone_controll_operation()
{
    if(gui->currentControlSource == CONTROL_HEIGHT)
    {
        Height_Control.setpoint = gui->Height_setpoint;
        Height_Control.point = img_thr->AR_Status.height;
        //        Height_Control.point = img_thr->Camera_Posotion.modified_z;
        Height_Control.Run_control(MODIFIED_PID,WITH_HYST,4);
        //        qDebug()<<Height_Control.D_out<<Height_Control.P_out<<Height_Control.I_out;


    }
    else if(gui->currentControlSource == CONTROL_SLAM_CALIB || gui->currentControlSource == CONTROL_SLAM_RE_CALIB)
    {
        SLAM_Calibration(gui->currentControlSource);
    }
    else if(gui->currentControlSource >= CONTROL_POS && img_thr->Camera_Posotion.SLAM_Calib_stage == SLAM_CALIB_FINISHED)
    {

        {
            if(gui->currentControlSource == CONTROL_Auto)
                Y_Control.setpoint = img_thr->Seg_OutPut.modified_SetPoint.y;
            else
                Y_Control.setpoint = gui->Y_setpoint;

            Y_Control.point = img_thr->Camera_Posotion.predicted_y;

            if(gui->currentControlSource == CONTROL_Auto)
                Y_Control.Run_control(MODIFIED_PID,WITH_OUT_HYST,10);
            else
                Y_Control.Run_control(MODIFIED_PID,WITH_OUT_HYST,1);
        }

        {
            if(gui->currentControlSource == CONTROL_Auto)
                X_Control.setpoint = (img_thr->Seg_OutPut.modified_SetPoint.x);
            else
                X_Control.setpoint = gui->X_setpoint;
            X_Control.point = img_thr->Camera_Posotion.predicted_x;

            if(gui->currentControlSource == CONTROL_Auto)
                X_Control.Run_control(MODIFIED_PID,WITH_OUT_HYST,10);
            else
                X_Control.Run_control(MODIFIED_PID,WITH_OUT_HYST,1);
        }

        {
            if(gui->currentControlSource == CONTROL_Auto)
                Z_Control.setpoint = (img_thr->Seg_OutPut.modified_SetPoint.z);
            else
                Z_Control.setpoint = gui->Z_setpoint;
            Z_Control.point = img_thr->Camera_Posotion.predicted_z;

            if(gui->currentControlSource == CONTROL_Auto)
                Z_Control.Run_control(MODIFIED_PID,WITH_OUT_HYST,10);
            else
                Z_Control.Run_control(MODIFIED_PID,WITH_OUT_HYST,10);
        }


        YAW_Control.setpoint = 0;
        if(gui->currentControlSource == CONTROL_Auto)
            YAW_Control.point = -img_thr->Camera_Posotion.predicted_PSI;
        else
            YAW_Control.point = -img_thr->Camera_Posotion.predicted_raw_PSI;
        YAW_Control.Run_control(MODIFIED_PID,WITH_OUT_HYST,2);

        //        qDebug()<<Y_Control.setpoint<<Y_Control.point<<Y_Control.D_out<<Y_Control.P_out;
        //        qDebug()<<X_Control.setpoint<<X_Control.point<<X_Control.D_out<<X_Control.P_out;
    }
    else if(gui->currentControlSource <= CONTROL_KB)
    {
        Height_Control.Reset_Control();
        Y_Control.Reset_Control();
        X_Control.Reset_Control();
        Z_Control.Reset_Control();
    }
}



