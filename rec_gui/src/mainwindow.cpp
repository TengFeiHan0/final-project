#include "mainwindow.h"
#include "ui_mainwindow.h"

#define LOOP_DURATION_SEC       0.02
#define LOOP_DURATION_MSEC      (LOOP_DURATION_SEC*1000)
#define KF_LOOP_DURATION_SEC        LOOP_DURATION_SEC

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    MainTimer = new QTimer(this);
    MainTimer->start(LOOP_DURATION_MSEC);
    flag = 1;

    sensYaw = 0.3;
    sensRP = 1;
    sensGaz = 0.5;

    YAW_setpoint = 0;

    ORB_IMG_flag = 0;


    for(int i=0;i<8;i++)
    {
        isPressed[i] = false;
        lastRepeat[i] = 0;
    }

    //Kalman..................
    KF = new KalmanFilter(2,1,1);
    KF_state = Mat(2,1,CV_32F);
    KF_Measurment = Mat(1,1,CV_32F);
    KF_Control = Mat(1,1,CV_32F);

    setIdentity(KF->transitionMatrix);
    KF->transitionMatrix.at<float>(1) = KF_LOOP_DURATION_SEC;

    KF->controlMatrix = Mat::zeros(2,1,CV_32F);
    KF->controlMatrix.at<float>(0) = 0.5*KF_LOOP_DURATION_SEC*KF_LOOP_DURATION_SEC;
    KF->controlMatrix.at<float>(1) = KF_LOOP_DURATION_SEC;

    KF->measurementMatrix = Mat::zeros(1,2,CV_32F);
    KF->measurementMatrix.at<float>(0) = 1.0f;

    setIdentity(KF->processNoiseCov,3e-3);
    setIdentity(KF->measurementNoiseCov,0.01);

    //KF_YAW...............
    KF_YAW = new KalmanFilter(1,1,1);
    KF_YAW_state = Mat(1,1,CV_32F);
    KF_YAW_Measurment = Mat(1,1,CV_32F);
    KF_YAW_Control = Mat(1,1,CV_32F);

    KF_YAW->transitionMatrix.at<float>(0) = 1;

    KF_YAW->controlMatrix = Mat::zeros(1,1,CV_32F);
    KF_YAW->controlMatrix.at<float>(0) = KF_LOOP_DURATION_SEC;

    KF_YAW->measurementMatrix = Mat::zeros(1,1,CV_32F);
    KF_YAW->measurementMatrix.at<float>(0) = 1.0f;


    // comprehensive Kolman Filter...........
    KF_Obs = new KalmanFilter(4,4,7);
    KF_Obs_state = Mat(4,1,CV_32F);
    KF_Obs_Measurment = Mat(4,1,CV_32F);
    KF_Obs_Control = Mat(7,1,CV_32F);

    KF_Obs->transitionMatrix = Mat::zeros(4,4,CV_32F);
    setIdentity(KF_Obs->transitionMatrix);
//    KF_Obs->transitionMatrix.at<float>(0,3) = KF_LOOP_DURATION_SEC;
//    KF_Obs->transitionMatrix.at<float>(1,4) = KF_LOOP_DURATION_SEC;
//    KF_Obs->transitionMatrix.at<float>(2,5) = KF_LOOP_DURATION_SEC;

    KF_Obs->controlMatrix = Mat::zeros(4,7,CV_32F);
    KF_Obs->controlMatrix.at<float>(0,0) = KF_LOOP_DURATION_SEC;
    KF_Obs->controlMatrix.at<float>(1,1) = KF_LOOP_DURATION_SEC;
    KF_Obs->controlMatrix.at<float>(2,2) = KF_LOOP_DURATION_SEC;
    KF_Obs->controlMatrix.at<float>(0,3) = 0.5*KF_LOOP_DURATION_SEC*KF_LOOP_DURATION_SEC;
    KF_Obs->controlMatrix.at<float>(1,4) = 0.5*KF_LOOP_DURATION_SEC*KF_LOOP_DURATION_SEC;
    KF_Obs->controlMatrix.at<float>(2,5) = 0.5*KF_LOOP_DURATION_SEC*KF_LOOP_DURATION_SEC;
    KF_Obs->controlMatrix.at<float>(3,6) = KF_LOOP_DURATION_SEC;

    KF_Obs->measurementMatrix = Mat::zeros(4,4,CV_32F);
    setIdentity(KF_Obs->measurementMatrix);

    setIdentity(KF_Obs->processNoiseCov,1e-3);

    KF_Obs->processNoiseCov.at<float>(0,0) = 1e-3;
    KF_Obs->processNoiseCov.at<float>(1,1) = 1e-3;
    KF_Obs->processNoiseCov.at<float>(2,2) = 1e-3;

//    KF_Obs->processNoiseCov.at<float>(3,3) = 1e-3;
//    KF_Obs->processNoiseCov.at<float>(4,4) = 1e-3;
//    KF_Obs->processNoiseCov.at<float>(5,5) = 1e-3;


    setIdentity(KF_Obs->measurementNoiseCov,5);
    KF_Obs->measurementNoiseCov.at<float>(0,0) = 1;
    KF_Obs->measurementNoiseCov.at<float>(1,1) = 1;
    KF_Obs->measurementNoiseCov.at<float>(2,2) = 1;

//    KF_Obs->measurementNoiseCov.at<float>(3,3) = 0.001;
//    KF_Obs->measurementNoiseCov.at<float>(4,4) = 0.001;
//    KF_Obs->measurementNoiseCov.at<float>(5,5) = 0.001;

    // YAW Angular Velocity
    YAW_Vel_Filter.DT = LOOP_DURATION_SEC;
    YAW_Vel_Filter.F_pass = 10;
    YAW_last = 0;
    YAW_Vel = 0;

    // Height Angular Velocity
    Height_Vel_Filter.DT = LOOP_DURATION_SEC;
    Height_Vel_Filter.F_pass = 8;
    Height_last = 0;
    Height_Vel = 0;


    //.......................

    setControlSource(CONTROL_NONE);
    ui->Auto_CB->setDisabled(1);
    ui->Auto_CB->setChecked(0);

    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    connect(ui->land_pb,SIGNAL(clicked()),this,SLOT(Land_Pb_Event()));
    connect(ui->takeoff_pb,SIGNAL(clicked()),this,SLOT(Takeoff_Pb_Event()));
    connect(ui->flat_pb,SIGNAL(clicked()),this,SLOT(FlatTrim_event()));
    connect(ui->togglecam_pb,SIGNAL(clicked()),this,SLOT(ToggleCam_event()));
    connect(ui->manual_rb,SIGNAL(released()),this,SLOT(CheckBoxes_Event()));
    connect(ui->none_rb,SIGNAL(released()),this,SLOT(CheckBoxes_Event()));
    connect(ui->height_rb,SIGNAL(released()),this,SLOT(CheckBoxes_Event()));
    connect(ui->pos_rb,SIGNAL(released()),this,SLOT(CheckBoxes_Event()));
    connect(ui->Auto_CB,SIGNAL(toggled(bool)),this,SLOT(Auto_Control_Event()));

    connect(ui->orb_pb,SIGNAL(clicked()),this,SLOT(ORB_Event()));
    connect(ui->slam_calib_pb,SIGNAL(clicked()),this,SLOT(slam_calib_Event()));

    connect(ui->pos_setpoint_pb,SIGNAL(clicked()),this,SLOT(pos_setpoint_pb_event()));

    pos_setpoint_pb_event();
    setFocus();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::run()
{
    sub = n.subscribe("chatter", 1000,&MainWindow::chatterCallback,this);
    spin();
}

void MainWindow::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

ControlCommand MainWindow::calcKBControl()
{
    float theta = 1.0*(3.14/180.0);
    // clear keys that have not been refreshed for 1s, it is set to "not pressed"
    for(int i=0;i<8;i++)
        isPressed[i] = isPressed[i] && ((lastRepeat[i] + 1000) > getMS());

    ControlCommand c;

    if(isPressed[0]) c.roll = -sensRP; // a
    if(isPressed[1]) c.pitch = sensRP; // s
    if(isPressed[2]) c.roll = sensRP; // d
    if(isPressed[3]) c.pitch = -sensRP; // w
    if(isPressed[4])
    {
        if(currentControlSource >= CONTROL_POS)
        {
//            YAW_setpoint -= 2;
            pcl::PointXYZ tmp_vec;
            tmp_vec.x = cos(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.x - sin(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.z;
            tmp_vec.y = img_thr->Seg_OutPut.Robot_Heading_setpoint.y;
            tmp_vec.z = sin(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.x + cos(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.z;
            img_thr->Seg_OutPut.Robot_Heading_setpoint = tmp_vec;
        }
        else
            c.yaw = -sensYaw; // left
    }
    if(isPressed[5])
    {
        if(currentControlSource >= CONTROL_POS)
        {
//            YAW_setpoint += 2;
            pcl::PointXYZ tmp_vec;
            tmp_vec.x = cos(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.x + sin(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.z;
            tmp_vec.y = img_thr->Seg_OutPut.Robot_Heading_setpoint.y;
            tmp_vec.z = -sin(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.x + cos(theta)*img_thr->Seg_OutPut.Robot_Heading_setpoint.z;
            img_thr->Seg_OutPut.Robot_Heading_setpoint = tmp_vec;
        }
        else
            c.yaw = sensYaw; // right
    }
    if(isPressed[6]) c.gaz = sensGaz; // up
    if(isPressed[7]) c.gaz = -sensGaz; // down

    c.useHovering = 1;

    return c;
}

void MainWindow::setControlSource(int type)
{
    if(type == CONTROL_KB)
    {
        ui->manual_rb->setChecked(true);
        currentControlSource = CONTROL_KB;
    }
    else if(type == CONTROL_NONE)
    {
        ui->none_rb->setChecked(true);
        currentControlSource = CONTROL_NONE;
    }
    else if(type == CONTROL_HEIGHT)
    {
        ui->height_rb->setChecked(true);
        currentControlSource = CONTROL_HEIGHT;
    }
}

void MainWindow::MainTimerEvent()
{
    //    if(flag)
    //    {
    //        ROS_INFO("2ID::%ld",QThread::currentThreadId());
    //        qDebug()<<"GUI THR::"<<QThread::currentThreadId();
    //        flag = 0;
    //    }


//    img_thr->Camera_Posotion.slope = 222.669;
//    img_thr->Camera_Posotion.Calib_P0_z.x = 0.215814;
//    img_thr->Camera_Posotion.Calib_P0_z.y = 47;
//    img_thr->Camera_Posotion.Calib_P0_x.x = 0.0634068;
//    img_thr->Camera_Posotion.Calib_P0_y.x = -0.110218 ;
//    img_thr->Camera_Posotion.SLAM_Calib_stage = SLAM_CALIB_FINISHED;

    QByteArray message_print;

    if(ros_bridge->PRINT_Buff.ReadFromBufferOut(message_print))
    {
        ui->msg_plainTextEdit->appendPlainText(message_print);
    }

    if(img_thr->Camera_Posotion.status >2)
    {
        ui->orb_label->setText("Unknown");
    }
    else if(img_thr->Camera_Posotion.status ==0)
    {
        ui->orb_label->setText("Re-Init");
    }
    else if(img_thr->Camera_Posotion.status ==1)
    {
        ui->orb_label->setText("OK");
    }

    //Estimate The YAW Angle....................
    YAW_Vel = ((img_thr->AR_Status.Rz) - YAW_last)/KF_LOOP_DURATION_SEC;
    YAW_Vel_Filter.filter(YAW_Vel);
    YAW_Vel = YAW_Vel_Filter.data;
    YAW_last = (img_thr->AR_Status.Rz);

    if(fabs(YAW_Vel) > (1000))
        YAW_Vel = last_YAW_Vel;
    last_YAW_Vel = YAW_Vel;

    predicted_YAW_Vel = (180.0/3.1415)*(img_thr->Camera_Posotion.raw_PSI - predicted_YAW_last)/KF_LOOP_DURATION_SEC;
    if(fabs(predicted_YAW_Vel) > (10000))
    {
        YAW_Vel = predicted_YAW_Vel;
    }
    predicted_YAW_last = (img_thr->Camera_Posotion.raw_PSI);

    //Estimate The Height velocity....................
    Height_Vel = ((img_thr->AR_Status.height) - Height_last)/KF_LOOP_DURATION_SEC;
    Height_Vel_Filter.filter(Height_Vel);
    Height_Vel = Height_Vel_Filter.data;
    Height_last = (img_thr->AR_Status.height);

    if(img_thr->Camera_Posotion.SLAM_Calib_stage == SLAM_CALIB_FINISHED)
    {
//        qDebug()<<img_thr->Camera_Posotion.slope << img_thr->Camera_Posotion.Calib_P0_z.x <<img_thr->Camera_Posotion.Calib_P0_z.y
//               << img_thr->Camera_Posotion.Calib_P0_x.x <<img_thr->Camera_Posotion.Calib_P0_y.x;
        KF_Control.at<float>(0) = img_thr->AR_Status.acc_pure_z*9.8*100;

        KF_Obs_Control.at<float>(0) = img_thr->AR_Status.Global_v_x*1.1;
        KF_Obs_Control.at<float>(1) = img_thr->AR_Status.Global_v_y*1.1;
        KF_Obs_Control.at<float>(2) = Height_Vel*1.1;
        KF_Obs_Control.at<float>(3) = img_thr->AR_Status.Global_acc_x*12*100;
        KF_Obs_Control.at<float>(4) = img_thr->AR_Status.Global_acc_y*12*100;
        KF_Obs_Control.at<float>(5) = img_thr->AR_Status.Global_acc_z*12*100;
        KF_Obs_Control.at<float>(6) = YAW_Vel*1.1;


        if(img_thr->Camera_Posotion.status ==1)
        {
            KF_Measurment.at<float>(0) = img_thr->Camera_Posotion.modified_z;
            setIdentity(KF->processNoiseCov,3e-3);

            KF_Obs_Measurment.at<float>(0) = img_thr->Camera_Posotion.modified_x;
            KF_Obs_Measurment.at<float>(1) = img_thr->Camera_Posotion.modified_y;
            KF_Obs_Measurment.at<float>(2) = img_thr->Camera_Posotion.modified_z;
//            KF_Obs_Measurment.at<float>(2) = img_thr->AR_Status.height;
            KF_Obs_Measurment.at<float>(3) = img_thr->Camera_Posotion.raw_PSI*(180.0/3.1415);

        }
        else
        {
            KF_Measurment.at<float>(0) = KF_state.at<float>(0);
            setIdentity(KF->processNoiseCov,0.1);

            KF_Obs_Measurment.at<float>(0) = img_thr->Camera_Posotion.predicted_x;
            KF_Obs_Measurment.at<float>(1) = img_thr->Camera_Posotion.predicted_y;
            KF_Obs_Measurment.at<float>(2) = img_thr->Camera_Posotion.predicted_z;
            KF_Obs_Measurment.at<float>(3) = KF_Obs_state.at<float>(3);
        }

        KF->predict(KF_Control);
        KF_state =  KF->correct(KF_Measurment);

        KF_Obs->predict(KF_Obs_Control);
        KF_Obs_state = KF_Obs->correct(KF_Obs_Measurment);

        img_thr->Camera_Posotion.predicted_x = KF_Obs_state.at<float>(0);
        img_thr->Camera_Posotion.predicted_y = KF_Obs_state.at<float>(1);
        img_thr->Camera_Posotion.predicted_z = KF_Obs_state.at<float>(2);
        img_thr->Camera_Posotion.predicted_raw_PSI = KF_Obs_state.at<float>(3);

        ui->cam_x_label->setText(QString::number(img_thr->Camera_Posotion.predicted_x));
        ui->cam_y_label->setText(QString::number(img_thr->Camera_Posotion.predicted_y));
        ui->cam_z_label->setText(QString::number(img_thr->Camera_Posotion.predicted_z));
        ui->YAW_label->setText(QString::number(img_thr->Camera_Posotion.predicted_raw_PSI));

    }
//    plotter_thr->plot(1,img_thr->Camera_Posotion.modified_z);
//    plotter_thr->plot(2,img_thr->Camera_Posotion.predicted_z);
//    plotter_thr->plot(3,Height_Vel);
    plotter_thr->plot(4,img_thr->Camera_Posotion.predicted_raw_PSI);

    //Estimate The YAW Angle....................

    KF_YAW_Control.at<float>(0) = YAW_Vel;

    if(img_thr->Camera_Posotion.status ==1)
    {
        KF_YAW_Measurment.at<float>(0) =  img_thr->Camera_Posotion.PSI*(180/3.1415);
        KF_YAW->predict(KF_YAW_Control);
        KF_YAW_state =  KF_YAW->correct(KF_YAW_Measurment);

        img_thr->Camera_Posotion.predicted_PSI = KF_YAW_state.at<float>(0);
    }

    if(ORB_IMG_flag)
    {
        if(img_thr->ORB_IMG_Buff.ReadFromBufferOut(frame))
        {
            Mat tmp;
            cv::resize(frame,tmp,cv::Size(640,380));
            if(tmp.channels() == 3)
                cvtColor(tmp,tmp,COLOR_RGB2BGR);
            frame_img =  Mat2QImage(tmp);
            ui->IMG_label->setPixmap(QPixmap::fromImage(frame_img));
            ui->IMG_label->adjustSize();
        }
    }
    else if(img_thr->IMG_Buff.ReadFromBufferOut(frame))
    {
        Mat tmp;
        cv::resize(frame,tmp,cv::Size(640,360));
        if(tmp.channels() == 3)
            cvtColor(tmp,tmp,COLOR_RGB2BGR);
        frame_img =  Mat2QImage(tmp);
        ui->IMG_label->setPixmap(QPixmap::fromImage(frame_img));
        ui->IMG_label->adjustSize();
    }

    ui->batt_label->setText(QString::number(img_thr->AR_Status.Batt));
    ui->rx_label->setText(QString::number(img_thr->AR_Status.Rx));
    ui->ry_label->setText(QString::number(img_thr->AR_Status.Ry));
    ui->rz_label->setText(QString::number(img_thr->AR_Status.Rz));
    ui->height_label->setText(QString::number(img_thr->AR_Status.height));
    ui->ax_label->setText(QString::number(img_thr->AR_Status.acc_x));
    ui->ay_label->setText(QString::number(img_thr->AR_Status.acc_y));
    ui->az_label->setText(QString::number(img_thr->AR_Status.acc_z));
    ui->vx_label->setText(QString::number(img_thr->AR_Status.v_x));
    ui->vy_label->setText(QString::number(img_thr->AR_Status.v_y));
    ui->vz_label->setText(QString::number(img_thr->AR_Status.v_z));


//    plotter_thr->plot(3,img_thr->AR_Status.v_x);
//    plotter_thr->plot(4,img_thr->AR_Status.acc_pure_x*9.8*10);
    //    plotter_thr->plot(2,acc_staric_z);
    //    plotter_thr->plot(3,img_thr->AR_Status.acc_static_z);
    //    qDebug()<<img_thr->AR_Status.acc_x<<img_thr->AR_Status.acc_y;


}

void MainWindow::Takeoff_Pb_Event()
{
    ros_bridge->send_Takeoff_msg();
}

void MainWindow::CheckBoxes_Event()
{
    uint s = CONTROL_NONE;

    if(ui->manual_rb->isChecked())
    {
        s = CONTROL_KB;
        ui->msg_plainTextEdit->appendPlainText("Manual_Control");
        QCheckBox kk;
        ui->Auto_CB->setDisabled(1);
        ui->Auto_CB->setChecked(0);
    }
    if(ui->none_rb->isChecked())
    {
        s = CONTROL_NONE;
        ui->msg_plainTextEdit->appendPlainText("None_Control");
        ui->Auto_CB->setDisabled(1);
        ui->Auto_CB->setChecked(0);
    }
    if(ui->height_rb->isChecked())
    {
        s = CONTROL_HEIGHT;
        ui->msg_plainTextEdit->appendPlainText("Height_Control");
        ui->Auto_CB->setDisabled(1);
        ui->Auto_CB->setChecked(0);
    }
    if(ui->pos_rb->isChecked())
    {
        s = CONTROL_POS;
        YAW_setpoint = 0;
        ui->msg_plainTextEdit->appendPlainText("POS_Control");
        ui->Auto_CB->setDisabled(0);
    }
    currentControlSource = s;
    setFocus();
}

void MainWindow::FlatTrim_event()
{
    ros_bridge->sendFlattrim();
}

void MainWindow::ToggleCam_event()
{
    ros_bridge->sendToggleCam();
}

void MainWindow::ORB_Event()
{
    if(!QString::compare(ui->orb_pb->text(),"ORB_IMG_On"))
    {
        ui->orb_pb->setText("ORB_IMG_Off");
        ORB_IMG_flag = 0;
    }
    else if(!QString::compare(ui->orb_pb->text(),"ORB_IMG_Off"))
    {
        ui->orb_pb->setText("ORB_IMG_On");
        ORB_IMG_flag = 1;
    }
}

void MainWindow::slam_calib_Event()
{
    currentControlSource = CONTROL_SLAM_CALIB;
    ui->msg_plainTextEdit->appendPlainText("Calibrating CAM POS");
    img_thr->Camera_Posotion.SLAM_Calib_stage = 0;

    setFocus();
}

void MainWindow::pos_setpoint_pb_event()
{
    Height_setpoint = ui->height_cn_num->currentText().toFloat();
    Y_setpoint = ui->y_cn_num->currentText().toFloat() + img_thr->Camera_Posotion.Calib_P0_y.x*img_thr->Camera_Posotion.slope;
    X_setpoint = ui->x_cn_num->currentText().toFloat() + img_thr->Camera_Posotion.Calib_P0_x.x*img_thr->Camera_Posotion.slope;
    Z_setpoint = Height_setpoint - img_thr->Camera_Posotion.Calib_P0_z.y + img_thr->Camera_Posotion.Calib_P0_z.x*img_thr->Camera_Posotion.slope;

    qDebug()<<X_setpoint<<Y_setpoint<<Z_setpoint;
}

void MainWindow::Auto_Control_Event()
{
    if(currentControlSource >= CONTROL_POS)
    {
        if(ui->Auto_CB->isChecked())
        {
            currentControlSource = CONTROL_Auto;
            ui->msg_plainTextEdit->appendPlainText("Auto_Control");
        }
        else
        {
            currentControlSource = CONTROL_POS;
            ui->msg_plainTextEdit->appendPlainText("POS_Control");
        }
    }
    setFocus();
}

void MainWindow::Land_Pb_Event()
{
    ros_bridge->send_Land_msg();
}

void MainWindow::keyPressEvent(QKeyEvent *key)
{
    if(currentControlSource >=CONTROL_KB && currentControlSource <=CONTROL_Auto )
    {
        int idx = mapKey(key->key());

        if(idx >= 0)
        {
            bool changed = !isPressed[idx];

            isPressed[idx] = true;
            lastRepeat[idx] = getMS();

            if(changed)
                ros_bridge->sendControlToDrone(calcKBControl());
        }

        else if(key->key() == Qt::Key_PageUp)	// page up
            ros_bridge->send_Takeoff_msg();

        else if(key->key() == Qt::Key_PageDown)	// page down
        {
            ros_bridge->send_Land_msg();
            setFocus();
            setControlSource(CONTROL_KB);
        }
    }

    if(key->key() == Qt::Key_Escape)	// ESC
    {
        setFocus();
        setControlSource(CONTROL_KB);
        CheckBoxes_Event();

    }

    if(key->key() == Qt::Key_Home)	// F2
    {
        setFocus();
        setControlSource(CONTROL_HEIGHT);
        CheckBoxes_Event();

    }

    //    if(key->key() == 16777264)	// F1
    //    {
    //        ros_bridge->sendToggleState();
    //    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *key)
{
    if(currentControlSource >=CONTROL_KB && currentControlSource <=CONTROL_POS)
    {
        int idx = mapKey(key->key());
        if(idx >= 0)
        {
            bool changed = false;
            if(!key->isAutoRepeat())	// ignore autorepeat-releases (!)
            {
                changed = isPressed[idx];
                isPressed[idx] = false;
            }

            if(changed)
                ros_bridge->sendControlToDrone(calcKBControl());
        }
    }
}

// KB control stuff
int MainWindow::mapKey(int k)
{
    //    qDebug("key");
    setFocus();
    switch(k)
    {
    case Qt::Key_A: //
        return 0;
    case Qt::Key_S: //
        return 1;
    case Qt::Key_D: //
        return 2;
    case Qt::Key_W: //
        return 3;
    case Qt::Key_Left: // left
        return 4;
    case Qt::Key_Right: // right
        return 5;
    case Qt::Key_Up: // up
        return 6;
    case Qt::Key_Down: // down
        return 7;
    }


    return -1;
}

QImage MainWindow::Mat2QImage(const cv::Mat &src)
{
    cv::Mat temp; // make the same cv::Mat
    //    cvtColor(src, temp,CV_BGR2GRAY); // cvtColor Makes a copt, that what i need
    cvtColor(src, temp,CV_BGR2RGB);
    QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    //    QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_Indexed8);

    QImage dest2(dest);
    dest2.detach(); // enforce deep copy
    return dest2;
}
