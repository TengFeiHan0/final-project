#include "captureviewer.h"

CaptureViewer::CaptureViewer(QObject *parent) : QObject(parent)
{
    loop_timer = 25;
    MainTimer = new QTimer(this);


    init_cap_flag = 0;
    color_correction_flag = 1;
    PCL_IMG_Capture_flag = Raw_IMG_Capture_flag = SLAM_IMG_Capture_flag = 0;
    SLAM_IMG_buff.initBuffer(30);
    Raw_IMG_buff.initBuffer(30);


    qRegisterMetaType<_type2>("_type2");

    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    connect(this,SIGNAL(thr_start_signal()),this,SLOT(Run_THR()));
    connect(this,SIGNAL(_otherSignal(_type2)),this,SLOT(_otherSlot(_type2)));
}

CaptureViewer::~CaptureViewer()
{
    SLAM_VideoWriter.release();
    PCL_VideoWriter.release();
    RAW_VideoWriter.release();
}

void CaptureViewer::thr_start()
{
    emit thr_start_signal();
}

void CaptureViewer::_other(_type2 mm)
{
    emit _otherSignal(mm);
}

void CaptureViewer::InitVideoRecorder()
{
    if(init_cap_flag == 1)
    {
        QDate CurrDate;
        QTime CurrTime;
        QString CurrDateText = CurrDate.currentDate().toString("yy_MM_dd");
        QString CurrTimeText = CurrTime.currentTime().toString("hh:mm:ss");
        QString FileName =  "Video/"+CurrDateText+"-"+CurrTimeText+"-PCL"+".avi";
        int FPS = 1000/viewer_thr->loop_time;
        PCL_VideoWriter.open(FileName.toAscii().data(),CV_FOURCC('M','P','4','2'),FPS,cv::Size(640,480));
        //        MainTimer->start(viewer_thr->loop_time);

        if(SLAM_IMG_Capture_flag == 1)
        {
            FileName.clear();
            FileName =  "Video/"+CurrDateText+"-"+CurrTimeText+"-SLAM"+".avi";
            SLAM_VideoWriter.open(FileName.toAscii().data(),CV_FOURCC('M','P','4','2'),30,SLAM_IMG.size());
        }

        if(Raw_IMG_Capture_flag == 1)
        {
            FileName.clear();
            FileName =  "Video/"+CurrDateText+"-"+CurrTimeText+"-RAW"+".avi";
            RAW_VideoWriter.open(FileName.toAscii().data(),CV_FOURCC('M','P','4','2'),30,RAW_IMG.size());
        }
        init_cap_flag = 0;
        PCL_IMG_Capture_flag = 1;
    }

}

void CaptureViewer::StopCaptureVideo()
{
    PCL_IMG_Capture_flag = Raw_IMG_Capture_flag = SLAM_IMG_Capture_flag = 2;
}


QImage CaptureViewer::Mat2QImage(const Mat &src)
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

void CaptureViewer::MainTimerEvent()
{
//    busy_flag = 1;
//    if(!listner_thr->SLAM_IMG.empty())
//        SLAM_IMG = listner_thr->SLAM_IMG.clone();
//    if(!listner_thr->RAW_IMG.empty())
//        RAW_IMG = listner_thr->RAW_IMG.clone();
//    busy_flag = 0;


    if(Raw_IMG_buff.ReadFromBufferIn(RAW_IMG))
    {
        if(color_correction_flag)
        {
            cv::Mat tmp;
            cv::cvtColor(RAW_IMG,tmp,CV_BGR2RGB);
            RAW_IMG = tmp.clone();
        }
        Raw_QImage = Mat2QImage(RAW_IMG);
        viewer_thr->Udpade_Raw_IMG(Raw_QImage);
        if(RAW_VideoWriter.isOpened() && Raw_IMG_Capture_flag == 1)
            RAW_VideoWriter << RAW_IMG;
        else if(Raw_IMG_Capture_flag == 2)
        {
            RAW_VideoWriter.release();
            Raw_IMG_Capture_flag = 0;
        }
    }

    if(SLAM_IMG_buff.ReadFromBufferIn(SLAM_IMG))
    {
        if(SLAM_VideoWriter.isOpened() && SLAM_IMG_Capture_flag == 1)
            SLAM_VideoWriter << SLAM_IMG;
        else if(SLAM_IMG_Capture_flag == 2)
        {
            SLAM_VideoWriter.release();
            SLAM_IMG_Capture_flag = 0;
        }
    }
}

void CaptureViewer::Run_THR()
{
    MainTimer->start(loop_timer);
}

void CaptureViewer::_otherSlot(_type2 mm)
{
    if(init_cap_flag)
    {
        InitVideoRecorder();
    }else if(PCL_IMG_Capture_flag == 1)
    {
        cv::Mat frame = QImage2Mat(mm);
        PCL_VideoWriter << frame;
    }
    else if(PCL_IMG_Capture_flag == 2)
    {
        PCL_IMG_Capture_flag = 0;
        PCL_VideoWriter.release();
    }
}

