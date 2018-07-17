#ifndef CAPTUREVIEWER_H
#define CAPTUREVIEWER_H

#include <QObject>

#include"seg_fcn.h"
#include <QTimer>
#include "viewer.h"
#include <QDate>
#include<QTime>
#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Image.h>
#include<image_transport/image_transport.h>
#include<QPixmap>
#include<QImage>
#include "listnerthr.h"
#include "CreatBuffer.h"

class Viewer;
class listnerthr;

class CaptureViewer : public QObject
{
    typedef QImage _type2;
    Q_OBJECT
public:
    explicit CaptureViewer(QObject *parent = 0);
    ~CaptureViewer();
    QTimer *MainTimer;
    float loop_timer;

    Viewer* viewer_thr;
    listnerthr* listner_thr;

    VideoWriter  PCL_VideoWriter,SLAM_VideoWriter,RAW_VideoWriter;
    int init_cap_flag;
    int Raw_IMG_Capture_flag,SLAM_IMG_Capture_flag,PCL_IMG_Capture_flag;
    bool color_correction_flag;
    cv::Mat Frame,SLAM_IMG,RAW_IMG;
    QImage Raw_QImage;
    bool busy_flag;
    CreatBuffer<cv::Mat> Raw_IMG_buff;
    CreatBuffer<cv::Mat> SLAM_IMG_buff;


    void thr_start();
    void _other(_type2 mm);
    void InitVideoRecorder();
    void StopCaptureVideo();

    cv::Mat QImage2Mat(QImage src)
    {
        QImage myImage=src;
        cv::Mat tmp(src.height(),src.width(),CV_8UC4,src.scanLine(0)); //RGB32 has 8 bits of R, 8 bits of G, 8 bits of B and 8 bits of Alpha. It's essentially RGBA.
        cv::Mat MatOut(src.height(),src.width(),CV_8UC3); //RGB32 has 8 bits of R, 8 bits of G, 8 bits of B and 8 bits of Alpha. It's essentially RGBA.

        cvtColor(tmp,MatOut,CV_RGBA2RGB);
        return MatOut;
    }

    QImage Mat2QImage(cv::Mat const& src);

signals:
    void thr_start_signal();
    void _otherSignal(_type2 mm);

protected slots:
    void MainTimerEvent();
    void Run_THR(void);
    void _otherSlot(_type2 mm);
};

#endif // CAPTUREVIEWER_H
