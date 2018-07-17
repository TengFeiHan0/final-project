#ifndef CREATBUFFER_H
#define CREATBUFFER_H

#include <QObject>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<QDebug>

using namespace std;

class CreatBuffer : public QObject
{
    Q_OBJECT
public:
    explicit CreatBuffer(QObject *parent = 0);
    vector<cv::Mat> BufferIn,BufferOut;
    int bufferSize,bufferInPointer,bufferOutPointer,overFlowIn,overFlowOut;
    int bufferInCurrentPointer,bufferOutCurrentPointer;
    bool initBufferFlag;
    void initBuffer(int buffSize);
    void Write2BufferIn(cv::Mat Frame);
    void Write2BufferOut(cv::Mat Frame);
    bool ReadFromBufferIn(cv::Mat &Frame);
    bool ReadFromBufferOut(cv::Mat &Frame);
    void resetBufferPointer();



signals:
    
public slots:
    
};

#endif // CREATBUFFER_H
