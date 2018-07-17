#ifndef CREATDATABUFFER_H
#define CREATDATABUFFER_H

#include <QObject>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<QDebug>

using namespace cv;
using namespace std;

class CreatDataBuffer : public QObject
{
    Q_OBJECT
public:
    explicit CreatDataBuffer(QObject *parent = 0);
    vector<QByteArray> BufferOut;
    int bufferSize,bufferOutPointer,overFlowOut;
    int bufferOutCurrentPointer;
    bool ImgReadyFlag;
    bool initBufferFlag;
    void initBuffer(int buffSize);
    void Write2BufferOut(QByteArray Data);
    bool ReadFromBufferOut(QByteArray &Data);
    void resetBufferPointer();



signals:
    
public slots:
    
};

#endif // CREATDATABUFFER_H
