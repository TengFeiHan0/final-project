#ifndef CREATBUFFER_H
#define CREATBUFFER_H

#include <QObject>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<QDebug>
#include<typeinfo>

using namespace std;

template<class T>
class CreatBuffer
{
public:
    explicit CreatBuffer();
    vector<T> BufferIn,BufferOut;
    int bufferSize,bufferInPointer,bufferOutPointer,overFlowIn,overFlowOut;
    int bufferInCurrentPointer,bufferOutCurrentPointer;
    bool initBufferFlag;
    void initBuffer(int buffSize);
    void Write2BufferIn(T Frame);
    void Write2BufferOut(T Frame);
    bool ReadFromBufferIn(T &Frame);
    bool ReadFromBufferOut(T &Frame);
    void resetBufferPointer();



};

template<class T>
CreatBuffer<T>::CreatBuffer()
{
    initBufferFlag = 0;
}

template<class T>
void CreatBuffer<T>::initBuffer(int buffSize)
{

    bufferSize = buffSize;
    bufferInPointer = 0;
    bufferOutPointer = 0;
    bufferOutCurrentPointer = 0;
    bufferInCurrentPointer = 0;
    overFlowIn = 0;
    overFlowOut = 0;
    BufferIn.resize(bufferSize);
    BufferOut.resize(bufferSize);
    initBufferFlag = 1;


}

template<class T>
void CreatBuffer<T>::Write2BufferIn(T Frame)
{
    if(initBufferFlag)
    {
#ifdef MAT_BUFF
        BufferIn[bufferInPointer] = Frame.clone();
        qDebug("MAT");
#else
        BufferIn[bufferInPointer] = Frame;
#endif

        bufferInPointer++;

        if(bufferInPointer >= bufferSize)
        {
            overFlowIn++;
            bufferInPointer = 0;

            //        imshow("img",videoBuffer[BUFFERSIZE-1]);
        }
    }
}

template<class T>
void CreatBuffer<T>::Write2BufferOut(T Frame)
{
    if(initBufferFlag)
    {
#ifdef MAT_BUFF
        BufferOut[bufferOutPointer] = Frame.clone();
#else
        BufferOut[bufferOutPointer] = Frame;
#endif
        bufferOutPointer++;

        if(bufferOutPointer >= bufferSize)
        {
            overFlowOut++;
            bufferOutPointer = 0;
            //        imshow("img",videoBuffer[BUFFERSIZE-1]);
        }
    }
}

template<class T>
bool CreatBuffer<T>::ReadFromBufferIn(T &Frame)
{
    if(initBufferFlag)
    {
        int diff=0;

        if(overFlowIn>0)
        {
            diff = bufferInPointer + overFlowIn*bufferSize - bufferInCurrentPointer;
        }
        else
            diff = bufferInPointer - bufferInCurrentPointer;

        if(diff>0)
        {
#ifdef MAT_BUFF
            Frame = BufferIn[bufferInCurrentPointer].clone();
#else
            Frame = BufferIn[bufferInCurrentPointer];
#endif

            //        imshow("image",CurrentFrame);
            bufferInCurrentPointer++;
            if(bufferInCurrentPointer >= bufferSize)
            {
                bufferInCurrentPointer = 0;
                overFlowIn--;
            }
            return 1;
        }
        return 0;
    }
}

template<class T>
bool CreatBuffer<T>::ReadFromBufferOut(T &Frame)
{
    if(initBufferFlag)
    {
        int diff=0;

        if(overFlowOut>0)
        {
            diff = bufferOutPointer + overFlowOut*bufferSize - bufferOutCurrentPointer;
        }
        else
            diff = bufferOutPointer - bufferOutCurrentPointer;

        if(diff>0)
        {
#ifdef MAT_BUFF
            Frame = BufferOut[bufferOutCurrentPointer].clone();
#else
            Frame = BufferOut[bufferOutCurrentPointer];
#endif

            //        imshow("image",CurrentFrame);
            bufferOutCurrentPointer++;
            if(bufferOutCurrentPointer >= bufferSize)
            {
                bufferOutCurrentPointer = 0;
                overFlowOut--;
            }
            return 1;
        }
        return 0;
    }
}

template<class T>
void CreatBuffer<T>::resetBufferPointer()
{
    if( overFlowOut>0 || bufferOutPointer > bufferOutCurrentPointer )
    {
        overFlowOut = 0;
        bufferOutCurrentPointer = bufferOutPointer-1;

        if(bufferOutCurrentPointer<0)
        {
            overFlowOut = 1;
            bufferOutCurrentPointer = bufferSize-1;
        }

    }

    if( overFlowIn>0 || bufferInPointer > bufferInCurrentPointer )
    {
        overFlowIn = 0;
        bufferInCurrentPointer = bufferInPointer-1;

        if(bufferInCurrentPointer<0)
        {
            overFlowIn = 1;
            bufferInCurrentPointer = bufferSize-1;
        }

    }
}

#endif // CREATBUFFER_H
