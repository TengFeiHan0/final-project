#include "CreatBuffer.h"

CreatBuffer::CreatBuffer(QObject *parent) :
    QObject(parent)
{
    initBufferFlag = 0;
}

void CreatBuffer::initBuffer(int buffSize)
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

void CreatBuffer::Write2BufferIn(cv::Mat Frame)
{
    if(initBufferFlag)
    {
        BufferIn[bufferInPointer]= Frame.clone();
        bufferInPointer++;

        if(bufferInPointer >= bufferSize)
        {
            overFlowIn++;
            bufferInPointer = 0;

            //        imshow("img",videoBuffer[BUFFERSIZE-1]);
        }
    }
}

void CreatBuffer::Write2BufferOut(cv::Mat Frame)
{
    if(initBufferFlag)
    {

        BufferOut[bufferOutPointer] = Frame.clone();
        bufferOutPointer++;

        if(bufferOutPointer >= bufferSize)
        {
            overFlowOut++;
            bufferOutPointer = 0;
            //        imshow("img",videoBuffer[BUFFERSIZE-1]);
        }
    }
}

bool CreatBuffer::ReadFromBufferIn(cv::Mat &Frame)
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
            Frame = BufferIn[bufferInCurrentPointer].clone();

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

bool CreatBuffer::ReadFromBufferOut(cv::Mat &Frame)
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
            Frame = BufferOut[bufferOutCurrentPointer].clone();

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

void CreatBuffer::resetBufferPointer()
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
