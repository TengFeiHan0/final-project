#include "CreatDataBuffer.h"

CreatDataBuffer::CreatDataBuffer(QObject *parent) :
    QObject(parent)
{
    initBufferFlag = 0;
    ImgReadyFlag = 0;
}

void CreatDataBuffer::initBuffer(int buffSize)
{

    bufferSize = buffSize;
    bufferOutPointer = 0;
    bufferOutCurrentPointer = 0;
    overFlowOut = 0;
    BufferOut.resize(bufferSize);
    initBufferFlag = 1;


}



void CreatDataBuffer::Write2BufferOut(QByteArray Data)
{
    if(initBufferFlag)
    {

        BufferOut[bufferOutPointer] = Data;
        bufferOutPointer++;
//        qDebug()<<bufferOutPointer<<"::"<<bufferOutCurrentPointer<<"::"<<overFlowOut;
        if( bufferOutPointer >= bufferOutCurrentPointer && overFlowOut>0 )
        {
            overFlowOut =0;
            bufferOutCurrentPointer = bufferOutPointer-1;
        }
        if(bufferOutPointer >= bufferSize)
        {
            overFlowOut++;
            bufferOutPointer = 0;
            //        imshow("img",videoBuffer[BUFFERSIZE-1]);
        }
    }
}


bool CreatDataBuffer::ReadFromBufferOut(QByteArray &Data)
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

//        qDebug()<<">>"<<diff<<"<<";
        if(diff>0)
        {
            Data = BufferOut[bufferOutCurrentPointer];

            //        imshow("image",CurrentFrame);
            bufferOutCurrentPointer++;
            if(bufferOutCurrentPointer >= bufferSize)
            {
                bufferOutCurrentPointer = 0;
                overFlowOut--;
            }
            ImgReadyFlag = 1;
            return 1;
        }
        ImgReadyFlag = 0;
        return 0;
    }
}
void CreatDataBuffer::resetBufferPointer()
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
}
