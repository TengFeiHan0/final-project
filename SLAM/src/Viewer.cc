/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <QFile>
#include <QString>
#include <QTextStream>
#include <QIODevice>
#include <mutex>
#include "Optimizer.h"

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    //my extention...............
    lost_counter = 0;
    IT = new image_transport::ImageTransport(nh);
    SLAM_IMG_PUB = IT->advertise("ORB_SLAM/Frame",10);
    Key_Frame_IMG_PUB = IT->advertise("ORB_SLAM/Key_Frame",10);

    SFM_pub = nh.advertise<SLAM::SFM_msg>("ORB_SLAM/SFM_Data",10);
}

void Viewer::Run()
{
    mbFinished = false;

    //    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    //    // 3D Mouse handler requires depth testing to be enabled
    //    glEnable(GL_DEPTH_TEST);

    //    // Issue specific OpenGl we might need
    //    glEnable (GL_BLEND);
    //    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",false,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    //    // Define Camera Render Object (for view / scene browsing)
    //    pangolin::OpenGlRenderState s_cam(
    //                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
    //                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
    //                );

    //    // Add named OpenGL viewport to window and provide 3D Handler
    //    pangolin::View& d_cam = pangolin::CreateDisplay()
    //            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
    //            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    mpSystem->DeactivateLocalizationMode();
    int k_last_size = mpMapDrawer->mpMap->GetAllKeyFrames().size();
    int file_counter = 0;
    while(1)
    {
        //        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        //        if(menuFollowCamera && bFollow)
        //        {
        //            s_cam.Follow(Twc);
        //        }
        //        else if(menuFollowCamera && !bFollow)
        //        {
        //            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
        //            s_cam.Follow(Twc);
        //            bFollow = true;
        //        }
        //        else if(!menuFollowCamera && bFollow)
        //        {
        //            bFollow = false;
        //        }

        //        if(menuLocalizationMode && !bLocalizationMode)
        //        {
        //            mpSystem->ActivateLocalizationMode();
        //            bLocalizationMode = true;
        //        }
        //        else if(!menuLocalizationMode && bLocalizationMode)
        //        {
        //            mpSystem->DeactivateLocalizationMode();
        //            bLocalizationMode = false;
        //        }

        //        d_cam.Activate(s_cam);
        //        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //        mpMapDrawer->DrawCurrentCamera(Twc);
        //        if(menuShowKeyFrames || menuShowGraph)
        //                    mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        //        if(menuShowPoints)
        //            mpMapDrawer->DrawMapPoints();

        //my extention................

        int k_size = mpMapDrawer->mpMap->GetAllKeyFrames().size();
        if(k_size!=k_last_size)
        {

            MKey_Frame = cv_bridge::CvImage(std_msgs::Header(),"mono8",mpTracker->mImGray).toImageMsg();
            Key_Frame_IMG_PUB.publish(MKey_Frame);
        }
        k_last_size = k_size;
        if(mpFrameDrawer->SlamGetState() == Tracking::LOST)
            lost_counter++;
        else
            lost_counter = 0;

        if(lost_counter > (1000*0.3)/mT )
            mpMapDrawer->PublishSLAMData(Twc,0);
        else
            mpMapDrawer->PublishSLAMData(Twc,1);
        //..................

        //        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();

        ORB_Frame = cv_bridge::CvImage(std_msgs::Header(),"bgr8",im).toImageMsg();
        SLAM_IMG_PUB.publish(ORB_Frame);

        cv::imshow("ORB-SLAM2: Current Frame",im);
        int key = cv::waitKey(mT);

        if(key == 102)
        {
//            Optimizer::LocalBundleAdjustment(mpTracker->mpLocalMapper->mpCurrentKeyFrame,0, mpMapDrawer->mpMap);
//            Optimizer::GlobalBundleAdjustemnt(mpMapDrawer->mpMap,20);
            cv::Mat Pwc(4,4,CV_32F);
            mpMapDrawer->GetCurrentCameraMatrix(Pwc);

            SLAM::SFM_msg SLAM_Data;
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<4;j++)
                    SLAM_Data.P.push_back(Pwc.at<float>(i,j));
            }
            SLAM_Data.image = *cv_bridge::CvImage(std_msgs::Header(),"bgr8",mpTracker->RawIMG).toImageMsg();
            SFM_pub.publish(SLAM_Data);

//            QString textfilename="/home/omid/my_workspace/SLAM/bin/Data"+QString::number(file_counter)+".txt";
//            QString imgfilename="/home/omid/my_workspace/SLAM/bin/img"+QString::number(file_counter)+".png";
//            QFile file( textfilename );
//            if ( file.open(QIODevice::WriteOnly) )
//            {
//                QTextStream stream( &file );
//                stream << "P:"<<Pwc.at<float>(0,0)<<","<<Pwc.at<float>(0,1)<<","<<Pwc.at<float>(0,2)<<","<<Pwc.at<float>(0,3)<<",\n"
//                              <<Pwc.at<float>(1,0)<<","<<Pwc.at<float>(1,1)<<","<<Pwc.at<float>(1,2)<<","<<Pwc.at<float>(1,3)<<",\n"
//                              <<Pwc.at<float>(2,0)<<","<<Pwc.at<float>(2,1)<<","<<Pwc.at<float>(2,2)<<","<<Pwc.at<float>(2,3)<<",\n"
//                              <<Pwc.at<float>(3,0)<<","<<Pwc.at<float>(3,1)<<","<<Pwc.at<float>(3,2)<<","<<Pwc.at<float>(3,3)<<"\n";
//                file.close();
//            }
//            cv::imwrite(imgfilename.toAscii().data(),mpTracker->RawIMG);
//            ROS_ERROR("Key:%d counter:%d",key,file_counter);
//            file_counter++;
        }

        if(menuReset)
        {
            menuShowGraph = false;
            menuShowKeyFrames = false;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
