#include <QApplication>
#include <ros/ros.h>
#include <QThread>

#include "viewer.h"
#include "listnerthr.h"
#include "planning.h"
#include "captureviewer.h"
#include "CreatBuffer.h"
#include "frontier.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc,argv,"exploration");

    QThread thr1,thr2,thr3,thr4;
    Viewer viewer_THR;
    listnerthr listner_THR;
    Planning planning_THR;
    CaptureViewer capture_THR;
    Frontier frontier_THR;

    viewer_THR.listner_thr = &listner_THR;
    viewer_THR.planning_thr = &planning_THR;
    viewer_THR.capture_thr = &capture_THR;
    viewer_THR.frontier_thr = &frontier_THR;
    planning_THR.listner_thr = &listner_THR;
    planning_THR.viewer_thr = &viewer_THR;
    planning_THR.frontier_thr = &frontier_THR;
    listner_THR.planning_thr = &planning_THR;
    listner_THR.viewer_thr = &viewer_THR;
    listner_THR.capture_thr = &capture_THR;
    capture_THR.viewer_thr = &viewer_THR;
    capture_THR.listner_thr = &listner_THR;
    frontier_THR.viewer_thr = &viewer_THR;
    frontier_THR.planning_thr = &planning_THR;

    viewer_THR.loop_time = 60;
    planning_THR.loop_time = 50;
    capture_THR.loop_timer = 25;
    frontier_THR.loop_time = 50;


    listner_THR.moveToThread(&thr1);
    planning_THR.moveToThread(&thr2);
    capture_THR.moveToThread(&thr3);
    frontier_THR.moveToThread(&thr4);

    thr1.start();
    thr2.start();
    thr3.start();
    thr4.start();

//    listner_THR.LoadCloudFromFile("DataLoggedFile");//DataLoggedFile//library2
//    listner_THR.OnlyGrabCloud();

    viewer_THR.thr_start();
    listner_THR.thr_start();
    planning_THR.thr_start();
    capture_THR.thr_start();
    frontier_THR.thr_start();

    viewer_THR.setFixedSize(850,540);
    viewer_THR.show();


//    listner_THR.SimulationMode();

    return a.exec();
}


