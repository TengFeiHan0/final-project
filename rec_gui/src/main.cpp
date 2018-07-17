#include "mainwindow.h"
#include <QApplication>
#include <QThread>
#include "ros_thr.h"
#include "listnerthr.h"
#include "plot_publisher.h"

unsigned int ros_header_timestamp_base = 0;

int main(int argc,char **argv)
{
    QApplication a(argc, argv);

    init(argc,argv,"listner");

    QThread thr1,thr2,thr3;

    MainWindow gui_THR;
    ROS_THR ros_THR;
    listnerthr IMG_THR;
    plot_publisher PLOTTER_THR;

    gui_THR.ros_bridge = &ros_THR;
    ros_THR.gui = &gui_THR;
    gui_THR.img_thr = &IMG_THR;
    IMG_THR.ros_bridg = &ros_THR;
    IMG_THR.gui = &gui_THR;
    ros_THR.img_thr = &IMG_THR;
    PLOTTER_THR.ros_bridg = &ros_THR;
    gui_THR.plotter_thr = &PLOTTER_THR;

    IMG_THR.moveToThread(&thr1);
    ros_THR.moveToThread(&thr2);
    PLOTTER_THR.moveToThread(&thr3);

    thr1.start();
    thr2.start();
    thr3.start();


    IMG_THR.thr_start();
    PLOTTER_THR.thr_start();

    gui_THR.setFixedSize(900,640);
    gui_THR.show();



    return a.exec();
}
