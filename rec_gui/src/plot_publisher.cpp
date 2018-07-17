#include "plot_publisher.h"

plot_publisher::plot_publisher(QObject *parent) : QObject(parent)
{

    MainTimer = new QTimer(this);
    //    MainTimer->start(30);

    connect(this,SIGNAL(once_run()),this,SLOT(run()));
}

plot_publisher::~plot_publisher()
{

}

void plot_publisher::plot(int topic_num, float data)
{

    switch (topic_num)
    {
    case 1:
        msg_data.data = data;
        pub_plotter1.publish(msg_data);
        break;
    case 2:
        msg_data.data = data;
        pub_plotter2.publish(msg_data);
        break;
    case 3:
        msg_data.data = data;
        pub_plotter3.publish(msg_data);
        break;
    case 4:
        msg_data.data = data;
        pub_plotter4.publish(msg_data);
        break;
    }

}

void plot_publisher::thr_start()
{
    emit once_run();
}

void plot_publisher::MainTimerEvent()
{

}

void plot_publisher::run()
{
    pub_plotter1 = ros_bridg->n.advertise<std_msgs::Float32>("plotter/1",10);
    pub_plotter2 = ros_bridg->n.advertise<std_msgs::Float32>("plotter/2",10);
    pub_plotter3 = ros_bridg->n.advertise<std_msgs::Float32>("plotter/3",10);
    pub_plotter4 = ros_bridg->n.advertise<std_msgs::Float32>("plotter/4",10);
    //    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
}

