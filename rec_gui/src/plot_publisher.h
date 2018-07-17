#ifndef PLOT_PUBLISHER_H
#define PLOT_PUBLISHER_H

#include <QObject>
#include "ros/ros.h"
#include <QTimer>
#include "ros_thr.h"
#include "std_msgs/Float32.h"


class ROS_THR;

using namespace ros;
using namespace std;
class plot_publisher : public QObject
{
    Q_OBJECT
public:
    explicit plot_publisher(QObject *parent = 0);
    ~plot_publisher();

    QTimer *MainTimer;
    Publisher pub_plotter1,pub_plotter2,pub_plotter3,pub_plotter4;
    ROS_THR *ros_bridg;

    std_msgs::Float32 msg_data;

    void plot(int topic_num,float data);
    void thr_start();

signals:
    void once_run();


private Q_SLOTS:
    void MainTimerEvent(void);
    void run();
};

#endif // PLOT_PUBLISHER_H
