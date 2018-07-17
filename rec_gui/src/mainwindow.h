#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include <QThread>
#include "listnerthr.h"
#include "ros_thr.h"
#include "low_pass_filter.h"

#include <QLabel>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QCheckBox>
#include <QRadioButton>
#include <QImage>
#include "HelperFunctions.h"
#include "plot_publisher.h"
#include <QComboBox>


#define CONTROL_NONE                0
#define CONTROL_KB                  1

#define CONTROL_HEIGHT              2
#define CONTROL_SLAM_CALIB          3
#define CONTROL_SLAM_RE_CALIB       4
#define CONTROL_POS                 5
#define CONTROL_Auto                6




using namespace ros;
using namespace std;

class ROS_THR;
class listnerthr;
struct ControlCommand;
class plot_publisher;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QTimer *MainTimer;
    NodeHandle n;
    Subscriber sub;


    listnerthr *img_thr;
    ROS_THR *ros_bridge;
    plot_publisher *plotter_thr;

    bool flag;
    bool Height_Control_flag;
    int currentControlSource;

    float Height_setpoint;
    float X_setpoint,Y_setpoint,Z_setpoint;
    float YAW_setpoint;

    //Yaw Angular velocity...........
    float YAW_Vel,last_YAW_Vel,YAW_last;
    float predicted_YAW_Vel,predicted_YAW_last;
    float Height_Vel,last_Height_Vel,Height_last;
    low_pass_filter YAW_Vel_Filter;
    low_pass_filter Height_Vel_Filter;

    double sensGaz, sensYaw, sensRP;
    Mat frame;
    QImage frame_img;

    bool ORB_IMG_flag;


    //Kalman.............
    cv::KalmanFilter *KF;
    Mat KF_state;
    Mat KF_Measurment,KF_Control;

    cv::KalmanFilter *KF_YAW;
    Mat KF_YAW_state;
    Mat KF_YAW_Measurment,KF_YAW_Control;

    cv::KalmanFilter *KF_Obs;
    Mat KF_Obs_state;
    Mat KF_Obs_Measurment,KF_Obs_Control;


    void run(void);
    void chatterCallback(const std_msgs::String::ConstPtr& msg);

    ControlCommand calcKBControl();
    void setControlSource(int type);

    QImage Mat2QImage(const cv::Mat &src);

private:
    Ui::MainWindow *ui;

private Q_SLOTS:
    void MainTimerEvent(void);
    void Land_Pb_Event(void);
    void Takeoff_Pb_Event(void);
    void CheckBoxes_Event(void);
    void FlatTrim_event(void);
    void ToggleCam_event(void);
    void ORB_Event(void);
    void slam_calib_Event(void);
    void pos_setpoint_pb_event(void);
    void Auto_Control_Event(void);

protected:
    // keyboard control.... this is the only way i managed to do this...
    void keyPressEvent( QKeyEvent * key);
    void keyReleaseEvent( QKeyEvent * key);
    int mapKey(int k);
    bool isPressed[8];	//{j k l i u o q a}
    unsigned int lastRepeat[8];
};

#endif // MAINWINDOW_H
