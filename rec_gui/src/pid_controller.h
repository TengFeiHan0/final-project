#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <QObject>
#include <math.h>
#include "low_pass_filter.h"
#include <QTimer>

#define BASIC_PID                   0
#define MODIFIED_PID                1

#define WITH_HYST                   2
#define WITH_OUT_HYST               3

class PID_Controller : public QObject
{
    Q_OBJECT
public:
    explicit PID_Controller(QObject *parent = 0);
    ~PID_Controller();

    low_pass_filter diff_err_filter;

    float Kp,Ki,Kd;
    float I_limit,P_limit,D_limit,Out_limit;
    float setpoint,point,out;

    float DT;
    float err_diff_F_pass;

    float P_out,D_out,I_out;


    void Run_control(bool type, int Hyst, float Hyst_val);
    void Reset_Control(void);
    float sign_num(float num);
    int get_Hyst_flag_status(void);

private:
    float error,error_integral,error_diff,error_last;
    float point_last;
    int Hyst_pass_flag;
    QTimer *MainTimer;

    uint time_counter;

private Q_SLOTS:
    void MainTimerEvent(void);

signals:

public slots:
};

#endif // PID_CONTROLLER_H
