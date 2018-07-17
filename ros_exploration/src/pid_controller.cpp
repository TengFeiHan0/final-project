#include "pid_controller.h"

PID_Controller::PID_Controller(QObject *parent) : QObject(parent)
{
    P_out = I_out = D_out = out = 0;
    setpoint = point = point_last = 0;
    error = error_diff = error_integral = error_last = 0;
    Kp = Ki = Kd = 0;
    err_diff_F_pass = 0;
    Hyst_pass_flag = 2;

    time_counter = 0;
    //    MainTimer = new QTimer(this);
    //    MainTimer->start(100);

    //    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
}

PID_Controller::~PID_Controller()
{

}

void PID_Controller::Run_control(bool type, int Hyst, float Hyst_val)
{

    if( ((Kp!=0) ||  (Ki!=0) || (Kd!=0)) && DT!=0 )
    {
        if(diff_err_filter.DT == 0)
        {
            diff_err_filter.DT = DT;
            diff_err_filter.F_pass = err_diff_F_pass;
        }
        error = setpoint - point;
        error_integral = error_integral + error*DT;

        if(type == BASIC_PID)
        {
            error_diff = (error - error_last)/DT;

        }
        else if(type == MODIFIED_PID)
        {
            error_diff = -(point-point_last)/DT;
        }

        if(diff_err_filter.filter(error_diff))
            error_diff = diff_err_filter.data;


        if( Hyst == WITH_HYST)
        {
            if(fabs(error) >= (1.5*Hyst_val))
            {
                Hyst_pass_flag = 2;
            }
            if(fabs(error) <= Hyst_val && Hyst_pass_flag == 2)
            {
                Hyst_pass_flag = 1;
                time_counter = 0;
            }
            if(Hyst_pass_flag == 1 && time_counter>(int)(0.6/DT))
            {
                Hyst_pass_flag = 0;
            }
        }

        if(Hyst_pass_flag != 0)
            time_counter++;

        P_out = error*Kp;
        I_out = error_integral*Ki;
        D_out = error_diff*Kd;

        P_out = (fabs(P_out) <= P_limit)?(P_out):(sign_num(P_out)*P_limit);
        I_out = (fabs(I_out) <= I_limit)?(I_out):(sign_num(I_out)*I_limit);
        D_out = (fabs(D_out) <= D_limit)?(D_out):(sign_num(D_out)*D_limit);

        out = P_out + I_out + D_out;
        out = (fabs(out) <= Out_limit)?(out):(sign_num(out)*Out_limit);

        if(Hyst_pass_flag == 0 && Hyst == WITH_HYST )
        {
            out = 0.000000;
            error_integral = 0.000000;
        }
//        qDebug()<< "hyst"<<Hyst_pass_flag << time_counter;
        error_last = error;
        point_last = point;
    }
}

void PID_Controller::Reset_Control()
{
    P_out = I_out = D_out = out = 0;
    error = error_diff = error_integral = error_last = 0;
}

float PID_Controller::sign_num(float num)
{
    if(num>0)
        return 1;
    else if(num < 0)
        return -1;
    else
        return 0;
}

int PID_Controller::get_Hyst_flag_status()
{
    return Hyst_pass_flag;
}

void PID_Controller::MainTimerEvent()
{
    time_counter++;
    qDebug("timer");
}

