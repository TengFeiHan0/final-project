#ifndef RRT_H
#define RRT_H

#include <QObject>
#include <QDebug>
#include "opencv2/opencv.hpp"


#define RRT_3D

using namespace std;


#ifdef RRT_3D
typedef cv::Point3f POINT;
#else
typedef cv::Point2f POINT;
#endif



class RRT : public QObject
{
    struct state
    {
        float Obs_radius;
        vector<POINT> Obs_pos;

        float Robot_radius;
        POINT start_point,goal_point;

        float Border_radius;
        pair<POINT,POINT> admissible_area;
    };

    struct _PARAM
    {
        float resulotion;
        float goal_THR;
        int Max_rrt_iteration;
        int Max_smooth_iteration;
        float step_len;

    };

    Q_OBJECT
public:
    explicit RRT(QObject *parent = 0);
    ~RRT();

    state space;
    vector<pair<POINT,int> > RRT_Tree;
    vector<POINT> RRT_PATH;
    vector<POINT> SMOOTH_PATH;
    vector<POINT> ROBOT_SETPOINT;
    vector<POINT> BREAK_POINT;
    vector<pair<POINT,POINT> > ALL_PATH;
    _PARAM RRT_Param;

    int PlanPathRRT(void);
    int SmoothPath(void);
    int Cal_Robot_SetPoints(float max_dist);
    int PathBreakPoint(float angle, float dist);
    float vec_len(POINT vec);
    float vec_len_2(POINT vec);
    vector<POINT> border_pos;
private:
    POINT Robot_pos;//,Robot_last_pos;
    pair<POINT,POINT> last_admissible_area;

    POINT make_random_point(float step_len);
    bool InCollision_Node(void);

    bool InCollision_Edge(POINT P0,POINT P1);
    void AddBorderPoints(float step);

signals:

public slots:
};

#endif // RRT_H
