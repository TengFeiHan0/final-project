#ifndef PRM_H
#define PRM_H

#include <QObject>
#include <QDebug>
#include "opencv2/opencv.hpp"
#include "seg_fcn.h"
#include "rrt.h"

#define PRM_3D

using namespace std;
using namespace SEG;
class PRM : public QObject
{
    typedef cv::Point3f POINT;

    struct state
    {
        float Obs_radius;
        vector<POINT> Obs_pos;

        float Robot_radius;
        POINT start_point;
        std::vector<POINT> goal_point;

        float Border_radius;
        pair<POINT,POINT> admissible_area;

        cv::Point2f Global_Floor_Ceil_Level;

        vector<POINT> PRM_Points;
    };

    struct _PARAM
    {
        float resulotion;
        uint K_nearest;
        float edge_min_dist;
        float Min_Graph_Edge_len;
    };

    Q_OBJECT
public:
    explicit PRM(QObject *parent = 0);
    ~PRM();

    state space;
    vector<POINT> border_pos;
    _PARAM PRM_Param;
    std::map<int,std::vector<uint> > PRM_Graph;
    uint start_point_idx;
    std::vector<int> goal_point_idx;
    std::set<int> OpenSet;
    std::set<int> CloseSet;
    std::map<int,int> CameFrom;
    std::map<int,float> G_Score;
    std::map<int,float> F_Score;
    std::vector<POINT> OptimizePath;
    RRT rrt_planning;
    vector<POINT> Inserted_PRM_Points;
    POINT nearest_goal_point;
    POINT last_point;

    bool GetOptimizePath();
    bool Run_A_Star(void);
    void RoadMapConstruction(void);
    bool InCollision_Node(POINT Robot_pos);
    bool InCollision_Edge(POINT P0,POINT P1);
    float vec_len(POINT vec);
    float vec_len_2(POINT vec);
    float H_dist(POINT P0,POINT P1);
    float H_dist(PRM::POINT P0, std::vector<PRM::POINT> P1);
    int FindLowestFScore();
    void ReconstructPath(std::map<int,int> _CameFrom, int _current);
    bool goal_PRM_points_collision_correction(POINT _goal_point);
    void search_Area(POINT curr_POS, float dist, std::vector<POINT> &selected_cloud);


signals:

public slots:
};

#endif // PRM_H
