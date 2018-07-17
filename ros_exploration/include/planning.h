#ifndef PLANNING_H
#define PLANNING_H

#include <QObject>
#include <QDebug>


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/cvfh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <QFile>
#include <QDataStream>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>
#include "pid_controller.h"

#include "listnerthr.h"
#include "seg_fcn.h"
#include "viewer.h"
#include "rrt.h"
#include "potentialfield.h"
#include "frontier.h"
#include "prm.h"

#define S_Scale                     1.0
#define ROBOT_R                     0.05
#define MIN_SEARCH_R                6.5
#define MAX_SEARCH_R                8.0
#define START_POINTS                1600
#define FIT_DIST                    0.2
#define SETPOINT_VICINITY_THR       0.4
#define HOLD_POS_TIME               0.1
#define ATTITIDE_FLUCTUATION        30
#define ROBOT_FIELD_OF_VIEW         50
#define MAX_ACCESSIBLE_EDGE_LEN     1.0
#define RANGE_OF_VIEW               5.5
#define MAX_SETPOINT_DIST           3
#define MIN_SETPOINT_DIST           2


#define NAV_DECISION_MAKING_MODE                    0
#define NAV_INIT_MODE                               1
#define NAV_SETPOINT_DETERMIN_MODE                  2
#define NAV_ROBOT_ATTITUDE_EVAL_MODE                3
#define NAV_TRAJECTORY_DETERMIN                     4
#define NAV_CORRECT_ROBOT_POS_MODE                  5
#define NAV_SEND_ROBOT_SETPOINT_MODE                6
#define NAV_PUBLISH_ROBOT_SETPOINT_MODE             7
#define NAV_INIT_AFTER_CORRECTION_MODE              8
#define NAV_SEND_YAW_SETPOINT_MODE                  9
#define NAV_LONG_PATH_TRAJECTORY_DETERMIN           10
#define NAV_HOLD_POSITION                           11

#define PUB_SEND_NAV_POS_SETPOINT                   100
#define PUB_SEND_NAV_YAW_SETPOINT                   200
#define PUB_SEND_NAV_RE_CALIBRATION                 300


#define REQUEST_FRONTIER_NON                                        0
#define REQUEST_FRONTIER_WATING_FOR_RESPONSE                        1
#define REQUEST_FRONTIER_RESPONSE_RECEIVED                          2




class listnerthr;
class Viewer;
class Frontier;

using namespace pcl;
using namespace std;
using namespace SEG;

class Planning : public QObject
{
    struct Define_area_struct
    {
        cv::Point2f X_Area,Y_Area,Z_Area;
        float Plane_fit_dist;
    };

    struct contour_str
    {
        cv::Point2i _min,_max;
        cv::Point2i offset;
        cv::vector<vector<Point> > contours;
        cv::vector<Vec4i> hierarchy;
        cv::Mat IMG;
        float scale;
    };

    Q_OBJECT
public:
    explicit Planning(QObject *parent = 0);
    ~Planning();
    listnerthr* listner_thr;
    Viewer* viewer_thr;
    Frontier* frontier_thr;


    PointT Robot_POS,Last_Robot_POS;
    PointCloudT::Ptr cloud;
    PointCloudT::Ptr robot_aroud_cloud;
    float loop_time;
    bool busy_flag;
    int SetpointBlockCounter,start_nav;
    float search_radius,search_setpoint;
    PotentialField SetpointPlanning,RobotpointPlanning;
    RRT rrt_planning;
    std::vector<QByteArray> shapes_name;
    std::vector<QByteArray> shapes_name_2;
    std::vector<QByteArray> shapes_name_3;
    int Navigation_Mode;
    vector<PointT> Robot_SetPoints;
    PointT SetPointForPublish;
    std::pair<PointT,bool> last_SetPointForPublish;
    PointT EndPathSetPointForPublish;
    PointT SetPointForNAV;
    vector<POINT> VirtualRobotPOS;
    int nav_counter;
    Define_area_struct Define_area_OutPut;
    int potential_field_iteration;
    bool motion_flag;
    PointT Robot_Heading;
    cv::Point2f Floor_Ceil_Level;
    std::pair<PointT,bool> backwardSetPoint;
    std::pair<PointT,bool> last_setpoint;
    float THR_For_generate_next_frontier;

    uint Request_Frontier_Flag;
    std::vector<std::pair<pair<int, PointT>,bool> >candidate_frontier;
    std::pair<std::vector<PointT>,PointCloudT::Ptr> Observed_cloud;
    PointCloudT::Ptr PRM_PointCloud;

    PRM PRM_Planning;

    QElapsedTimer ltimer;





    //............................
    std::pair<PointT,cv::Point2f> candidate_setpoint;
    std::vector<std::pair<PointT,cv::Point2f> > candidate_points;
    float Yaw_setpoint;
    PointT Robot_Heading_Setpoint;
    std::vector<PointT> accessiblePoints;
    bool Try_Other_Condidates;
    //............................................


    uint Frontier_thr_run_counter;

    QTimer *MainTimer;

    //Publishers...........
    ros::NodeHandle nh;
    ros::Publisher Navigation_OutPut_Pub;

    void thr_start();
    void select_aroud_points();
    void Navigation_Fcn(PointCloudT::Ptr _cloud,PointCloudT::Ptr _cloud2);
    void Initiate_Plannings(PointCloudT::Ptr _cloud, PointCloudT::Ptr _cloud2);
    void DrawPlanningOutPut();
    void CorrectRobotPos(void);
    void pub_setpint(PointT _point, float _type);
    void compute_PA_cloud(PointCloudT::Ptr _cloud);
    void find_farest_point(PointCloudT::Ptr _cloud, PointT robot, PointT heading, float thr, bool disp);
    void select_best_frontier(std::vector<std::pair<PointT, bool> > &Frontier_points, PointT robot_pos, PointT heading, std::vector<std::pair<pair<int, PointT>,bool> > &out_point);
    bool check_setpoint_validation(PointCloudT::Ptr _cloud, std::vector<PointT> _accessiblePoits, PointT new_point, float max_dist, bool disp);
    void store_corrected_map_point(PointCloudT::Ptr _cloud, std::pair<std::vector<PointT>, PointCloudT::Ptr> &out_cloud, PointT robot_pos, float search_R);
    bool findLongPath(PointCloudT::Ptr _cloud, PointT robot_pos, std::vector<PointT> goal_pos, std::vector<PointT> &_accessiblePoints, vector<PointT> &Out_setpoints, bool disp);
    bool CheckInsidePoint(POINT A0,POINT A1,PointT P_test);
    void BreakDownFarSetpoints(std::vector<PointT> &Setpoints);
    void buildAccessiblePoints(std::vector<PointT> Robot_SetPoints,std::vector<PointT>& accessiblePoints);
    void updateAccessiblePoints(PointT _robot_pos,std::vector<PointT>& accessiblePoints);
    std::pair<PointT,bool> FindMinRotationPoint(std::vector<PointT> Setpoints);
    Define_area_struct Define_area(int disp, bool print, int cluster_num,PointCloud<PointXYZ>::Ptr _cloud,PointCloud<PointXYZ>::Ptr Wall_cloud, PointT _dim, uint divide_method, uint method, uint correction_method, float modify);

    void request_response_frontier(void);


private:
    bool farest_fcn_flag;


signals:
       void thr_start_signal();
       void request_response_frontier_signal(void);

protected slots:
    void MainTimerEvent(void);
    void Run_THR(void);
    void request_response_frontier_slot(void);
};

#endif // PLANNING_H
