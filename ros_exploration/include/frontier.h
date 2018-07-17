#ifndef FRONTIER_H
#define FRONTIER_H

#include <QObject>
#include "listnerthr.h"
#include "seg_fcn.h"
#include "viewer.h"
#include "CreatBuffer.h"
#include "planning.h"



class Viewer;
class Planning;

using namespace pcl;
using namespace std;
using namespace SEG;

class Frontier : public QObject
{
public:

    struct Space_Robot_Property
    {
        cv::Point2f X_Area,Y_Area,Z_Area;
        PointT Robot_POS,Robot_Heading;
        float Grid_dim;
        float frontierDist;
        uint DevisionNum;
        float _floor,_ceil,Level_dist,Max_Obst_height;
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

    typedef pcl::PointXYZ _PointT;
    typedef pcl::PointCloud<_PointT>::Ptr _PointCloudT;
    typedef Space_Robot_Property _Space_Robot_Property;

private:
    Q_OBJECT
public:
    explicit Frontier(QObject *parent = 0);
    ~Frontier();

    Viewer* viewer_thr;
    Planning* planning_thr;

    CreatBuffer<int> Creat_Grig_buff;
    CreatBuffer<int> Detect_Frontier_buff;
    std::vector<std::pair<PointT,bool> > Frontier_points;
    std::map<std::pair<int,int>,cv::Point3f > OC_Grid;
    std::map<std::pair<int,int>,cv::Point3f > OC_Grid_For_view;
    std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > Visited_Occ_Block;
    std::map<std::pair<int,int>,bool> NONVisited_Occ_Block;
    std::map<std::pair<int,int>,cv::Point3f > Obst_OC_Grid;
    std::vector<std::map<std::pair<int,int>,cv::Point3f > > L_Obst_OC_Grid;
    std::vector<std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > > L_Obst_Visited_Occ_Block;
    std::map<std::pair<int,int>,cv::Point3f > Obst_OC_Grid_1;
    std::map<std::pair<int,int>,cv::Point3f > Obst_OC_Grid_2;
    std::map<std::pair<int,int>,cv::Point3f > Obst_OC_Grid_3;
    std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > Obst_Visited_Occ_Block_1;
    std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > Obst_Visited_Occ_Block_2;
    std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > Obst_Visited_Occ_Block_3;
    std::map<std::pair<int,int>,cv::Point3f > Base_Cloud_Grid;
    std::map<std::pair<int,int>,cv::Point3f > Base_RawCloud_Grid;
    std::map<pair<int,int>,bool> visited_free_grid;
    std::map<std::pair<int,int>,int> inside_OC_Grid_point;
    PointCloudT::Ptr Raw_cloud;
    PointCloudT::Ptr Robot_aroud_cloud;
    PointCloudT::Ptr Wall_cloud;
    Space_Robot_Property Space_Property;
    float real_Level_num;
    std::vector<QByteArray> fr_shapes_name;

    float loop_time;
    QTimer *MainTimer;


    void newSearch(PointT robot_pos, PointT heading);
    void thr_start();
    void CreatOccupancyGrid(_PointCloudT _raw_cloud,_PointCloudT _robot_aroud_cloud,_PointCloudT _wall_cloud,_Space_Robot_Property Define_area_OutPut);
    void FrontierDetermination();
    void ObstacleDetection(_PointCloudT _cloud, float dim, PointT robot_pos, PointT heading,std::map<std::pair<int,int>,cv::Point3f > &OC_Grid,std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > &Visited_Occ_Block);
    void OccupancyGrid (_PointCloudT _cloud, float dim, PointT robot_pos, PointT heading);
    void Determin_Cloud_Base(_PointCloudT _cloud, float dim);
    void DeterminBaseOfRawCloud(_PointCloudT _cloud, float dim);
    void EdgeDetection(std::map<std::pair<int,int>,cv::Point3f > Base_Cloud_Grid,
                       std::map<std::pair<int, int>, std::pair<bool, Point2f> > &Obst_point, float dim);
    void CheckFrontierValidation(std::map<std::pair<int, int>, Point3f> Base_Cloud_Grid,
                                 std::vector<std::pair<PointT, bool> > &_Frontier_points, float dim);
    contour_str ContourExtractor(std::map<std::pair<int,int>,cv::Point3f > Cloud_Grid, Point2i offset, float scale);
    void Determin_Margin_Area(float dim, bool disp);
    void FrontierDetection(std::map<std::pair<int,int>,cv::Point3f > _Cloud_Grid, std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > Obst_grid, PointT robot_pos, float dim, std::vector<std::pair<PointT, bool> > &Frontier_points, float frontiers_dist, bool disp);
    void SliceOfCloud(_PointCloudT _cloud, float _y, float thr, _PointCloudT slicedCloud);
    void draw_occ_grid(std::map<std::pair<int,int>,cv::Point3f > cloud, PointT offset, float dim, QByteArray _name);


signals:
    void thr_start_signal();
    void CreatOccupancyGridSignal(_PointCloudT _raw_cloud,_PointCloudT _robot_aroud_cloud,_PointCloudT _wall_cloud,_Space_Robot_Property Define_area_OutPut);
    void FrontierDeterminationSignal();

protected slots:
    void Run_THR(void);
    void MainTimerEvent(void);
    void CreatOccupancyGridSlot(_PointCloudT _raw_cloud,_PointCloudT _robot_aroud_cloud,_PointCloudT _wall_cloud,_Space_Robot_Property Define_area_OutPut);
    void FrontierDeterminationSlot();
};

#endif // FRONTIER_H
