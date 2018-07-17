#ifndef VIEWER_H
#define VIEWER_H

#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkPNGWriter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>
#include<QVTKWidget.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QTimer>
#include <QElapsedTimer>
#include <QDebug>
#include "seg_fcn.h"
#include <QThread>
#include "planning.h"
#include "captureviewer.h"
#include <listnerthr.h>
#include <QPixmap>
#include <QRect>
#include "frontier.h"
#include <QImage>

using namespace pcl;
using namespace std;

namespace Ui {
class Viewer;
}

class listnerthr;
class Planning;
class CaptureViewer;
class Frontier;

class Viewer : public QMainWindow
{
    typedef pcl::PointXYZ _PointT;
    typedef pcl::PointCloud<_PointT>::ConstPtr _PointCloudT;
    typedef pcl::ModelCoefficients  _Coefficient;
    typedef pcl::PointCloud<pcl::Normal>::Ptr _NormalT;
    typedef vector<QByteArray> _Arr_vec;
    typedef QImage _QImage_type;

    typedef pcl::PointXYZRGBA _PointCT;
    typedef pcl::PointCloud<_PointCT> _PointCloud_CT;
    typedef _PointCloud_CT::ConstPtr _PointCloud_Ptr_CT;

    Q_OBJECT

public:
    explicit Viewer(QWidget *parent = 0);
    ~Viewer();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr VisitedPointCloud;
    std::map<std::pair<std::pair<float,float>,float>,bool> VisitedMapPoints;
    uint visited_cloud_counter;

    PointT Robot_POS,Robot_Heading;
    QTimer *MainTimer;
    float loop_time;
    bool Save2File_flag,busy_flag;
    QPixmap qvtkWidgetPixmap;
    std::vector<PointT> Robot_Axes_For_View;

    listnerthr* listner_thr;
    Planning* planning_thr;
    CaptureViewer* capture_thr;
    Frontier* frontier_thr;

    int time_sec,time_min,time_counter;

    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event_,void* viewer_void);
    void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);

    void thr_start();
    void ConstructVisitedPointCloud(_PointCloudT _cloud, PointCloudT::Ptr visited_PointCloud,
                                    std::map<std::pair<int,int>,bool > tmp_OC_Grid,std::map<std::pair<std::pair<float, float>, float>, bool> &_VisitedMapPoints);
    void ConstructVisitedPointCloud(_PointCloudT _cloud, PointCloudT::Ptr visited_PointCloud, std::map<std::pair<int, int>, bool> tmp_OC_Grid);
    void Draw_cloud2viewer(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
    void Draw_cloud2viewer(_PointCloud_Ptr_CT cloud,QByteArray pc_name,uchar points_size);
    void Draw_Visitedcloud2viewer(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
    void DrawSphere(_PointT center,float R,float r,float g,float b,int shape,QByteArray pc_name);
    void DrawCylinder(_Coefficient &coeff,QByteArray name);
    void DrawPlane(_Coefficient &coeff,QByteArray name);
    void DrawNormal(_PointCloudT cloud, _NormalT normal, float m, float n, QByteArray name);
    void DrawLine(_PointT p0,_PointT p1,_PointT color,float width,QByteArray name);
    void RemoveFromViewer(_Arr_vec name_vec);
    void RemoveFromViewer(QByteArray name);
    void SetShapeRenderingProperties(int property,double val1,QByteArray name);
    void SetShapeRenderingProperties(int property,double val1, double val2, double val3,QByteArray name);
    void draw_plane(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, ModelCoefficients coeff, char *ID);
    void add_cloud2viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,PointCloud<PointXYZ>::ConstPtr cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
    void viewer_init(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void Udpade_Raw_IMG(_QImage_type img);
    void Draw_Robot_Axes();

    cv::Mat QImage2Mat(QImage src)
    {
        QImage myImage=src;
        cv::Mat tmp(src.height(),src.width(),CV_8UC4,src.scanLine(0)); //RGB32 has 8 bits of R, 8 bits of G, 8 bits of B and 8 bits of Alpha. It's essentially RGBA.
        cv::Mat MatOut(src.height(),src.width(),CV_8UC3); //RGB32 has 8 bits of R, 8 bits of G, 8 bits of B and 8 bits of Alpha. It's essentially RGBA.

        cvtColor(tmp,MatOut,CV_RGBA2RGB);
        return MatOut;
    }



signals:
    void thr_start_signal();
    void Draw2viewerSignal(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
    void Draw2viewerSignal(_PointCloud_Ptr_CT cloud,QByteArray pc_name,uchar points_size);
    void Draw_Visitedcloud2viewerSignal(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
    void DrawSphereSignal(_PointT center,float R,float r,float g,float b,int shape,QByteArray pc_name);
    void DrawCylinderSignal(_Coefficient coeff,QByteArray name);
    void DrawPlaneSignal(_Coefficient coeff,QByteArray name);
    void DrawNormalSignal(_PointCloudT cloud,_NormalT normal,float m,float n,QByteArray name);
    void DrawLineSignal(_PointT p0,_PointT p1,_PointT color,float width,QByteArray name);
    void RemoveFromViewerSignal(_Arr_vec name_vec);
    void RemoveFromViewerSignal(QByteArray name);
    void SetShapeRenderingPropertiesSignal(int property,double val1,QByteArray name);
    void SetShapeRenderingPropertiesSignal(int property,double val1, double val2, double val3,QByteArray name);
    void Udpade_Raw_IMG_SIGNAL(_QImage_type img);

protected slots:
    void MainTimerEvent();
    void Run_THR(void);
    void Draw2viewerSlot(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
    void Draw2viewerSlot(_PointCloud_Ptr_CT cloud, QByteArray pc_name, uchar points_size);
    void Draw_Visitedcloud2viewerSlot(_PointCloudT _cloud, QByteArray pc_name, uchar r, uchar g, uchar b, uchar points_size);
    void DrawSphereSlot(_PointT center,float R,float r,float g,float b,int shape,QByteArray pc_name);
    void DrawCylinderSlot(_Coefficient coeff,QByteArray name);
    void DrawPlaneSlot(_Coefficient coeff,QByteArray name);
    void DrawNormalSignalSlot(_PointCloudT cloud,_NormalT normal,float m,float n,QByteArray name);
    void DrawLineSlot(_PointT p0,_PointT p1,_PointT color,float width,QByteArray name);
    void RemoveAllShapesSLOT(void);
    void RemoveFromViewerSLOT(_Arr_vec name_vec);
    void RemoveFromViewerSLOT(QByteArray name);
    void SetShapeRenderingPropertiesSlot(int property,double val1,QByteArray name);
    void SetShapeRenderingPropertiesSlot(int property,double val1, double val2, double val3,QByteArray name);
    void record_pb_Event(void);
    void Udpade_Raw_IMG_SLOT(_QImage_type img);
    void restart_pb_event(void);


public:
    Ui::Viewer *ui;

};

#endif // VIEWER_H
