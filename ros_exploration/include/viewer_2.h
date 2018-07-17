#ifndef VIEWER_H
#define VIEWER_H

#include <QObject>
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
#include <QTimer>
#include <QApplication>
#include <QDebug>
#include <listnerthr.h>
#include <QFile>
#include <QDataStream>
#include "seg_fcn.h"
#include <QThread>
#include "planning.h"
#include "captureviewer.h"

using namespace pcl;
using namespace std;

class listnerthr;
class Planning;
class CaptureViewer;

class Viewer : public QObject
{
    typedef pcl::PointXYZ _PointT;
    typedef pcl::PointCloud<_PointT>::ConstPtr _PointCloudT;
    typedef pcl::ModelCoefficients  _Coefficient;
    typedef pcl::PointCloud<pcl::Normal>::Ptr _NormalT;
    typedef vector<QByteArray> _Arr_vec;

    Q_OBJECT
public:
    explicit Viewer(QObject *parent = 0);
    ~Viewer();

    visualization::PCLVisualizer viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    PointT Robot_POS;
    QTimer *MainTimer;
    float loop_time;
    bool Save2File_flag,busy_flag;

    listnerthr* listner_thr;
    Planning* planning_thr;
    CaptureViewer* capture_thr;

    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter;
    vtkSmartPointer<vtkPNGWriter> writer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkAlgorithmOutput* _output;
    bool capture_flag;

    void thr_start();
    void Draw_cloud2viewer(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
    void DrawSphere(_PointT center,float R,float r,float g,float b,int shape,QByteArray pc_name);
    void DrawCylinder(_Coefficient &coeff,QByteArray name);
    void DrawPlane(_Coefficient &coeff,QByteArray name);
    void DrawNormal(_PointCloudT cloud, _NormalT normal, float m, float n, QByteArray name);
    void DrawLine(_PointT p0,_PointT p1,_PointT color,float width,QByteArray name);
    void RemoveFromViewer(_Arr_vec name_vec);
    void RemoveFromViewer(QByteArray name);
    void Capture_viewer();


    void SetShapeRenderingProperties(int property,double val1,QByteArray name);
    void SetShapeRenderingProperties(int property,double val1, double val2, double val3,QByteArray name);


    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event_,void* viewer_void);



signals:
       void thr_start_signal();
       void Draw2viewerSignal(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
       void DrawSphereSignal(_PointT center,float R,float r,float g,float b,int shape,QByteArray pc_name);
       void DrawCylinderSignal(_Coefficient coeff,QByteArray name);
       void DrawPlaneSignal(_Coefficient coeff,QByteArray name);
       void DrawNormalSignal(_PointCloudT cloud,_NormalT normal,float m,float n,QByteArray name);
       void DrawLineSignal(_PointT p0,_PointT p1,_PointT color,float width,QByteArray name);
       void RemoveFromViewerSignal(_Arr_vec name_vec);
       void RemoveFromViewerSignal(QByteArray name);

       void SetShapeRenderingPropertiesSignal(int property,double val1,QByteArray name);
       void SetShapeRenderingPropertiesSignal(int property,double val1, double val2, double val3,QByteArray name);

protected slots:
    void MainTimerEvent(void);
    void Run_THR(void);
    void Draw2viewerSlot(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
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
};

#endif // VIEWER_H
