#include "viewer.h"

Viewer::Viewer(QObject *parent) : QObject(parent)
{
    viewer.setWindowName("3D Viewer");
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

    SEG::viewer_init(viewer);
    viewer.registerKeyboardCallback(&Viewer::keyboardEventOccurred,*this,(void*)&viewer);
    cloud->points.clear();
    loop_time = 30;
    Save2File_flag = 0;
    busy_flag = 0;

    MainTimer = new QTimer();

    qRegisterMetaType<_PointT>("_PointT");
    qRegisterMetaType<_PointCloudT>("_PointCloudT");
    qRegisterMetaType<_Coefficient>("_Coefficient");
    qRegisterMetaType<_NormalT>("_NormalT");
    qRegisterMetaType<_Arr_vec>("_Arr_vec");

    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    connect(this,SIGNAL(thr_start_signal()),this,SLOT(Run_THR()));
    connect(this,SIGNAL(Draw2viewerSignal(_PointCloudT,QByteArray,uchar,uchar,uchar,uchar)),
            this,SLOT(Draw2viewerSlot(_PointCloudT,QByteArray,uchar,uchar,uchar,uchar)));
    connect(this,SIGNAL(DrawSphereSignal(_PointT,float,float,float,float,int,QByteArray)),this,SLOT(DrawSphereSlot(_PointT,float,float,float,float,int,QByteArray)));
    connect(this,SIGNAL(DrawCylinderSignal(_Coefficient,QByteArray)),this,SLOT(DrawCylinderSlot(_Coefficient,QByteArray)));
    connect(this,SIGNAL(DrawPlaneSignal(_Coefficient,QByteArray)),this,SLOT(DrawPlaneSlot(_Coefficient,QByteArray)));
    connect(this,SIGNAL(DrawNormalSignal(_PointCloudT,_NormalT,float,float,QByteArray)),this,SLOT(DrawNormalSignalSlot(_PointCloudT,_NormalT,float,float,QByteArray)));
    connect(this,SIGNAL(SetShapeRenderingPropertiesSignal(int,double,QByteArray)),this,SLOT(SetShapeRenderingPropertiesSlot(int,double,QByteArray)));
    connect(this,SIGNAL(SetShapeRenderingPropertiesSignal(int,double,double,double,QByteArray)),this,SLOT(SetShapeRenderingPropertiesSlot(int,double,double,double,QByteArray)));
    connect(this,SIGNAL(DrawLineSignal(_PointT,_PointT,_PointT,float,QByteArray)),this,SLOT(DrawLineSlot(_PointT,_PointT,_PointT,float,QByteArray)));
    connect(this,SIGNAL(RemoveFromViewerSignal(_Arr_vec)),this,SLOT(RemoveFromViewerSLOT(_Arr_vec)));
    connect(this,SIGNAL(RemoveFromViewerSignal(QByteArray)),this,SLOT(RemoveFromViewerSLOT(QByteArray)));

    windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetWriteToMemory(1);
    capture_flag = 0;

}

Viewer::~Viewer()
{

}

void Viewer::MainTimerEvent()
{
    //        qDebug()<<"Viewer"<<QThread::currentThreadId();
    if(listner_thr->cloud->points.size()>0)
    {
        if(Save2File_flag)
            busy_flag = 1;

        cloud->points.clear();
        cloud->points.resize(listner_thr->cloud->points.size());
        cloud->points = listner_thr->cloud->points;
        add_cloud2viewer(viewer,cloud,"points",200,55,55,2);
        Robot_POS = listner_thr->Robot_POS;
        viewer.removeShape("Robot_POS");
        if(planning_thr->Navigation_Mode == NAV_HOLD_POSITION)
            viewer.addSphere(Robot_POS,0.1,0.8,0.8,0.8,"Robot_POS");
        else
            viewer.addSphere(Robot_POS,0.1,0.1,0.2,0.5,"Robot_POS");

        if(Save2File_flag)
        {
            Save2File_flag = 0;
            Save2File(cloud);
            busy_flag = 0;
        }
    }



    viewer.spinOnce();


    Capture_viewer();

    if(viewer.wasStopped())
        qApp->quit();
}

void Viewer::Draw_cloud2viewer(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    emit Draw2viewerSignal(cloud,pc_name,r,g,b,points_size);
}

void Viewer::DrawSphere(_PointT center, float R, float r, float g, float b, int shape, QByteArray pc_name)
{
    emit DrawSphereSignal(center,R,r,g,b,shape,pc_name);
}

void Viewer::DrawCylinder(_Coefficient &coeff,QByteArray name)
{
    emit DrawCylinderSignal(coeff,name);
}

void Viewer::DrawPlane(_Coefficient &coeff, QByteArray name)
{
    emit DrawPlaneSignal(coeff,name);
}

void Viewer::DrawNormal(_PointCloudT cloud,_NormalT normal,float m,float n,QByteArray name)
{
    emit DrawNormalSignal(cloud,normal,m,n,name);
}

void Viewer::DrawLine(_PointT p0,_PointT p1,_PointT color,float width,QByteArray name)
{
    emit DrawLineSignal(p0,p1,color,width,name);
}

void Viewer::RemoveFromViewer(Viewer::_Arr_vec name_vec)
{
    emit RemoveFromViewerSignal(name_vec);
}

void Viewer::RemoveFromViewer(QByteArray name)
{
    emit RemoveFromViewerSignal(name);
}

void Viewer::Capture_viewer()
{
    if(capture_flag)
    {
        renderWindow  = viewer.getRenderWindow();
        renderWindow->Render();
        windowToImageFilter->SetInput(0);
        windowToImageFilter->SetInput(renderWindow);
        //    windowToImageFilter->SetMagnification(3);
        windowToImageFilter->ReadFrontBufferOff();
        windowToImageFilter->Update();
        _output = windowToImageFilter->GetOutputPort();
        capture_thr->_other(_output);
    }
}

void Viewer::SetShapeRenderingProperties(int property, double val1, QByteArray name)
{
    emit SetShapeRenderingPropertiesSignal(property,val1,name);
}

void Viewer::SetShapeRenderingProperties(int property, double val1, double val2, double val3, QByteArray name)
{
    emit SetShapeRenderingPropertiesSignal(property,val1,val2,val3,name);
}

void Viewer::Run_THR()
{
    MainTimer->start(loop_time);
}

void Viewer::Draw2viewerSlot(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    viewer.removePointCloud(pc_name.data());
    visualization::PointCloudColorHandlerCustom<PointXYZ> points_color (cloud,r,g,b);

    viewer.addPointCloud<PointXYZ>(cloud,points_color,pc_name.data());
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,points_size,pc_name.data());
}

void Viewer::DrawSphereSlot(_PointT center, float R, float r, float g, float b, int shape, QByteArray pc_name)
{
    viewer.removeShape(pc_name.data());
    viewer.addSphere(center,R,r,g,b,pc_name.data());
    if(shape == 1)
    {
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,pc_name.data());
    }
}

void Viewer::DrawCylinderSlot(_Coefficient coeff, QByteArray name)
{
    _Coefficient tmp;
    tmp.values.resize(7);
    for(int i=0;i<7;i++)
        tmp.values[i] = coeff.values[i];

    viewer.removeShape(name.data());
    viewer.addCylinder(tmp,name.data());
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,coeff.values[7],coeff.values[8],coeff.values[9],name.data());
}

void Viewer::DrawPlaneSlot(_Coefficient coeff, QByteArray name)
{
    viewer.removeShape(name.data());
    draw_plane(viewer,coeff,name.data());
}

void Viewer::DrawNormalSignalSlot(_PointCloudT cloud, _NormalT normal, float m, float n, QByteArray name)
{
    viewer.removePointCloud(name.data());
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normal,m,n,name.data());
}

void Viewer::DrawLineSlot(Viewer::_PointT p0, Viewer::_PointT p1, Viewer::_PointT color, float width, QByteArray name)
{
    viewer.removeShape(name.data());
    viewer.addLine(p0,p1,color.x,color.y,color.z,name.data());
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,width,name.data());
}

void Viewer::RemoveAllShapesSLOT()
{
    viewer.removeAllShapes();
}

void Viewer::RemoveFromViewerSLOT(Viewer::_Arr_vec name_vec)
{
    for(int i=0;i<name_vec.size();i++)
        viewer.removeShape(name_vec[i].data());
}

void Viewer::RemoveFromViewerSLOT(QByteArray name)
{
    if(!QString(name).compare("_ALL_"))
    {
        qDebug("ALLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL");
        viewer.removeAllShapes();
    }
    viewer.removeShape(name.data());
}

void Viewer::SetShapeRenderingPropertiesSlot(int property,double val1,QByteArray name)
{
    viewer.setShapeRenderingProperties(property,val1,name.data());
}

void Viewer::SetShapeRenderingPropertiesSlot(int property, double val1, double val2, double val3, QByteArray name)
{
    viewer.setShapeRenderingProperties(property,val1,val2,val3,name.data());
}

void Viewer::thr_start()
{
    emit thr_start_signal();
}

void Viewer::keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event_,void* viewer_void)
{
    float step = 0.05;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    //        qDebug()<<event_.getKeySym().data();
    if(event_.keyDown ())
    {
        if ((event_.getKeySym () == "Escape" || event_.getKeySym () == "q"))
        {
            qApp->quit();
        }
        if ((event_.getKeySym () == "n" || event_.getKeySym () == "N"))
        {
            planning_thr->Navigation_Mode = NAV_INIT_MODE;
            qDebug("Navigating...");

        }
        if ((event_.getKeySym () == "s" || event_.getKeySym () == "S"))
        {
            Save2File_flag = 1;
            qDebug("Saving...");

        }
        if ((event_.getKeySym () == "c" || event_.getKeySym () == "C"))
        {
            capture_thr->capture_flag = 1;
            capture_flag = 1;
            qDebug("Capturing the screen...........");

        }
        if ((event_.getKeySym ().compare("Up") == 0) )
        {
            listner_thr->Robot_POS.z -= step;
        }
        if ((event_.getKeySym ().compare("Down") == 0) )
        {
            listner_thr->Robot_POS.z += step;
        }
        if ((event_.getKeySym ().compare("Left") == 0) )
        {
            listner_thr->Robot_POS.x -= step;
        }
        if ((event_.getKeySym ().compare("Right") == 0) )
        {
            listner_thr->Robot_POS.x += step;
        }
        if ((event_.getKeySym ().compare("period") == 0) )
        {
            listner_thr->Robot_POS.y -= step;
        }
        if ((event_.getKeySym ().compare("comma") == 0) )
        {
            listner_thr->Robot_POS.y += step;
        }
        //        viewer.removeShape("Robot_POS");
        //        viewer.addSphere(Robot_POS,0.1,0.1,0.2,0.5,"Robot_POS");
    }
}




