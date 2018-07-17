#include "viewer.h"
#include "../build/ui_viewer.h"

Viewer::Viewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Viewer)
{
    ui->setupUi(this);

    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    VisitedPointCloud.reset (new PointCloudT);
    cloud->points.clear();
    loop_time = 50;
    Save2File_flag = 0;
    busy_flag = 0;
    Robot_Axes_For_View.clear();
    Robot_Axes_For_View.push_back(PointT(0,0,0));
    visited_cloud_counter =0;


    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer_init(viewer);
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    viewer->resetCamera ();
    ui->qvtkWidget->update ();

    viewer->registerKeyboardCallback(&Viewer::keyboardEventOccurred,*this,(void*)&viewer);
    viewer->registerMouseCallback(&Viewer::mouseEventOccurred,*this,(void*)&viewer);

    MainTimer = new QTimer(this);
    MainTimer->start(30);

    time_sec = time_min = time_counter = 0;


    qRegisterMetaType<_PointT>("_PointT");
    qRegisterMetaType<_PointCloudT>("_PointCloudT");
    qRegisterMetaType<_PointCloud_Ptr_CT>("_PointCloud_Ptr_CT");
    qRegisterMetaType<_Coefficient>("_Coefficient");
    qRegisterMetaType<_NormalT>("_NormalT");
    qRegisterMetaType<_Arr_vec>("_Arr_vec");
    qRegisterMetaType<_QImage_type>("_QImage_type");

    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    connect(this,SIGNAL(thr_start_signal()),this,SLOT(Run_THR()));
    connect(this,SIGNAL(Draw2viewerSignal(_PointCloudT,QByteArray,uchar,uchar,uchar,uchar)),
            this,SLOT(Draw2viewerSlot(_PointCloudT,QByteArray,uchar,uchar,uchar,uchar)));
    connect(this,SIGNAL(Draw2viewerSignal(_PointCloud_Ptr_CT,QByteArray,uchar)),
            this,SLOT(Draw2viewerSlot(_PointCloud_Ptr_CT,QByteArray,uchar)));
    connect(this,SIGNAL(Draw_Visitedcloud2viewerSignal(_PointCloudT,QByteArray,uchar,uchar,uchar,uchar)),
            this,SLOT(Draw_Visitedcloud2viewerSlot(_PointCloudT,QByteArray,uchar,uchar,uchar,uchar)));
    connect(this,SIGNAL(DrawSphereSignal(_PointT,float,float,float,float,int,QByteArray)),this,SLOT(DrawSphereSlot(_PointT,float,float,float,float,int,QByteArray)));
    connect(this,SIGNAL(DrawCylinderSignal(_Coefficient,QByteArray)),this,SLOT(DrawCylinderSlot(_Coefficient,QByteArray)));
    connect(this,SIGNAL(DrawPlaneSignal(_Coefficient,QByteArray)),this,SLOT(DrawPlaneSlot(_Coefficient,QByteArray)));
    connect(this,SIGNAL(DrawNormalSignal(_PointCloudT,_NormalT,float,float,QByteArray)),this,SLOT(DrawNormalSignalSlot(_PointCloudT,_NormalT,float,float,QByteArray)));
    connect(this,SIGNAL(SetShapeRenderingPropertiesSignal(int,double,QByteArray)),this,SLOT(SetShapeRenderingPropertiesSlot(int,double,QByteArray)));
    connect(this,SIGNAL(SetShapeRenderingPropertiesSignal(int,double,double,double,QByteArray)),this,SLOT(SetShapeRenderingPropertiesSlot(int,double,double,double,QByteArray)));
    connect(this,SIGNAL(DrawLineSignal(_PointT,_PointT,_PointT,float,QByteArray)),this,SLOT(DrawLineSlot(_PointT,_PointT,_PointT,float,QByteArray)));
    connect(this,SIGNAL(RemoveFromViewerSignal(_Arr_vec)),this,SLOT(RemoveFromViewerSLOT(_Arr_vec)));
    connect(this,SIGNAL(RemoveFromViewerSignal(QByteArray)),this,SLOT(RemoveFromViewerSLOT(QByteArray)));
    connect(ui->capture_pb,SIGNAL(clicked()),this,SLOT(record_pb_Event()));
    connect(this,SIGNAL(Udpade_Raw_IMG_SIGNAL(_QImage_type)),this,SLOT(Udpade_Raw_IMG_SLOT(_QImage_type)));
    connect(ui->restart_pb,SIGNAL(clicked()),this,SLOT(restart_pb_event()));

}

Viewer::~Viewer()
{
    delete ui;
}

void Viewer::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    //    writer->SetInput(ui->qvtkWidget->cachedImage());
}

void Viewer::MainTimerEvent()
{

    QElapsedTimer ltimer;
    int passed_Time;


    //        qDebug()<<"Viewer"<<QThread::currentThreadId();
    if(listner_thr->cloud->points.size()>0)
    {
        if(Save2File_flag)
            busy_flag = 1;

        cloud->points.clear();
        cloud->points.resize(listner_thr->cloud->points.size());
        cloud->points = listner_thr->cloud->points;
        if(listner_thr->only_grab_map != 0)
            add_cloud2viewer(viewer,cloud,"points",200,55,55,2);
        Robot_POS = listner_thr->Robot_POS;
        Robot_Heading = listner_thr->Robot_Heading;

        viewer->removeShape("Robot_POS");
        if(planning_thr->Navigation_Mode == NAV_HOLD_POSITION)
            viewer->addSphere(Robot_POS,0.1,0.8,0.8,0.8,"Robot_POS");
        else
            viewer->addSphere(Robot_POS,0.1,0.1,0.2,0.5,"Robot_POS");

        if(!Robot_Axes_For_View.empty())
        {
            Robot_Axes_For_View[0] = Robot_Heading;
            Draw_Robot_Axes();
        }


        if(Save2File_flag)
        {
            Save2File_flag = 0;
            SEG::Save2File(cloud);
            busy_flag = 0;
        }

        //        ltimer.start();
        //        ConstructVisitedPointCloud(cloud,VisitedPointCloud);
        //        add_cloud2viewer(viewer,VisitedPointCloud,"points",200,55,55,2);
        //        passed_Time = ltimer.nsecsElapsed()/1000000.0;
        //        qDebug()<<"Viewer Thread  "<<passed_Time;
    }

    if(ui->color_cb->isChecked())
        capture_thr->color_correction_flag = 1;
    else
        capture_thr->color_correction_flag = 0;

    QPixmap pixmap(ui->qvtkWidget->size());
    ui->qvtkWidget->render(&pixmap);
    capture_thr->_other(pixmap.toImage());

    //    if(!capture_thr->RAW_IMG.empty() && !capture_thr->Raw_QImage.isNull())
    //    {
    //        ui->RAW_label->setPixmap(QPixmap::fromImage(capture_thr->Raw_QImage).scaled(ui->raw_frame->size()));
    //    }

    if(capture_thr->PCL_IMG_Capture_flag == 1)
    {
        if( time_counter >=(1000/loop_time))
        {
            QString time_str;
            time_sec++;
            time_min = time_sec/60;
            if(time_min<10)
                time_str = "0"+QString::number(time_min)+":";
            else
                time_str = QString::number(time_min)+":";

            if(time_sec%60 < 10)
                time_str = time_str+"0"+QString::number(time_sec%60);
            else
                time_str = time_str+QString::number(time_sec%60);
            ui->Time_label->setText(time_str);
            time_counter = 0;

        }
        else
            time_counter++;
    }
    else
        time_sec = time_min = time_counter = 0;

    ui->qvtkWidget->update();

    passed_Time = ltimer.nsecsElapsed()/1000000.0;
    //    if(passed_Time > loop_time)
    //        qDebug()<<"Viewer Thread  Overflowed ------>"<<passed_Time<<"/"<<loop_time;
}

void Viewer::ConstructVisitedPointCloud(_PointCloudT _cloud, PointCloudT::Ptr visited_PointCloud, std::map<std::pair<int, int>, bool> tmp_OC_Grid, std::map<std::pair<std::pair<float,float>,float>,bool>& _VisitedMapPoints)
{
    QElapsedTimer ltimer;

    std::pair<float,float> pair_1;
    std::pair<std::pair<float,float>,float> pair_2;
    int i,j;
    float dim_inv = 1/(frontier_thr->Space_Property.Grid_dim);

    ltimer.start();

    for(uint k=0;k<_cloud->points.size();k++)
    {
        pair_1 = make_pair(_cloud->points[k].x,_cloud->points[k].y);
        pair_2 = make_pair(pair_1,_cloud->points[k].z);
        if(_VisitedMapPoints[pair_2] == 0)
        {
            i = (floor)((_cloud->points[k].x)*dim_inv);
            j = (floor)((_cloud->points[k].z)*dim_inv);

            if(tmp_OC_Grid[make_pair(i,j)] != 0)
            {
                visited_PointCloud->points.push_back(_cloud->points[k]);
                _VisitedMapPoints[pair_2] = 1;
            }
        }
    }
    //        int passed_Time = ltimer.nsecsElapsed()/1000000.0;
    //        qDebug()<<"Viewer Thread  "<<passed_Time;

}
void Viewer::ConstructVisitedPointCloud(_PointCloudT _cloud, PointCloudT::Ptr visited_PointCloud,std::map<std::pair<int,int>,bool > tmp_OC_Grid)
{
    int i,j;
    float dim_inv = 1/(frontier_thr->Space_Property.Grid_dim);

    for(uint k=0;k<_cloud->points.size();k++)
    {
        i = (floor)((_cloud->points[k].x)*dim_inv);
        j = (floor)((_cloud->points[k].z)*dim_inv);

        if(tmp_OC_Grid[make_pair(i,j)] != 0)
        {
            visited_PointCloud->points.push_back(_cloud->points[k]);
        }
    }

}
void Viewer::Draw_cloud2viewer(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    emit Draw2viewerSignal(cloud,pc_name,r,g,b,points_size);
}
void Viewer::Draw_cloud2viewer(_PointCloud_Ptr_CT cloud, QByteArray pc_name, uchar points_size)
{
    emit Draw2viewerSignal(cloud,pc_name,points_size);
}

void Viewer::Draw_Visitedcloud2viewer(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    emit Draw_Visitedcloud2viewerSignal(cloud,pc_name,r,g,b,points_size);
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

void Viewer::SetShapeRenderingProperties(int property, double val1, QByteArray name)
{
    emit SetShapeRenderingPropertiesSignal(property,val1,name);
}

void Viewer::SetShapeRenderingProperties(int property, double val1, double val2, double val3, QByteArray name)
{
    emit SetShapeRenderingPropertiesSignal(property,val1,val2,val3,name);
}

void Viewer::draw_plane(boost::shared_ptr<visualization::PCLVisualizer> viewer, ModelCoefficients coeff, char *ID)
{
    Eigen::Vector3f i(1,0,0),j(0,1,0),k(0,0,1);
    std::vector<Eigen::Vector3f> P;
    P.resize(3);
    P[2] = Eigen::Vector3f(coeff.values[0],coeff.values[1],coeff.values[2]);

    if(P[2][0] !=0)
    {
        P[1][1] = 1;
        P[1][2] = 0;
        P[1][0] = -(P[2][1]*P[1][1]+P[2][2]*P[1][2])/P[2][0];
    }
    else if(P[3][1] !=0)
    {
        P[1][0] = 1;
        P[1][2] = 0;
        P[1][1] = -(P[2][0]*P[1][0]+P[2][2]*P[1][2])/P[2][1];
    }
    else if(P[3][2] !=0)
    {
        P[1][0] = 1;
        P[1][1] = 0;
        P[1][2] = -(P[2][0]*P[1][0]+P[2][1]*P[1][1])/P[2][2];
    }
    P[0] = P[1].cross(P[2]);

    for(int c=0;c<3;c++)
    {
        float abs_vec = sqrt(P[c].dot(P[c]));

        P[c][0] = P[c][0]/abs_vec;
        P[c][1] = P[c][1]/abs_vec;
        P[c][2] = P[c][2]/abs_vec;
    }

    Eigen::Matrix<float,3,3> R ;

    R << P[0].dot(i), P[1].dot(i), P[2].dot(i),
            P[0].dot(j), P[1].dot(j), P[2].dot(j),
            P[0].dot(k), P[1].dot(k), P[2].dot(k);

    Eigen::Quaternionf Q(R);
    Eigen::Vector3f T(coeff.values[3],coeff.values[4],coeff.values[5]);

    viewer->addCube(T,Q,coeff.values[6],coeff.values[7],0.0001,ID);

    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,ID);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,coeff.values[8]/255.,coeff.values[9]/255.,coeff.values[10]/255.,ID);

}

void Viewer::add_cloud2viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,PointCloud<PointXYZ>::ConstPtr cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    viewer->removePointCloud(pc_name.data());
    //    ui->qvtkWidget->update();
    visualization::PointCloudColorHandlerCustom<PointXYZ> points_color (cloud,r,g,b);

    viewer->addPointCloud<PointXYZ>(cloud,points_color,pc_name.data());
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,points_size,pc_name.data());
    //    ui->qvtkWidget->update();


}

void Viewer::viewer_init(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    viewer->setBackgroundColor(0.93,0.93,0.93);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
}

void Viewer::Udpade_Raw_IMG(_QImage_type img)
{
    emit Udpade_Raw_IMG_SIGNAL(img);
}

void Viewer::Draw_Robot_Axes()
{
    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize(10);

    for(int i=0;i<2;i++)
    {
        QByteArray cyl_str = QString("ROBOT_Axis_"+QString::number(i)).toLatin1();
        viewer->removeShape(cyl_str.data());
    }
    for(int i=0;i<Robot_Axes_For_View.size();i++)
    {
        QByteArray cyl_str = QString("ROBOT_Axis_"+QString::number(i)).toLatin1();
        cylinder_coeff.values[3] = Robot_Axes_For_View[i].x;
        cylinder_coeff.values[4] = Robot_Axes_For_View[i].y;
        cylinder_coeff.values[5] = Robot_Axes_For_View[i].z;
        cylinder_coeff.values[0] = Robot_POS.x;
        cylinder_coeff.values[1] = Robot_POS.y;
        cylinder_coeff.values[2] = Robot_POS.z;
        cylinder_coeff.values[6] = 0.02;
        cylinder_coeff.values[7] = (i*50+100)/255.;
        cylinder_coeff.values[8] = (i*60+150)/255.;
        cylinder_coeff.values[9] = (i*70+90)/255.;
        DrawCylinderSlot(cylinder_coeff,cyl_str);
    }

}

void Viewer::Run_THR()
{
    MainTimer->start(loop_time);
}

void Viewer::Draw2viewerSlot(_PointCloudT cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    viewer->removePointCloud(pc_name.data());
    visualization::PointCloudColorHandlerCustom<PointXYZ> points_color (cloud,r,g,b);

    viewer->addPointCloud<PointXYZ>(cloud,points_color,pc_name.data());
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,points_size,pc_name.data());
}

void Viewer::Draw2viewerSlot(_PointCloud_Ptr_CT cloud,QByteArray pc_name,uchar points_size)
{
    viewer->removePointCloud(pc_name.data());
    viewer->addPointCloud(cloud,pc_name.data());
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,points_size,pc_name.data());

}

void Viewer::Draw_Visitedcloud2viewerSlot(_PointCloudT _cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    //    QElapsedTimer ltimer;
    //    ltimer.start();
    PointCloudT::Ptr _aroud_cloud (new PointCloudT());
    visited_cloud_counter++;
    std::map<std::pair<int,int>,bool > tmp_OC_Grid;
    VisitedPointCloud->points.clear();

    {
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = frontier_thr->OC_Grid_For_view.begin();
        for(;ite != frontier_thr->OC_Grid_For_view.end();ite++)
        {
            if(ite->second != cv::Point3f(0,0,0))
                tmp_OC_Grid[ite->first] = 1;
        }
    }

    ConstructVisitedPointCloud(_cloud,_aroud_cloud,tmp_OC_Grid);
    add_cloud2viewer(viewer,_aroud_cloud,pc_name,r,g,b,points_size);

    ConstructVisitedPointCloud(cloud,VisitedPointCloud,tmp_OC_Grid);
    add_cloud2viewer(viewer,VisitedPointCloud,"visited_points",200,55,55,2);

    //    int passed_Time = ltimer.nsecsElapsed()/1000000.0;
    //    qDebug()<<"draw visited takes2  "<<passed_Time;
}

void Viewer::DrawSphereSlot(_PointT center, float R, float r, float g, float b, int shape, QByteArray pc_name)
{
    viewer->removeShape(pc_name.data());
    viewer->addSphere(center,R,r,g,b,pc_name.data());
    if(shape == 1)
    {
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,pc_name.data());
    }
    //    ui->qvtkWidget->update();
}

void Viewer::DrawCylinderSlot(_Coefficient coeff, QByteArray name)
{
    _Coefficient tmp;
    tmp.values.resize(7);
    for(int i=0;i<7;i++)
        tmp.values[i] = coeff.values[i];

    viewer->removeShape(name.data());
    viewer->addCylinder(tmp,name.data());
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,coeff.values[7],coeff.values[8],coeff.values[9],name.data());
}

void Viewer::DrawPlaneSlot(_Coefficient coeff, QByteArray name)
{
    viewer->removeShape(name.data());
    draw_plane(viewer,coeff,name.data());
}

void Viewer::DrawNormalSignalSlot(_PointCloudT cloud, _NormalT normal, float m, float n, QByteArray name)
{
    viewer->removePointCloud(name.data());
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normal,m,n,name.data());
}

void Viewer::DrawLineSlot(Viewer::_PointT p0, Viewer::_PointT p1, Viewer::_PointT color, float width, QByteArray name)
{
    viewer->removeShape(name.data());
    viewer->addLine(p0,p1,color.x,color.y,color.z,name.data());
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,width,name.data());
}

void Viewer::RemoveAllShapesSLOT()
{
    viewer->removeAllShapes();
}

void Viewer::RemoveFromViewerSLOT(Viewer::_Arr_vec name_vec)
{
    for(int i=0;i<name_vec.size();i++)
        viewer->removeShape(name_vec[i].data());
}

void Viewer::RemoveFromViewerSLOT(QByteArray name)
{
    if(!QString(name).compare("_ALL_"))
    {
        viewer->removeAllShapes();
    }
    viewer->removeShape(name.data());
}

void Viewer::SetShapeRenderingPropertiesSlot(int property,double val1,QByteArray name)
{
    viewer->setShapeRenderingProperties(property,val1,name.data());
}

void Viewer::SetShapeRenderingPropertiesSlot(int property, double val1, double val2, double val3, QByteArray name)
{
    viewer->setShapeRenderingProperties(property,val1,val2,val3,name.data());
}

void Viewer::record_pb_Event()
{
    if(ui->capture_pb->text() == "Start")
    {
        ui->capture_pb->setText("Stop");

        if(ui->SLAM_cb->isChecked())
            capture_thr->SLAM_IMG_Capture_flag = 1;
        else
            capture_thr->SLAM_IMG_Capture_flag = 0;

        if(ui->RAW_cb->isChecked())
            capture_thr->Raw_IMG_Capture_flag = 1;
        else
            capture_thr->Raw_IMG_Capture_flag = 0;
        if(ui->color_cb->isChecked())
            capture_thr->color_correction_flag = 1;
        else
            capture_thr->color_correction_flag = 0;

        capture_thr->init_cap_flag = 1;
    }
    else
    {
        ui->capture_pb->setText("Start");

        capture_thr->StopCaptureVideo();
    }
}

void Viewer::Udpade_Raw_IMG_SLOT(_QImage_type img)
{
    ui->RAW_label->setPixmap(QPixmap::fromImage(img).scaled(ui->raw_frame->size()));
}

void Viewer::restart_pb_event()
{
    viewer->removeAllShapes();
    planning_thr->Navigation_Mode = NAV_HOLD_POSITION;
}

void Viewer::thr_start()
{
    emit thr_start_signal();
}



void Viewer::keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event_,void* viewer_void)
{
    float step = 0.05;
    float theta = 2*(3.14/180);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    //    qDebug()<<event_.getKeySym().data();
    if(event_.keyDown ())
    {
        if ((event_.getKeySym () == "Escape" || event_.getKeySym () == "q"))
        {
            qApp->quit();
        }
        if ((event_.getKeySym () == "n" || event_.getKeySym () == "N"))
        {
            planning_thr->last_SetPointForPublish.second = 0;
            planning_thr->last_setpoint.second = 0;
            planning_thr->Last_Robot_POS = Robot_POS;
            planning_thr->Navigation_Mode = NAV_DECISION_MAKING_MODE;
            qDebug("Navigating...");

        }
        if ((event_.getKeySym () == "s" || event_.getKeySym () == "S"))
        {
            Save2File_flag = 1;
            qDebug("Saving...");

        }
        if ((event_.getKeySym () == "c" || event_.getKeySym () == "C"))
        {
            record_pb_Event();
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
        if ((event_.getKeySym ().compare("quoteright") == 0) )
        {
            listner_thr->Robot_POS.y -= step;
        }
        if ((event_.getKeySym ().compare("slash") == 0) )
        {
            listner_thr->Robot_POS.y += step;
        }
        if ((event_.getKeySym ().compare("period") == 0) )
        {
            PointT tmp_vec;
            tmp_vec.x = cos(theta)*listner_thr->Robot_Heading.x + sin(theta)*listner_thr->Robot_Heading.z;
            tmp_vec.y = listner_thr->Robot_Heading.y;
            tmp_vec.z = -sin(theta)*listner_thr->Robot_Heading.x + cos(theta)*listner_thr->Robot_Heading.z;
            listner_thr->Robot_Heading = tmp_vec;
            //            listner_thr->Robot_ANGLE.z += 2*(3.14/180);
            //            listner_thr->Robot_Heading = PointT(sin( listner_thr->Robot_ANGLE.z),0,cos( listner_thr->Robot_ANGLE.z));
        }

        if ((event_.getKeySym ().compare("comma") == 0) )
        {
            PointT tmp_vec;
            tmp_vec.x = cos(theta)*listner_thr->Robot_Heading.x - sin(theta)*listner_thr->Robot_Heading.z;
            tmp_vec.y = listner_thr->Robot_Heading.y;
            tmp_vec.z = sin(theta)*listner_thr->Robot_Heading.x + cos(theta)*listner_thr->Robot_Heading.z;
            listner_thr->Robot_Heading = tmp_vec;

            //            listner_thr->Robot_ANGLE.z -= 2*(3.14/180);
            //            listner_thr->Robot_Heading = PointT(sin( listner_thr->Robot_ANGLE.z),0,cos( listner_thr->Robot_ANGLE.z));
        }
        //        viewer->removeShape("Robot_POS");
        //        viewer->addSphere(Robot_POS,0.1,0.1,0.2,0.5,"Robot_POS");
    }
}

