#include "frontier.h"

Frontier::Frontier(QObject *parent) : QObject(parent)
{
    loop_time = 30;
    Creat_Grig_buff.initBuffer(100);
    Detect_Frontier_buff.initBuffer(100);
    MainTimer = new QTimer(this);
    Raw_cloud = PointCloudT::Ptr (new PointCloudT);
    Robot_aroud_cloud = PointCloudT::Ptr (new PointCloudT);
    Wall_cloud = PointCloudT::Ptr (new PointCloudT);


    qRegisterMetaType<_PointT>("_PointT");
    qRegisterMetaType<_PointCloudT>("_PointCloudT");
    qRegisterMetaType<_Space_Robot_Property>("_Space_Robot_Property");

    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    connect(this,SIGNAL(thr_start_signal()),this,SLOT(Run_THR()));
    connect(this,SIGNAL(CreatOccupancyGridSignal(_PointCloudT,_PointCloudT,_PointCloudT,_Space_Robot_Property)),this,
            SLOT(CreatOccupancyGridSlot(_PointCloudT,_PointCloudT,_PointCloudT,_Space_Robot_Property)));
    connect(this,SIGNAL(FrontierDeterminationSignal()),this,SLOT(FrontierDeterminationSlot()));

}

Frontier::~Frontier()
{

}



void Frontier::thr_start()
{
    emit thr_start_signal();
}

void Frontier::FrontierDetermination()
{
    emit FrontierDeterminationSignal();
}

void Frontier::MainTimerEvent()
{
    int int_tmp;
    //    QElapsedTimer ltimer;
    //    int passed_Time;


    if(Creat_Grig_buff.ReadFromBufferIn(int_tmp))
    {
        //        ltimer.start();
        float _mid;
        float cl_len;
        float dim = Space_Property.Grid_dim;

        //        newSearch(Space_Property.Robot_POS,Space_Property.Robot_Heading);

        if(L_Obst_OC_Grid.size() < Space_Property.DevisionNum)
        {
            L_Obst_OC_Grid.resize(Space_Property.DevisionNum);
            L_Obst_Visited_Occ_Block.resize(Space_Property.DevisionNum);
        }

        {
            real_Level_num  = 0;
            float dist = fabs(Space_Property._ceil-Space_Property._floor) + 0.2;
            float m = ceil(dist/Space_Property.Level_dist);
            cl_len = 0.2;
            for(uint j=1;j<m;j++)
            {
                PointCloudT::Ptr L_Sliced_Cloud (new PointCloudT());
                if((j-1) < Space_Property.DevisionNum)
                {
                    real_Level_num = real_Level_num+1;
                    _mid = ( 1-((float)j/m) )*Space_Property._floor + ((float)j/m)*Space_Property._ceil;

                    SliceOfCloud(Wall_cloud,_mid,cl_len,L_Sliced_Cloud);
                    ObstacleDetection (L_Sliced_Cloud,dim,Space_Property.Robot_POS,Space_Property.Robot_Heading,L_Obst_OC_Grid[j-1],L_Obst_Visited_Occ_Block[j-1]);
//                    float g = j*60;
//                    QByteArray sph_str = QString("sliced_POINTS_"+QString::number(j)).toLatin1();
//                    viewer_thr->Draw_cloud2viewer(L_Sliced_Cloud,sph_str,0,g,220,6);
                }
            }
        }

        {
            cv::Point3f cv_tmp_point;
            float devision_inv = 1/((float)Space_Property.DevisionNum);
            float thr = (floor((Space_Property.Max_Obst_height/Space_Property.Level_dist)) * (5.0/6.0))/(real_Level_num);
            std::map<std::pair<int,int>,cv::Point3f >::iterator ite = OC_Grid.begin();
            for(;ite != OC_Grid.end();ite++)
            {
                cv_tmp_point = cv::Point3f(0,0,0);
                for(uint i=0;i<Space_Property.DevisionNum;i++)
                {
                    cv_tmp_point = cv_tmp_point + L_Obst_OC_Grid[i][ite->first];
                }
                Obst_OC_Grid[ite->first] = cv_tmp_point*devision_inv;

                if(Obst_OC_Grid[ite->first].x <= thr)
                    Obst_OC_Grid[ite->first] = cv::Point3f(0,0,0);
            }
            //            draw_occ_grid(Obst_OC_Grid,PointT(0,-0.5,0),dim,"OBST_OC_GRID");
        }



        {
            PointCloudT::Ptr Sliced_Cloud (new PointCloudT());
            _mid = (Space_Property.Y_Area.x + Space_Property.Y_Area.y )/2 + 0.2;
            cl_len = (Space_Property.Y_Area.y - Space_Property.Y_Area.x)/2 ;
            SliceOfCloud(Wall_cloud,_mid,cl_len,Sliced_Cloud);
            //            viewer_thr->Draw_cloud2viewer(Sliced_Cloud,"sliced_POINTS",0,0,200,6);
            OccupancyGrid (Sliced_Cloud,dim,Space_Property.Robot_POS,Space_Property.Robot_Heading);
            draw_occ_grid(OC_Grid_For_view,PointT(0,1,0),dim,"OC_GRID");
        }


        {
            PointCloudT::Ptr Sliced_RawCloud (new PointCloudT());
            _mid = (Space_Property.Y_Area.x + Space_Property.Y_Area.y )/2;
            cl_len = (Space_Property.Y_Area.y - Space_Property.Y_Area.x)/2 +3;
            SliceOfCloud(Raw_cloud,_mid,cl_len,Sliced_RawCloud);
            DeterminBaseOfRawCloud(Sliced_RawCloud,dim);

        }
        {
            PointCloudT::Ptr Sliced_Cloud (new PointCloudT());
            _mid = (Space_Property.Y_Area.x + Space_Property.Y_Area.y )/2;
            cl_len = (Space_Property.Y_Area.y - Space_Property.Y_Area.x)/2 +0.1;
            SliceOfCloud(Robot_aroud_cloud,_mid,cl_len,Sliced_Cloud);
            Determin_Cloud_Base(Sliced_Cloud,dim);
        }

        Determin_Margin_Area(dim,0);

        //        passed_Time = ltimer.nsecsElapsed()/1000000.0;
        //        draw_occ_grid(Base_RawCloud_Grid,PointT(0,-0.8,0),dim,"Base_RawCL_GRID");
        //        qDebug()<<"Frontier Thread "<<passed_Time;
    }

    int_tmp =0;
    if(Detect_Frontier_buff.ReadFromBufferIn(int_tmp))
    {
        FrontierDetection(OC_Grid,Visited_Occ_Block,Space_Property.Robot_POS,Space_Property.Grid_dim,Frontier_points,Space_Property.frontierDist,0);
        //        CheckFrontierValidation(Base_Cloud_Grid,Frontier_points,Space_Property.Grid_dim);
        planning_thr->request_response_frontier();
    }


    //    if(passed_Time > loop_time)
    //        qDebug()<<"Frontier Thread  Overflowed ------>"<<passed_Time<<"/"<<loop_time;

}

void Frontier::CreatOccupancyGridSlot(_PointCloudT _raw_cloud,_PointCloudT _robot_aroud_cloud, _PointCloudT _wall_cloud, _Space_Robot_Property Define_area_OutPut)
{
    Raw_cloud->points.clear();
    Raw_cloud->points.resize(_raw_cloud->points.size());
    Raw_cloud->points = _raw_cloud->points;

    Robot_aroud_cloud->points.clear();
    Robot_aroud_cloud->points.resize(_robot_aroud_cloud->points.size());
    Robot_aroud_cloud->points = _robot_aroud_cloud->points;

    Wall_cloud->points.clear();
    Wall_cloud->points.resize(_wall_cloud->points.size());
    Wall_cloud->points = _wall_cloud->points;

    Space_Property = Define_area_OutPut;

    Creat_Grig_buff.Write2BufferIn(10);

}

void Frontier::FrontierDeterminationSlot()
{
    if(Space_Property.frontierDist == 0)
    {
        qDebug()<<" The Distance of two frontiers considered to zero, Please correct this";
    }
    else
        Detect_Frontier_buff.Write2BufferIn(10);
}

void Frontier::CreatOccupancyGrid(_PointCloudT _raw_cloud,PointCloud<PointXYZ>::Ptr _robot_aroud_cloud, PointCloud<PointXYZ>::Ptr _wall_cloud, _Space_Robot_Property Define_area_OutPut)
{
    emit CreatOccupancyGridSignal(_raw_cloud,_robot_aroud_cloud,_wall_cloud,Define_area_OutPut);
}

void Frontier::ObstacleDetection(_PointCloudT _cloud, float dim, PointT robot_pos, PointT heading,std::map<std::pair<int,int>,cv::Point3f > &OC_Grid,std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> > &Visited_Occ_Block)
{
    int i,j;
    std::map<std::pair<int,int>,PointCloud<PointXYZ> > cloud_vec;
    cv::Point2i _min(1000000,1000000),_max(-1000000,-1000000);
    for(int c=0;c<_cloud->points.size();c++)
    {
        i = (floor)((_cloud->points[c].x)/dim);
        j = (floor)((_cloud->points[c].z)/dim);

        _min.x = (_min.x > i)?(i):(_min.x);
        _min.y = (_min.y > j)?(j):(_min.y);

        _max.x = (_max.x < i)?(i):(_max.x);
        _max.y = (_max.y < j)?(j):(_max.y);

        cloud_vec[make_pair(i,j)].points.push_back(_cloud->points[c]);
    }

    _min.x -= 2;_min.y -= 2;
    _max.x += 2;_max.y += 2;

    std::map<std::pair<int,int>,cv::Point3f > L_OC_Grid;
    std::map<std::pair<int,int>,bool > Visited_Grid_Block;
    for(int k=_min.x;k<=_max.x;k++)
    {
        for(int q=_min.y;q<=_max.y;q++)
        {
//            if(OC_Grid[make_pair(k,q)] == cv::Point3f(0,0,0))
//                OC_Grid[make_pair(k,q)] = cv::Point3f(0,0,0);

//            if(L_OC_Grid[make_pair(k,q)] == cv::Point3f(0,0,0))
            L_OC_Grid[make_pair(k,q)] = cv::Point3f(0,0,0);
        }
    }
    //    EdgeDetection(OC_Grid,robot_pos,dim);
    {
        std::map<std::pair<int,int>,PointCloud<PointXYZ> >::iterator ite = cloud_vec.begin();
        float _size;
        for(;ite != cloud_vec.end();ite++)
        {
            _size = (float)(ite->second.points.size());
            //            if(_size > 5)
            //            {
            //                L_OC_Grid[ite->first] = cv::Point3f(1.0,0,0);
            //            }
            if(_size > 2)
            {
                float r = (_size > 5)?(1):((_size/6.0));
                L_OC_Grid[ite->first].x = r;
            }
        }
    }

    float max_dist = RANGE_OF_VIEW;
    float resolution = 0.1;
    {
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = L_OC_Grid.begin();

        cv::Point3f _robot_point = PCL2CV(robot_pos);
        _robot_point.y = 0;
        for(;ite != L_OC_Grid.end();ite++)
        {
            cv::Point3f cell_val_0 = ite->second;
            cv::Point3f query_point;
            query_point.x = ((float)ite->first.first + 0.5) * dim;
            query_point.y = 0;
            query_point.z = ((float)ite->first.second + 0.5) * dim;
            if( (ite->second.x !=0  && cell_val_0 != cv::Point3f(1,1,1)))
            {
                bool first_observed = 0;
                uint first_occ_add = 0;
                std::pair<int,int> first_occ_pair;
                cv::Point3f P1 = query_point;
                cv::Point3f P0 = _robot_point;
                float dist = vec_mag((P0 - P1));
                if(dist < max_dist)
                {
                    P1 = ( (P1-P0)*(1/dist) ) * max_dist + P0;
                    dist = max_dist;
                }
                if(fabs(RobotPointAngle(PCL2CV(heading),(query_point-_robot_point))) <= 50 && dist <= max_dist)
                {
                    float m = ceil(dist/resolution);
                    for(uint q=1;q<=m;q++)
                    {
                        cv::Point3f _Robot_pos = ( 1-((float)q/m) )*P0 + ((float)q/m)*P1;
                        i = (floor)((_Robot_pos.x)/dim);
                        j = (floor)((_Robot_pos.z)/dim);

                        cv::Point3f cell_val = L_OC_Grid[make_pair(i,j)];
                        Visited_Grid_Block[make_pair(i,j)] = 1;

                        if((cell_val.x != 0 || Visited_Occ_Block[make_pair(i,j)].first == 1) && !first_observed)
                        {
                            //                            if(Visited_Occ_Block[make_pair(i,j)] == 1)
                            //                                L_OC_Grid[make_pair(i,j)] = cv::Point3f(1.0,0,0);
                            if(Visited_Occ_Block[make_pair(i,j)].first == 0)
                            {
                                Visited_Occ_Block[make_pair(i,j)].first = 1;
                                Visited_Occ_Block[make_pair(i,j)].second.y = cell_val.x;
                                first_occ_add = 1;
                                first_occ_pair = make_pair(i,j);
                            }

                            first_observed = 1;
                        }
                        else if(Visited_Occ_Block[make_pair(i,j)].first ==0 && cell_val.x != 0  && first_observed)
                        {
                            //                                L_OC_Grid[make_pair(i,j)] = cv::Point3f(0,0,0);
                            Visited_Occ_Block[make_pair(i,j)].first = 1;
                            Visited_Occ_Block[make_pair(i,j)].second.y = cell_val.x;
                        }
                    }
                }else
                    ite->second = cv::Point3f(0,0,0);

            }
        }
    }

    {
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = L_OC_Grid.begin();
        for(;ite != L_OC_Grid.end();ite++)
        {
            if(ite->second.x !=0)
                OC_Grid[ite->first] = 0.5*(OC_Grid[ite->first]+ite->second);
        }

    }
}
void Frontier::newSearch(PointT robot_pos,PointT heading)
{
    float dim = 0.1;
    std::map<std::pair<int,int>,cv::Point3f > L_OC_Grid;
    float _x = -4;
    float _y = -4;
    int i,j;
    cv::Point2i _min(1000000,100000),_max(-100000,-100000);
    while(_x < 4)
    {
        while(_y<4)
        {
            i = (floor)((_x)/dim);
            j = (floor)((_y)/dim);
            _min.x = (_min.x > i)?(i):(_min.x);
            _min.y = (_min.y > j)?(j):(_min.y);

            _max.x = (_max.x < i)?(i):(_max.x);
            _max.y = (_max.y < j)?(j):(_max.y);

            L_OC_Grid[make_pair(i,j)] = cv::Point3f(0,0,0);
            _y +=0.1;
        }
        _y = -4;
        _x +=0.1;
    }

    float max_r = 2;
    float max_r_2 = max_r*max_r;
    int max_r_idx = (floor)((max_r)/dim);
    cv::Point2i robot_idx;
    robot_idx.x = (floor)((robot_pos.x)/dim);
    robot_idx.y = (floor)((robot_pos.z)/dim);

    int start_id_x = ((robot_idx.x-max_r_idx) < _min.x)?(_min.x):((robot_idx.x-max_r_idx));
    int end_id_x = ((robot_idx.x+max_r_idx) > _max.x)?(_max.x):((robot_idx.x+max_r_idx));

    int start_id_y = ((robot_idx.y-max_r_idx) < _min.y)?(_min.y):((robot_idx.y-max_r_idx));
    int end_id_y = ((robot_idx.y+max_r_idx) > _max.y)?(_max.y):((robot_idx.y+max_r_idx));

    for(int i=start_id_x;i<=end_id_x;i++)
    {
        for(int j=start_id_y;j<=end_id_y;j++)
        {
            cv::Point3f query_point;
            query_point.x = ((float)i + 0.5) * dim;
            query_point.y = robot_pos.y;
            query_point.z = ((float)j + 0.5) * dim;
            float angel = fabs(RobotPointAngle(PCL2CV(heading),(query_point-PCL2CV(robot_pos))));

            if(vec_mag_2((query_point - PCL2CV(robot_pos))) < max_r_2 && angel<=50)
                L_OC_Grid[make_pair(i,j)] = cv::Point3f(1,1,1);
        }
    }

    draw_occ_grid(L_OC_Grid,PointT(0,-2,0),dim,"LL_OC_Grid");
}
void Frontier::OccupancyGrid(_PointCloudT _cloud, float dim, PointT robot_pos, PointT heading)
{
    int i,j;
    float obst_dist = 0.4;
    std::map<std::pair<int,int>,PointCloud<PointXYZ> > cloud_vec;
    cv::Point2i _min(1000000,100000),_max(-100000,-100000);
    for(int c=0;c<_cloud->points.size();c++)
    {
        i = (floor)((_cloud->points[c].x)/dim);
        j = (floor)((_cloud->points[c].z)/dim);

        _min.x = (_min.x > i)?(i):(_min.x);
        _min.y = (_min.y > j)?(j):(_min.y);

        _max.x = (_max.x < i)?(i):(_max.x);
        _max.y = (_max.y < j)?(j):(_max.y);

        cloud_vec[make_pair(i,j)].points.push_back(_cloud->points[c]);
    }

    _min.x -= 2;_min.y -= 2;
    _max.x += 2;_max.y += 2;

    std::map<std::pair<int,int>,cv::Point3f > L_OC_Grid;
    std::map<std::pair<int,int>,bool > Visited_Grid_Block;
    for(int k=_min.x;k<=_max.x;k++)
    {
        for(int q=_min.y;q<=_max.y;q++)
        {
            float tmp = OC_Grid[make_pair(k,q)].x + OC_Grid[make_pair(k,q)].y + OC_Grid[make_pair(k,q)].z ;
            if(tmp == 0)
                OC_Grid[make_pair(k,q)] = cv::Point3f(0,0,0);

            //            if(L_OC_Grid[make_pair(k,q)] == cv::Point3f(0,0,0))
            L_OC_Grid[make_pair(k,q)] = cv::Point3f(0,0,0);
        }
    }
    //    EdgeDetection(OC_Grid,robot_pos,dim);
    {
        std::map<std::pair<int,int>,PointCloud<PointXYZ> >::iterator ite = cloud_vec.begin();
        uint _size;
        for(;ite != cloud_vec.end();ite++)
        {
            _size = ite->second.points.size();
            if(_size > 5)
            {
                L_OC_Grid[ite->first] = cv::Point3f(1.0,0,0);
            }
            else if(_size > 2)
            {
                L_OC_Grid[ite->first] = cv::Point3f(0.5,0,0);
            }
        }
    }

    float max_dist = RANGE_OF_VIEW;
    float max_dist_2 = RANGE_OF_VIEW*RANGE_OF_VIEW;
    float resolution = 0.1;
    {
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = L_OC_Grid.begin();

        cv::Point3f _robot_point = PCL2CV(robot_pos);
        _robot_point.y = 0;
        for(;ite != L_OC_Grid.end();ite++)
        {
            cv::Point3f cell_val_0 = ite->second;
            cv::Point3f query_point;
            query_point.x = ((float)ite->first.first + 0.5) * dim;
            query_point.y = 0;
            query_point.z = ((float)ite->first.second + 0.5) * dim;
            if( (ite->second.x !=0  && cell_val_0 != cv::Point3f(1,1,1)) && Visited_Occ_Block[ite->first].second.x < obst_dist)
            {
                bool first_observed = 0;
                uint first_occ_add = 0;
                std::pair<int,int> first_occ_pair;
                cv::Point3f P1 = query_point;
                cv::Point3f P0 = _robot_point;
                float dist = vec_mag((P0 - P1));
                if(dist < max_dist)
                {
                    P1 = ( (P1-P0)*(1/dist) ) * max_dist + P0;
                    dist = max_dist;
                }
                if(fabs(RobotPointAngle(PCL2CV(heading),(query_point-_robot_point))) <= 50 && dist <= max_dist)
                {
                    float m = ceil(dist/resolution);
                    for(uint q=1;q<=m;q++)
                    {
                        cv::Point3f _Robot_pos = ( 1-((float)q/m) )*P0 + ((float)q/m)*P1;
                        i = (floor)((_Robot_pos.x)/dim);
                        j = (floor)((_Robot_pos.z)/dim);

                        cv::Point3f cell_val = L_OC_Grid[make_pair(i,j)];
                        Visited_Grid_Block[make_pair(i,j)] = 1;

                        if(cell_val == cv::Point3f(0,0,0) && !first_observed && (Visited_Occ_Block[make_pair(i,j)].first == 0 || Visited_Occ_Block[make_pair(i,j)].second.x > obst_dist))
                        {
                            //                            if(Base_Cloud_Grid[make_pair(i,j)] != cv::Point3f(0,0,0))
                            L_OC_Grid[make_pair(i,j)] = cv::Point3f(1,1,1);

                        }
                        else if(( (cell_val.x != 0 && cell_val != cv::Point3f(1,1,1)) || Visited_Occ_Block[make_pair(i,j)].first == 1) && !first_observed)
                        {
                            //                            if(Visited_Occ_Block[make_pair(i,j)] == 1)
                            //                                L_OC_Grid[make_pair(i,j)] = cv::Point3f(1.0,0,0);
                            if(Visited_Occ_Block[make_pair(i,j)].first == 0)
                            {
                                Visited_Occ_Block[make_pair(i,j)].first = 1;
                                Visited_Occ_Block[make_pair(i,j)].second.y = cell_val.x;
                                //                                first_occ_add = 1;
                                first_occ_pair = make_pair(i,j);
                            }
                            NONVisited_Occ_Block[make_pair(i,j)] = 0;
                            first_observed = 1;
                        }
                        else if(Visited_Occ_Block[make_pair(i,j)].first ==0 && cell_val.x != 0 && cell_val != cv::Point3f(1,1,1) && first_observed)
                        {
                            //                            if(Visited_Occ_Block[first_occ_pair].second.x < obst_dist)
                            //                                L_OC_Grid[make_pair(i,j)] = cv::Point3f(0,0,0);
                            NONVisited_Occ_Block[make_pair(i,j)] = 1;
                            Visited_Occ_Block[make_pair(i,j)].first = 1;
                            Visited_Occ_Block[make_pair(i,j)].second.y = cell_val.x;
                        }

                        if(first_occ_add == 1 && Visited_Occ_Block[make_pair(i,j)].first == 0 && Visited_Occ_Block[make_pair(i,j)].second.x < obst_dist) //OC_Grid[make_pair(i,j)] == cv::Point3f(1,1,1) &&
                        {
                            std::map<std::pair<int,int>,cv::Point3f >::iterator ite2 = OC_Grid.begin();
                            for(;ite2 != OC_Grid.end();ite2++)
                            {
                                cv::Point3f query_point;
                                query_point.x = ((float)ite2->first.first + 0.5) * dim;
                                query_point.y = 0;
                                query_point.z = ((float)ite2->first.second + 0.5) * dim;

                                float dist2 = vec_mag_2((_Robot_pos-query_point));

                                if( ite2->second == cv::Point3f(1,1,1) && dist2 < 0.0225 )
                                {
                                    if(Visited_Occ_Block[ite2->first].first == 0)
                                    {
                                        OC_Grid[ite2->first] = cv::Point3f(0,0,0);
                                        L_OC_Grid[ite2->first] = cv::Point3f(0,0,0);
                                    }
                                    //                                    else
                                    //                                        first_occ_add = 2;


                                    OC_Grid[make_pair(i,j)] = cv::Point3f(0,0,0);
                                    L_OC_Grid[make_pair(i,j)] = cv::Point3f(0,0,0);

                                }
                            }
                        }
                        else if(first_occ_add == 1 && Visited_Occ_Block[make_pair(i,j)].first == 1 && first_occ_pair != make_pair(i,j))
                            first_occ_add = 2;
                    }
                }else
                    ite->second = cv::Point3f(0,0,0);

            }
        }
    }

    {
        cv::Point3f rotated_vec;
        float theta;
        cv::Point3f _robot_point = PCL2CV(robot_pos);
        _robot_point.y = 0;
        for(int k=-50;k<=50;k++)
        {
            theta = ((float)k)*(3.1415/180.0);
            rotated_vec.x = cos(theta)*heading.x + sin(theta)*heading.z;
            rotated_vec.y = 0;
            rotated_vec.z = cos(theta)*heading.z - sin(theta)*heading.x;

            rotated_vec = rotated_vec*(1/vec_mag(rotated_vec));
            cv::Point3f P1 =  _robot_point + (max_dist+3)*rotated_vec;;
            cv::Point3f P0 = _robot_point;
            float dist = vec_mag((P0 - P1));
            float m = ceil(dist/resolution);
            bool _observed = 0;

            std::vector<std::pair<int,int> > make_free;

            for(uint q=1;q<=m;q++)
            {
                cv::Point3f Generated_POS = ( 1-((float)q/m) )*P0 + ((float)q/m)*P1;

                i = (floor)((Generated_POS.x)/dim);
                j = (floor)((Generated_POS.z)/dim);

                if(i <= _max.x && j<=_max.y && i >= _min.x && j>=_min.y)
                {
                    Visited_Grid_Block[make_pair(i,j)] = 1;
                    if(L_OC_Grid[make_pair(i,j)] != cv::Point3f(1,1,1) && (Visited_Occ_Block[make_pair(i,j)].first == 0 || Visited_Occ_Block[make_pair(i,j)].second.x > obst_dist) )
                    {
                        float dist2 = vec_mag_2((_robot_point - Generated_POS));

                        float max_r = 0.25;
                        float max_r_2 = max_r*max_r;
                        int max_r_idx = (floor)((max_r)/dim);
                        cv::Point2i robot_idx;
                        robot_idx.x = (floor)((Generated_POS.x)/dim);
                        robot_idx.y = (floor)((Generated_POS.z)/dim);

                        int start_id_x = ((robot_idx.x-max_r_idx) < _min.x)?(_min.x):((robot_idx.x-max_r_idx));
                        int end_id_x = ((robot_idx.x+max_r_idx) > _max.x)?(_max.x):((robot_idx.x+max_r_idx));

                        int start_id_y = ((robot_idx.y-max_r_idx) < _min.y)?(_min.y):((robot_idx.y-max_r_idx));
                        int end_id_y = ((robot_idx.y+max_r_idx) > _max.y)?(_max.y):((robot_idx.y+max_r_idx));

                        for(int _i=start_id_x;_i<=end_id_x;_i++)
                        {
                            for(int _j=start_id_y;_j<=end_id_y;_j++)
                            {
                                cv::Point3f _query_point;
                                _query_point.x = ((float)_i + 0.5) * dim;
                                _query_point.y = 0;
                                _query_point.z = ((float)_j + 0.5) * dim;

                                if(vec_mag_2((_query_point - Generated_POS)) < max_r_2 )
                                {

                                    if(Base_RawCloud_Grid[make_pair(_i,_j)] != cv::Point3f(0,0,0))
                                    {
                                        make_free.clear();
                                        make_free.push_back(make_pair(_i,_j));
                                    }

                                    if(( (L_OC_Grid[make_pair(_i,_j)].x !=0 && L_OC_Grid[make_pair(_i,_j)] != cv::Point3f(1,1,1)) ||
                                         Visited_Occ_Block[make_pair(_i,_j)].first == 1 )  && _observed == 0)
                                    {
                                        if(Visited_Occ_Block[make_pair(_i,_j)].second.x <= obst_dist || Obst_OC_Grid[make_pair(_i,_j)].x != 0)
                                        {
                                            NONVisited_Occ_Block[make_pair(_i,_j)] = 0;
                                            _observed = 1;
                                            q = m ;
                                        }
                                    }
                                }
                            }
                        }
                        if(!_observed && dist2< max_dist_2)
                        {
                            if(Visited_Occ_Block[make_pair(i,j)].first == 0 || Visited_Occ_Block[make_pair(i,j)].second.x > obst_dist)
                            {
                                if(L_OC_Grid[make_pair(i,j)].x == 0 && Obst_OC_Grid[make_pair(i,j)].x == 0 )//&& Base_Cloud_Grid[make_pair(i,j)] != cv::Point3f(0,0,0))
                                {
                                    L_OC_Grid[make_pair(i,j)] = cv::Point3f(1,1,1);
                                    make_free.push_back(make_pair(i,j));
                                }
                            }
                        }
                    }
                }
            }

            for(uint q=0;q<make_free.size();q++)
            {
                L_OC_Grid[make_free[q]] = cv::Point3f(0,0,0);
                OC_Grid[make_free[q]] = OC_Grid[make_free[q]];
            }
        }
    }

    {
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = L_OC_Grid.begin();
        for(;ite != L_OC_Grid.end();ite++)
        {
            if(ite->second.x !=0)
            {
                OC_Grid[ite->first] = 0.5*(OC_Grid[ite->first]+ite->second);
                OC_Grid_For_view[ite->first] = 0.5*(OC_Grid_For_view[ite->first]+ite->second);

            }

            if(Visited_Occ_Block[ite->first].first == 1)
            {
                if(OC_Grid[ite->first].z > 0.8)
                {
                    Visited_Occ_Block[ite->first] = make_pair(0,cv::Point2f(0,0));
                    OC_Grid[ite->first] = cv::Point3f(1,1,1);
                    OC_Grid_For_view[ite->first] = cv::Point3f(1,1,1);
                }
            }
            else if(OC_Grid[ite->first].z < 0.2)
            {
                OC_Grid[ite->first] = cv::Point3f(0,0,0);
                OC_Grid_For_view[ite->first] = cv::Point3f(0,0,0);

            }

            if(NONVisited_Occ_Block[ite->first] == 1)
                OC_Grid_For_view[ite->first] = cv::Point3f(0,0,0);


            //            if(Visited_Occ_Block[ite->first].first ==1)
            //            {
            //                if(Visited_Occ_Block[ite->first].second.x >obst_dist)
            //                {
            //                    //                   Visited_Occ_Block[ite->first].second.x = 0.5*(Visited_Occ_Block[ite->first].second.x + L_OC_Grid[ite->first].x);
            //                    OC_Grid[ite->first] = cv::Point3f(0,1,1);
            //                }
            //                //                else
            //                //                  OC_Grid[ite->first] = cv::Point3f(1,0,0);
            //            }
        }

    }
    //    draw_occ_grid(L_OC_Grid,PointT(4,-0.2,0),dim,"L_OC_GRID");
}

void Frontier::Determin_Cloud_Base(_PointCloudT _cloud, float dim)
{
    int i,j;
    std::map<std::pair<int,int>,PointCloud<PointXYZ> > cloud_vec;
    std::map<std::pair<int,int>,cv::Point3f > Border_Cloud_Grid;
    cv::Point2i _min(100000000000,100000000),_max(-100000000,-1000000);

    for(int c=0;c<_cloud->points.size();c++)
    {
        i = (floor)((_cloud->points[c].x)/dim);
        j = (floor)((_cloud->points[c].z)/dim);

        _min.x = (_min.x > i)?(i):(_min.x);
        _min.y = (_min.y > j)?(j):(_min.y);

        _max.x = (_max.x < i)?(i):(_max.x);
        _max.y = (_max.y < j)?(j):(_max.y);

        cloud_vec[make_pair(i,j)].points.push_back(_cloud->points[c]);
    }

    for(int k=_min.x;k<=_max.x;k++)
    {
        for(int q=_min.y;q<=_max.y;q++)
        {
            Border_Cloud_Grid[make_pair(k,q)] = cv::Point3f(1,1,1);
        }
    }

    pair<int,int> max_member;
    {
        std::map<std::pair<int,int>,PointCloud<PointXYZ> >::iterator ite = cloud_vec.begin();
        uint _size,max_size = 0;
        for(;ite != cloud_vec.end();ite++)
        {
            _size = ite->second.points.size();
            if(_size > 3)
            {
                if(max_size < _size)
                {
                    max_member = ite->first;
                    max_size = _size;
                }
                Base_Cloud_Grid[ite->first] = (cv::Point3f(1.0,0,1.0) + Base_Cloud_Grid[ite->first])*0.5;
            }
            else if(_size <= 1)
            {
                Base_Cloud_Grid[ite->first] = Base_Cloud_Grid[ite->first]*0.9;
            }
        }
    }

    {
        std::vector<cv::Point3f> query_points;
        query_points.push_back(cv::Point3f(_min.x,0,_min.y));
        query_points.push_back(cv::Point3f(_min.x,0,_max.y));
        query_points.push_back(cv::Point3f(_max.x,0,_min.y));
        query_points.push_back(cv::Point3f(_max.x,0,_max.y));

        float resolution = 0.1;

        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = Border_Cloud_Grid.begin();
        for(;ite != Border_Cloud_Grid.end();ite++)
        {
            if(ite->first.first == _min.x || ite->first.first == _max.x || ite->first.second == _min.y || ite->first.second == _max.y)
            {
                for(uint k = 0;k<query_points.size();k++)
                {
                    cv::Point3f P1;
                    P1.x = (query_points[k].x + 0.5) * dim;
                    P1.y = 0;
                    P1.z = (query_points[k].z + 0.5) * dim;

                    cv::Point3f P0;
                    P0.x = ((float)ite->first.first + 0.5) * dim;
                    P0.y = 0;
                    P0.z = ((float)ite->first.second + 0.5) * dim;

                    float dist = vec_mag((P0 - P1));
                    float m = ceil(dist/resolution);
                    for(uint q=1;q<m;q++)
                    {
                        cv::Point3f Generated_POS = ( 1-((float)q/m) )*P0 + ((float)q/m)*P1;
                        i = (floor)((Generated_POS.x)/dim);
                        j = (floor)((Generated_POS.z)/dim);

                        cv::Point3f cell_val = Base_Cloud_Grid[make_pair(i,j)];

                        if(cell_val != cv::Point3f(0,0,0))
                        {
                            q = m;
                            Border_Cloud_Grid[make_pair(i,j)] = cv::Point3f(1,0,0);
                        }
                        else
                        {
                            Border_Cloud_Grid[make_pair(i,j)] = cv::Point3f(0,0,0);
                        }
                    }

                }
            }
        }

        ite = Border_Cloud_Grid.begin();
        for(;ite != Border_Cloud_Grid.end();ite++)
        {
            if(ite->second != cv::Point3f(0,0,0))
            {
                Base_Cloud_Grid[ite->first] = cv::Point3f(1.0,0,1.0);
            }
        }
    }

    //    draw_occ_grid(Base_Cloud_Grid,PointT(0,-0.4,0),dim,"Base_CL_GRID");
    //    draw_occ_grid(Border_Cloud_Grid,PointT(0,-0.5,0),dim,"Border_CL_GRID");
}

void Frontier::DeterminBaseOfRawCloud(Frontier::_PointCloudT _cloud, float dim)
{
    int i,j;
    std::map<std::pair<int,int>,PointCloud<PointXYZ> > cloud_vec;
    std::map<std::pair<int,int>,cv::Point3f > Border_Cloud_Grid;
    cv::Point2i _min(100000000000,100000000),_max(-100000000,-1000000);

    for(int c=0;c<_cloud->points.size();c++)
    {
        i = (floor)((_cloud->points[c].x)/dim);
        j = (floor)((_cloud->points[c].z)/dim);

        _min.x = (_min.x > i)?(i):(_min.x);
        _min.y = (_min.y > j)?(j):(_min.y);

        _max.x = (_max.x < i)?(i):(_max.x);
        _max.y = (_max.y < j)?(j):(_max.y);

        cloud_vec[make_pair(i,j)].points.push_back(_cloud->points[c]);
    }

    for(int k=_min.x;k<=_max.x;k++)
    {
        for(int q=_min.y;q<=_max.y;q++)
        {
            Border_Cloud_Grid[make_pair(k,q)] = cv::Point3f(1,1,1);
        }
    }

    pair<int,int> max_member;
    {
        std::map<std::pair<int,int>,PointCloud<PointXYZ> >::iterator ite = cloud_vec.begin();
        uint _size,max_size = 0;
        for(;ite != cloud_vec.end();ite++)
        {
            _size = ite->second.points.size();
            if(_size > 2)
            {
                if(max_size < _size)
                {
                    max_member = ite->first;
                    max_size = _size;
                }
                Base_RawCloud_Grid[ite->first] = (cv::Point3f(1.0,0,1.0) + Base_RawCloud_Grid[ite->first])*0.5;
            }
            //            else if(_size <= 1)
            //            {
            //                Base_RawCloud_Grid[ite->first] = Base_RawCloud_Grid[ite->first]*0.9;
            //            }
        }
    }

    //    contour_str base_contour =  ContourExtractor(Base_RawCloud_Grid,cv::Point2i(5,5),8);

    //    /// Draw contours
    //    cv::vector<vector<Point> > approxShape;
    //    approxShape.resize(base_contour.contours.size());
    //    RNG rng(12345);
    //    cv::Mat drawing = cv::Mat::zeros( base_contour.IMG.size(), CV_8UC3 );
    //    for( int i = 0; i< base_contour.contours.size(); i++ )
    //    {
    //        cv::approxPolyDP(base_contour.contours[i], approxShape[i],5, true);
    //        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //        cv::drawContours( drawing, approxShape, i, color,1, 8, base_contour.hierarchy, 0, Point() );
    //    }
    //    imshow("cc",drawing);

    {
        PointT offset = PointT(0,-0.8,0);

        pcl::PointCloud<PointXYZRGBA>::Ptr for_draw_Cloud (new PointCloud<PointXYZRGBA>());
        //        pcl::PointCloud<PointXYZ>::Ptr for_dense_Cloud (new PointCloud<PointXYZ>());
        pcl::PointXYZRGBA tmp_point;
        //        pcl::PointXYZ tmp_point2;
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = Base_RawCloud_Grid.begin();
        for(;ite != Base_RawCloud_Grid.end();ite++)
        {
            tmp_point.x = offset.x + ((float)ite->first.first + 0.5) * dim;
            tmp_point.y = offset.y;
            tmp_point.z = offset.z + ((float)ite->first.second + 0.5) * dim;
            tmp_point.r = ite->second.x *255;
            tmp_point.g = ite->second.y *255;
            tmp_point.b = ite->second.z *255;

            //            tmp_point2.x = offset.x + ((float)ite->first.first + 0.5) * dim;
            //            tmp_point2.y = offset.y;
            //            tmp_point2.z = offset.z + ((float)ite->first.second + 0.5) * dim;

            //            for_dense_Cloud->points.push_back(tmp_point2);
            for_draw_Cloud->points.push_back(tmp_point);
        }

        //        cloud_denser(for_dense_Cloud,for_dense_Cloud,dim*3,5,10);

        //        uint cccc = 0;
        //        for(int c=0;c<for_dense_Cloud->points.size();c++)
        //        {
        //            i = (floor)((for_dense_Cloud->points[c].x)/dim);
        //            j = (floor)((for_dense_Cloud->points[c].z)/dim);

        //            if(Base_RawCloud_Grid[make_pair(i,j)] == cv::Point3f(0,0,0))
        //            {
        //                Base_RawCloud_Grid[make_pair(i,j)] = cv::Point3f(1.0,0,1.0);
        //                cccc++;
        //            }
        //        }
        //        qDebug()<<cccc;


        //                viewer_thr->Draw_cloud2viewer(for_draw_Cloud,"Base_RawCL_GRID",8);
    }
}

void Frontier::EdgeDetection(std::map<std::pair<int, int>, Point3f> Base_Cloud_Grid,
                             std::map<std::pair<int, int>, std::pair<bool, Point2f> > &Obst_point, float dim)
{
    if(Base_Cloud_Grid.size()>20)
    {
        contour_str base_contour =  ContourExtractor(Base_Cloud_Grid,cv::Point2i(5,5),8);

        /// Draw contours
        RNG rng(12345);
        cv::Mat drawing = cv::Mat::zeros( base_contour.IMG.size(), CV_8UC3 );
        uint max_size = 0;
        cv::vector<vector<Point> > candidate_contours;
        cv::vector<Vec4i> candidate_hierarchy;
        vector<Point> external_contour;


        for( int i = 0; i< base_contour.contours.size(); i++ )
        {
            if(base_contour.contours[i].size() > max_size)
            {
                max_size = base_contour.contours[i].size();
            }
        }

        for( int i = 0; i< base_contour.contours.size(); i++ )
        {
            if(base_contour.contours[i].size() > 0.5*max_size)
            {
                candidate_contours.push_back(base_contour.contours[i]);
                candidate_hierarchy.push_back(base_contour.hierarchy[i]);

                for(uint k=0;k<base_contour.contours[i].size();k++)
                    external_contour.push_back(base_contour.contours[i][k]);
            }
        }
        vector<Point> comprihensive_countour;
        vector<vector<Point> >hull(1);
        for(int i = 0; i < candidate_contours.size(); i++)
        {
            for(uint j=0;j<candidate_contours[i].size();j++)
                comprihensive_countour.push_back(candidate_contours[i][j]);
        }

        cv::convexHull( cv::Mat(comprihensive_countour), hull[0], false );

//        for( int i = 0; i< candidate_contours.size(); i++ )
//        {
//            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//            cv::drawContours( drawing, candidate_contours, i, color,1, 8, candidate_hierarchy, 0, Point() );

//            cv::drawContours( drawing, hull, 0, color, 1, 8, candidate_hierarchy, 0, Point() );
//        }



        {
            cv::Point2f Query_POS ;
            std::map<std::pair<int,int>,std::pair<bool,cv::Point2f> >::iterator ite2 = Obst_point.begin();
            for(;ite2 != Obst_point.end();ite2++)
            {
                if(ite2->second.first == 1 && external_contour.size() > 10)
                {
                    Query_POS.y = base_contour.scale*( ite2->first.first - base_contour._min.x + base_contour.offset.x);
                    Query_POS.x = base_contour.scale*( ite2->first.second - base_contour._min.y + base_contour.offset.y);
                    float kk=0;
                    if(candidate_contours.size() > 1)
                    {
                        kk = fabs(cv::pointPolygonTest(hull[0],Query_POS,true));
                    }
                    else
                    {
                        kk = fabs(cv::pointPolygonTest(external_contour,Query_POS,true));
                    }


                    kk *= (dim/base_contour.scale);
                    ite2->second.second.x = kk;

//                    if(kk > 0.5)
//                        cv::circle(drawing,Query_POS,5,cv::Scalar(0,200,0),2);
//                    else
//                        cv::circle(drawing,Query_POS,5,cv::Scalar(0,0,200),2);
                }
            }
        }
        {
            cv::Point2f Query_POS ;
            std::map<std::pair<int,int>,cv::Point3f >::iterator ite2 = Base_RawCloud_Grid.begin();
            for(;ite2 != Base_RawCloud_Grid.end();ite2++)
            {
                if(ite2->second != cv::Point3f(0,0,0) && external_contour.size() > 10)
                {
                    Query_POS.y = base_contour.scale*( ite2->first.first - base_contour._min.x + base_contour.offset.x);
                    Query_POS.x = base_contour.scale*( ite2->first.second - base_contour._min.y + base_contour.offset.y);

                    float kk = fabs(cv::pointPolygonTest(external_contour,Query_POS,true));
                    kk *= (dim/base_contour.scale);
                    if(kk>0.6)
                        ite2->second = cv::Point3f(0,0,0);

                    //                    if(kk > 0.5)
                    //                        cv::circle(drawing,Query_POS,5,cv::Scalar(0,200,0),2);
                    //                    else
                    //                        cv::circle(drawing,Query_POS,5,cv::Scalar(0,0,200),2);
                }
            }
        }
//        cv::imshow("IMG",drawing);

        {
            cv::Point2f Query_POS ;
            std::map<std::pair<int,int>,cv::Point3f >::iterator ite2 = OC_Grid.begin();
            for(;ite2 != OC_Grid.end();ite2++)
            {
                if(ite2->second != cv::Point3f(0,0,0) && external_contour.size() > 10 && Obst_point[ite2->first].first == 0)
                {
                    if(inside_OC_Grid_point[ite2->first] < 20)
                    {
                        Query_POS.y = base_contour.scale*( ite2->first.first - base_contour._min.x + base_contour.offset.x);
                        Query_POS.x = base_contour.scale*( ite2->first.second - base_contour._min.y + base_contour.offset.y);

                        float kk = cv::pointPolygonTest(external_contour,Query_POS,true);
                        kk *= (dim/base_contour.scale);

                        if(kk < 0)
                        {
                            ite2->second = 0.9*ite2->second;
                            inside_OC_Grid_point[ite2->first] = inside_OC_Grid_point[ite2->first] - 1;
                        }
                        else
                            inside_OC_Grid_point[ite2->first] = inside_OC_Grid_point[ite2->first] + 1;
                    }
                }
            }
        }

        //        std::map<std::pair<int, int>, Point3f> out_put;
        //        for(uint i=0;i<drawing.rows;i++)
        //        {
        //            for(uint j=0;j <drawing.cols;j++)
        //            {
        //                cv::Vec3b color = drawing.at<cv::Vec3b>(i,j);
        //                if(color != cv::Vec3b(0,0,0))
        //                {
        //                    int k,q;
        //                    k = floor(((float)i/scale)) - offset.x + _min.x;
        //                    q = floor(((float)j/scale)) - offset.y + _min.y;
        //                    out_put[make_pair(k,q)] = cv::Point3f(0,1,1);
        //                }
        //            }
        //        }

        //        draw_occ_grid(out_put,PointT(0,-0.5,0),dim,"BORDER_OF_BASE");
    }
}

void Frontier::CheckFrontierValidation(std::map<std::pair<int, int>, Point3f> Base_Cloud_Grid,
                                       std::vector<std::pair<PointT,bool> >& _Frontier_points, float dim)
{
    uint outlier_frontiers = 0;
    if(Base_Cloud_Grid.size()>20)
    {
        contour_str base_contour =  ContourExtractor(Base_Cloud_Grid,cv::Point2i(5,5),8);

        /// Draw contours
        RNG rng(12345);
        cv::Mat drawing = cv::Mat::zeros( base_contour.IMG.size(), CV_8UC3 );
        uint max_size = 0;
        cv::vector<vector<Point> > candidate_contours;
        cv::vector<Vec4i> candidate_hierarchy;
        vector<Point> external_contour;

        for( int i = 0; i< base_contour.contours.size(); i++ )
        {
            if(base_contour.contours[i].size() > max_size)
            {
                max_size = base_contour.contours[i].size();
            }
        }

        for( int i = 0; i< base_contour.contours.size(); i++ )
        {
            if(base_contour.contours[i].size() > 0.5*max_size)
            {
                candidate_contours.push_back(base_contour.contours[i]);
                candidate_hierarchy.push_back(base_contour.hierarchy[i]);

                for(uint k=0;k<base_contour.contours[i].size();k++)
                    external_contour.push_back(base_contour.contours[i][k]);
            }
        }


        {
            cv::Point2f Query_POS ;
            for(uint i=0;i<_Frontier_points.size();i++)
            {
                if(_Frontier_points[i].second == 0 && external_contour.size() > 10)
                {
                    int i_x = (floor)((_Frontier_points[i].first.x)/dim);
                    int j_y = (floor)((_Frontier_points[i].first.z)/dim);
                    Query_POS.y = base_contour.scale*( i_x - base_contour._min.x + base_contour.offset.x);
                    Query_POS.x = base_contour.scale*( j_y - base_contour._min.y + base_contour.offset.y);

                    float kk = cv::pointPolygonTest(external_contour,Query_POS,true);
                    kk *= (dim/base_contour.scale);

                    if(kk < -0.4)
                    {
                        outlier_frontiers++;
                        _Frontier_points[i].second = 1;
                    }
                }
            }
        }
    }
    qDebug()<<"outlier_frontiers:"<<outlier_frontiers;
}

Frontier::contour_str Frontier::ContourExtractor(std::map<std::pair<int, int>, Point3f> Cloud_Grid, Point2i offset, float scale)
{
    cv::Point2i _min(100000000,100000000),_max(-100000000,-100000000);
    int i,j;
    contour_str out_put;
    std::map<std::pair<int,int>,cv::Point3f >::iterator ite = Cloud_Grid.begin();
    for(;ite != Cloud_Grid.end();ite++)
    {
        i = ite->first.first;
        j = ite->first.second;

        _min.x = (_min.x > i)?(i):(_min.x);
        _min.y = (_min.y > j)?(j):(_min.y);

        _max.x = (_max.x < i)?(i):(_max.x);
        _max.y = (_max.y < j)?(j):(_max.y);
    }
    i =  _max.x - _min.x + 1;
    j =  _max.y - _min.y + 1;
    cv::Mat IMG = cv::Mat::zeros(i+offset.x,j+offset.y,CV_8UC1);

    uchar val;
    for(int k=_min.x;k<=_max.x;k++)
    {
        for(int q=_min.y;q<=_max.y;q++)
        {
            if(Cloud_Grid[make_pair(k,q)] == cv::Point3f(0,0,0))
                val = 0;
            else
                val = 255;

            IMG.at<uchar>(k-_min.x+offset.x,q-_min.y + offset.y) = val;
        }
    }
    cv::Mat Scaled_IMG(IMG.rows*scale,IMG.cols*scale,CV_8UC1);
    cv::resize(IMG,Scaled_IMG,Scaled_IMG.size(),0,0,INTER_AREA);

    /// Find contours
    cv::findContours(Scaled_IMG, out_put.contours, out_put.hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    out_put._min = _min; out_put._max = _max;
    out_put.offset = offset;
    out_put.scale = scale;
    out_put.IMG = Scaled_IMG.clone();

    return out_put;
}

void Frontier::Determin_Margin_Area(float dim, bool disp)
{
    //    {
    //        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = OC_Grid.begin();
    //        for(;ite != OC_Grid.end();ite++)
    //        {
    //            if(ite->second != cv::Point3f(0,0,0) && Base_Cloud_Grid[ite->first] == cv::Point3f(0,0,0))
    //                Base_Cloud_Grid[ite->first] = cv::Point3f(1,0,1);
    //        }
    //    }

    EdgeDetection(Base_Cloud_Grid,Visited_Occ_Block,dim);

    if(disp)
    {
        PointXYZRGBA tmp_point;
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = Base_Cloud_Grid.begin();
        PointCloud<PointXYZRGBA>::Ptr for_draw_Cloud (new PointCloud<PointXYZRGBA>());
        for(;ite != Base_Cloud_Grid.end();ite++)
        {
            tmp_point.x = ((float)ite->first.first + 0.5) * dim;
            tmp_point.y = 0.8;
            tmp_point.z = ((float)ite->first.second + 0.5) * dim;
            tmp_point.r = ite->second.x *255;
            tmp_point.g = ite->second.y *255;
            tmp_point.b = ite->second.z *255;

            for_draw_Cloud->points.push_back(tmp_point);
        }
        viewer_thr->Draw_cloud2viewer(for_draw_Cloud,"Base_CL_GRID",8);
    }
}

void Frontier::FrontierDetection(std::map<std::pair<int, int>, Point3f> _Cloud_Grid, std::map<std::pair<int, int>, std::pair<bool, Point2f> > Obst_grid,
                                 PointT robot_pos,float dim, std::vector<std::pair<PointT, bool> > &Frontier_points, float frontiers_dist,bool disp)
{

    float robot_diameter = 0.9;
    float max_r = 0.25;
    float frontier_around_search = 1.5*1.5;

    cv::Point2i _min(1000000,100000),_max(-100000,-100000);
    {
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = _Cloud_Grid.begin();
        int i,j;
        for(;ite != _Cloud_Grid.end();ite++)
        {
            i = ite->first.first;
            j = ite->first.second;
            _min.x = (_min.x > i)?(i):(_min.x);
            _min.y = (_min.y > j)?(j):(_min.y);

            _max.x = (_max.x < i)?(i):(_max.x);
            _max.y = (_max.y < j)?(j):(_max.y);
        }
    }

    //    Frontier_points.clear();

    {
        std::vector<std::pair<PointT, bool> > _Frontier_points;
        for(uint k=0;k<Frontier_points.size();k++)
        {
            if(Frontier_points[k].second == 1)
                _Frontier_points.push_back(Frontier_points[k]);
        }
        Frontier_points.clear();

        for(uint k=0;k<_Frontier_points.size();k++)
        {
            Frontier_points.push_back(_Frontier_points[k]);
        }
    }

    std::map<std::pair<int, int>, Point3f> frontier_grid_points;
    {
        float max_r_2 = max_r*max_r;
        std::map<std::pair<int,int>,cv::Point3f >::iterator ite0 = _Cloud_Grid.begin();
        for(;ite0 != _Cloud_Grid.end();ite0++)
        {
            if(ite0->second == cv::Point3f(0,0,0))
                //            if(ite0->second.x != 0 && ite0->second.y != 0 && ite0->second.z != 0)
            {
                int observed_counter = 0;
                int max_r_idx = 3;
                cv::Point2i robot_idx;
                robot_idx.x = ite0->first.first;
                robot_idx.y = ite0->first.second;

                int start_id_x = ((robot_idx.x-max_r_idx) < _min.x)?(_min.x):((robot_idx.x-max_r_idx));
                int end_id_x = ((robot_idx.x+max_r_idx) > _max.x)?(_max.x):((robot_idx.x+max_r_idx));

                int start_id_y = ((robot_idx.y-max_r_idx) < _min.y)?(_min.y):((robot_idx.y-max_r_idx));
                int end_id_y = ((robot_idx.y+max_r_idx) > _max.y)?(_max.y):((robot_idx.y+max_r_idx));


                for(int i=start_id_x;i<=end_id_x;i++)
                {
                    for(int j=start_id_y;j<=end_id_y;j++)
                    {
                        //                            cv::Point3f query_point;
                        //                            query_point.x = ((float)i + 0.5) * dim;
                        //                            query_point.y = robot_pos.y;
                        //                            query_point.z = ((float)j + 0.5) * dim;

                        //                            if(vec_mag_2((query_point - PCL2CV(robot_pos))) < max_r_2 )
                        {
                            if(Obst_grid[make_pair(i,j)].first == 1 || (_Cloud_Grid[make_pair(i,j)].x!=0 && _Cloud_Grid[make_pair(i,j)].y == 0 && _Cloud_Grid[make_pair(i,j)].z == 0) )
                            {
                                observed_counter = -100000;
                                j = end_id_y+1;
                                i = end_id_x+1;
                            }else if(_Cloud_Grid[make_pair(i,j)].x!=0 && _Cloud_Grid[make_pair(i,j)].y!=0 && _Cloud_Grid[make_pair(i,j)].z!=0)
                            {
                                if(abs(robot_idx.x - i) <= 1 &&  abs(robot_idx.y - j) <= 1)
                                    observed_counter++;
                            }
                        }
                    }
                }

                if(observed_counter >= 1)
                {
                    frontier_grid_points[ite0->first] = cv::Point3f(0,1,1);
                }
            }
        }
    }

    //    {
    //        pcl::PointCloud<PointXYZ>::Ptr for_dense_Cloud (new PointCloud<PointXYZ>());
    //        pcl::PointXYZ tmp_point2;
    //        std::map<std::pair<int,int>,cv::Point3f >::iterator ite = frontier_grid_points.begin();
    //        for(;ite != frontier_grid_points.end();ite++)
    //        {
    //            tmp_point2.x = ((float)ite->first.first + 0.5) * dim;
    //            tmp_point2.y = 0;
    //            tmp_point2.z = ((float)ite->first.second + 0.5) * dim;

    //            for_dense_Cloud->points.push_back(tmp_point2);
    //        }

    //        cloud_denser(for_dense_Cloud,for_dense_Cloud,dim*3,5,10);

    //        uint cccc = 0;
    //        int i,j;
    //        for(int c=0;c<for_dense_Cloud->points.size();c++)
    //        {
    //            i = (floor)((for_dense_Cloud->points[c].x)/dim);
    //            j = (floor)((for_dense_Cloud->points[c].z)/dim);

    //            if(frontier_grid_points[make_pair(i,j)] == cv::Point3f(0,0,0))
    //            {
    //                frontier_grid_points[make_pair(i,j)] = cv::Point3f(0,1,1);
    //                cccc++;
    //            }
    //        }

    //    }

    contour_str frontier_contour =  ContourExtractor(frontier_grid_points,cv::Point2i(5,5),6);

    cv::vector<Vec4i> candidate_hierarchy;
    vector<Point> candidate_contours;
    std::vector<float> bound_rect_length;
    std::vector<cv::Rect> bound_rect;

    for( int i = 0; i< frontier_contour.contours.size(); i++ )
    {
        cv::Rect tmp_rec = cv::boundingRect(frontier_contour.contours[i]);

        float rect_len = (tmp_rec.height >= tmp_rec.width)?(tmp_rec.height):(tmp_rec.width);
        rect_len *= (dim/frontier_contour.scale);

        bound_rect.push_back(tmp_rec);
        bound_rect_length.push_back(rect_len);

    }

    RNG rng(12345);
    cv::Mat drawing = cv::Mat::zeros( frontier_contour.IMG.size(), CV_8UC3 );

    for( int i = 0; i< frontier_contour.contours.size(); i++ )
    {
        if(bound_rect_length[i] > robot_diameter )
        {
            cv::Point3f mid_tmp_point;
            //            mid_tmp_point.z = ((bound_rect[i].x + bound_rect[i].width/2)/frontier_contour.scale) - frontier_contour.offset.y + frontier_contour._min.y;
            //            mid_tmp_point.z *= dim;
            //            mid_tmp_point.y = 0;
            //            mid_tmp_point.x = ((bound_rect[i].y + bound_rect[i].height/2)/frontier_contour.scale) - frontier_contour.offset.x + frontier_contour._min.x;
            //            mid_tmp_point.x *= dim;

            int sz = frontier_contour.contours[i].size();
            cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);
            for (int j = 0; j < data_pts.rows; j++)
            {
                data_pts.at<double>(j, 0) = (double)frontier_contour.contours[i][j].x;
                data_pts.at<double>(j, 1) = (double)frontier_contour.contours[i][j].y;
            }

            //Perform PCA analysis
            cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
            //Store the center of the object
            cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                                       static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

            mid_tmp_point.z = (((float)cntr.x)/frontier_contour.scale) - frontier_contour.offset.y + frontier_contour._min.y;
            mid_tmp_point.z *= dim;
            mid_tmp_point.y = 0;
            mid_tmp_point.x = (((float)cntr.y)/frontier_contour.scale) - frontier_contour.offset.x + frontier_contour._min.x;
            mid_tmp_point.x *= dim;

            bool observed_near_point = 0;
            for(uint k=0;k<Frontier_points.size();k++)
            {
                float _dist = vec_mag(( PCL2CV(Frontier_points[k].first) - mid_tmp_point ));
                if(_dist <= frontiers_dist )
                {
                    observed_near_point = 1;
                    k = Frontier_points.size();
                }

            }

            if(!observed_near_point)
            {
                float _dist2Robot = vec_mag(( PCL2CV(robot_pos) - mid_tmp_point ));

                if(_dist2Robot >= (frontiers_dist))
                    Frontier_points.push_back(make_pair(CV2PCL(mid_tmp_point),0));
            }

            candidate_hierarchy.push_back(frontier_contour.hierarchy[i]);
            for(uint k=0;k<frontier_contour.contours[i].size();k++)
                candidate_contours.push_back(frontier_contour.contours[i][k]);

            if(disp)
            {
                cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                cv::rectangle(drawing,bound_rect[i],color,2);
                cv::drawContours( drawing, frontier_contour.contours, i, color,1, 8, frontier_contour.hierarchy, 0, Point() );
            }

        }
    }


    {
        uint rejected_frontiers= 0;
        for(uint k=0;k<Frontier_points.size();k++)
        {
            if(Frontier_points[k].second == 0 && planning_thr->accessiblePoints.size()>5)
            {
                int observed_near_acc = 0;
                for(uint i=5;i<planning_thr->accessiblePoints.size();i++)
                {
                    PointT tmp_acc_point = planning_thr->accessiblePoints[i];
                    tmp_acc_point.y = Frontier_points[k].first.y;

                    float _dist = Point2PointDist_2(tmp_acc_point,Frontier_points[k].first);
                    if(_dist < frontier_around_search)
                    {
                        observed_near_acc++;
                    }
                }
                if(observed_near_acc > 2)
                {
                    Frontier_points[k].second = 1;
                    rejected_frontiers++;
                }
                else if(observed_near_acc == 1)
                {
                    float black=0,white=0;
                    std::map<std::pair<int,int>,cv::Point3f >::iterator ite0 = _Cloud_Grid.begin();
                    for(;ite0 != _Cloud_Grid.end();ite0++)
                    {
                        cv::Point3f Generated_POS;
                        Generated_POS.x = ((float)ite0->first.first + 0.5) * dim;
                        Generated_POS.y = 0;
                        Generated_POS.z = ((float)ite0->first.second + 0.5) * dim;

                        float _dist = vec_mag_2(( PCL2CV(Frontier_points[k].first) - Generated_POS ));
                        if(_dist <= frontier_around_search)
                        {
                            if(ite0->second.x != 0 && ite0->second.y != 0 && ite0->second.z != 0)
                                white = white+1;
                            else if(ite0->second == cv::Point3f(0,0,0))
                                black = black+1;
                        }
                    }
                    if( white >= (1.2*black))
                    {
                        rejected_frontiers++;
                        Frontier_points[k].second = 1;
                    }
                }
            }
        }
        qDebug()<<"Frontier_points:"<<Frontier_points.size()<<" rejected_frontiers:"<<rejected_frontiers;
    }

    if(disp)
        cv::imshow("frontiers",drawing);

    draw_occ_grid(frontier_grid_points,PointT(0,0.5,0),dim,"frontier_grid");
    viewer_thr->RemoveFromViewer(fr_shapes_name);
    fr_shapes_name.clear();
    for(uint k=0;k<Frontier_points.size();k++)
    {
        QByteArray sph_str = QString("fr_sph_"+QString::number(k)).toLatin1();
        fr_shapes_name.push_back(sph_str);
        if(Frontier_points[k].second == 0)
            viewer_thr->DrawSphere(Frontier_points[k].first,0.1,1,0,0,0,sph_str);
        else
            viewer_thr->DrawSphere(Frontier_points[k].first,0.12,0,1,1,1,sph_str);
    }

}

void Frontier::SliceOfCloud(_PointCloudT _cloud, float _y, float thr, _PointCloudT slicedCloud)
{
    float dist;
    PointT tmp_point;
    slicedCloud->points.clear();

    for(uint i=0;i<_cloud->points.size();i++)
    {
        dist = fabs(_cloud->points[i].y - _y);
        if(dist < thr)
        {
            tmp_point = _cloud->points[i];
            tmp_point.y = _y;

            slicedCloud->points.push_back(tmp_point);
        }
    }

    //    viewer_thr->Draw_cloud2viewer(slicedCloud,"Sliced_POINTS",255,255,255,10);
}

void Frontier::draw_occ_grid(std::map<std::pair<int, int>, Point3f> cloud, PointT offset, float dim, QByteArray _name)
{
    PointCloud<PointXYZRGBA>::Ptr for_draw_Cloud (new PointCloud<PointXYZRGBA>());
    PointXYZRGBA tmp_point;
    std::map<std::pair<int,int>,cv::Point3f >::iterator ite = cloud.begin();
    for(;ite != cloud.end();ite++)
    {
        tmp_point.x = offset.x + ((float)ite->first.first + 0.5) * dim;
        tmp_point.y = offset.y;
        tmp_point.z = offset.z + ((float)ite->first.second + 0.5) * dim;
        tmp_point.r = ite->second.x *255;
        tmp_point.g = ite->second.y *255;
        tmp_point.b = ite->second.z *255;

        for_draw_Cloud->points.push_back(tmp_point);
    }

    viewer_thr->Draw_cloud2viewer(for_draw_Cloud,_name,8);
}

void Frontier::Run_THR()
{
    MainTimer->start(loop_time);
}



