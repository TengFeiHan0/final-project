#include "planning.h"

#define MAX_GOAL_DIST           1.5
#define MIN_GOAL_DIST           1.0


Planning::Planning(QObject *parent) : QObject(parent)
{
    cloud = PointCloudT::Ptr (new PointCloudT);
    robot_aroud_cloud = PointCloudT::Ptr (new PointCloudT);
    Observed_cloud.second = PointCloudT::Ptr (new PointCloudT);
    PRM_PointCloud = PointCloudT::Ptr (new PointCloudT);

    loop_time = 30;
    Robot_POS = PointT(0,0,0);
    Floor_Ceil_Level = cv::Point2f(10000,-10000);
    busy_flag = 0;
    start_nav = -1;
    SetpointBlockCounter = 0;
    Navigation_Mode = NAV_HOLD_POSITION;
    Try_Other_Condidates = 0;
    Frontier_thr_run_counter = 0;
    Request_Frontier_Flag = 0;
    last_setpoint.second = 0;
    last_SetPointForPublish.second = 0;

    search_radius = MIN_SEARCH_R;
    search_setpoint = START_POINTS;

    rrt_planning.space.start_point = PCL2CV(Robot_POS);
    rrt_planning.space.goal_point = POINT(-0.5,-0.5,3);
    rrt_planning.space.Obs_radius = 0.3;
    rrt_planning.space.Robot_radius = 0.35;
    rrt_planning.space.Border_radius = 0.1;

    rrt_planning.RRT_Param.goal_THR = 0.2;
    rrt_planning.RRT_Param.Max_rrt_iteration = 10000;
    rrt_planning.RRT_Param.Max_smooth_iteration = 800;
    rrt_planning.RRT_Param.resulotion = 0.15;
    rrt_planning.RRT_Param.step_len = 0.3;
    rrt_planning.space.admissible_area.first = POINT(-2,-1.2,-1.2);
    rrt_planning.space.admissible_area.second = POINT(2,0,10);

    PRM_Planning.space.start_point = POINT(0,-0.5,0);
    PRM_Planning.space.Obs_radius = 0.3;
    PRM_Planning.space.Robot_radius = 0.4;
    PRM_Planning.space.Border_radius = 0.2;

    PRM_Planning.PRM_Param.resulotion = 0.15;
    PRM_Planning.PRM_Param.K_nearest = 5;
    PRM_Planning.PRM_Param.edge_min_dist = 3;
    PRM_Planning.PRM_Param.Min_Graph_Edge_len = 0.6;
    PRM_Planning.space.admissible_area.first = POINT(-10000,-10000,-10000);
    PRM_Planning.space.admissible_area.second = POINT(10000,10000,10000);

    PRM_Planning.rrt_planning.space = rrt_planning.space;
    PRM_Planning.rrt_planning.RRT_Param = rrt_planning.RRT_Param;

    SetpointPlanning.space.Obs_coeff = 0.004;
    SetpointPlanning.space.Obs_radius = 1;
    SetpointPlanning.space.Border_coeff = 0.008;
    SetpointPlanning.space.Border_radius = 1;
    SetpointPlanning.space.Start_coeff = 0.2;//0.002;
    SetpointPlanning.space.Start_radius = MIN_SETPOINT_DIST;
    SetpointPlanning.space.Goal_coeff = 0.2;
    SetpointPlanning.space.Goal_radius = MAX_SETPOINT_DIST;
    SetpointPlanning.space.Max_Force = 0.15;
    SetpointPlanning.space.admissible_area.first = POINT(-2,-4,-0.2);
    SetpointPlanning.space.admissible_area.second = POINT(2,0.3,10);

    RobotpointPlanning.space.Start_pos = PCL2CV(Robot_POS);
    RobotpointPlanning.space.Obs_coeff = 0.0008;
    RobotpointPlanning.space.Obs_radius = 1.0;
    RobotpointPlanning.space.Border_coeff = 0.0001;
    RobotpointPlanning.space.Border_radius = 1.2;
    RobotpointPlanning.space.Max_Force = 0.15;
    RobotpointPlanning.space.admissible_area.first = POINT(-2,-4,-0.2);
    RobotpointPlanning.space.admissible_area.second = POINT(2,0.3,10);
    potential_field_iteration = 0;

    MainTimer = new QTimer(this);

    Navigation_OutPut_Pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("ROS_EXP/OUTPUT", 10);

    connect(MainTimer,SIGNAL(timeout()),this,SLOT(MainTimerEvent()));
    connect(this,SIGNAL(thr_start_signal()),this,SLOT(Run_THR()));
    connect(this,SIGNAL(request_response_frontier_signal()),this,SLOT(request_response_frontier_slot()));

    motion_flag = 0;

}

Planning::~Planning()
{

}
void Planning::MainTimerEvent()
{
    //        qDebug()<<"Planning"<<QThread::currentThreadId();
    ltimer.start();

    if(listner_thr->cloud->points.size()>0)
    {
        busy_flag = 1;
        cloud->points.clear();
        cloud->points.resize(listner_thr->cloud->points.size());
        cloud->points = listner_thr->cloud->points;

        Robot_POS = listner_thr->Robot_POS;
        Robot_Heading = listner_thr->Robot_Heading;
        busy_flag = 0;
    }

    //    {
    //        accessiblePoints.clear();
    //        accessiblePoints.push_back(PointT(0.5,0,0));
    //        accessiblePoints.push_back(PointT(0.5,0,0.5));
    //        accessiblePoints.push_back(PointT(0,0,0.5));
    //        accessiblePoints.push_back(PointT(0.5,0,1));
    //        accessiblePoints.push_back(PointT(0.25,0,1));
    //        accessiblePoints.push_back(PointT(0,0,2));
    //        accessiblePoints.push_back(PointT(1,0,1.5));

    //        accessiblePoints.push_back(PointT(1,0,2.5));

    //        accessiblePoints.push_back(PointT(1.1,0,2.5));
    //        accessiblePoints.push_back(PointT(1,0,2.4));
    //        accessiblePoints.push_back(PointT(1,0,2.6));
    //        accessiblePoints.push_back(PointT(1,0.1,2.6));

    //        accessiblePoints.push_back(PointT(2,0,2.5));
    //        accessiblePoints.push_back(PointT(2,0,2.4));
    //        accessiblePoints.push_back(PointT(2,0,2.6));
    //        accessiblePoints.push_back(PointT(2,0.1,2.5));


    //        std::vector<PointT> goal_tmp_pos;
    //        goal_tmp_pos.push_back(PointT(2,-0.2,4.8));
    //        goal_tmp_pos.push_back(PointT(1.5,-0.2,4.2));

    //        findLongPath(cloud,PointT(0,-0.5,0),goal_tmp_pos,accessiblePoints,Robot_SetPoints,1);
    //        //Draw PRM Graph............................................................................................
    //        viewer_thr->RemoveFromViewer(shapes_name_3);
    //        shapes_name_3.clear();
    //        std::map<int,std::vector<uint> >::iterator my_ite = PRM_Planning.PRM_Graph.begin();
    //        for(;my_ite!=PRM_Planning.PRM_Graph.end();my_ite++)
    //        {
    //            for(uint i=0;i< my_ite->second.size();i++)
    //            {
    //                PointT P0 = CV2PCL(PRM_Planning.space.PRM_Points[my_ite->first]);
    //                PointT P1 = CV2PCL(PRM_Planning.space.PRM_Points[my_ite->second[i]]);
    //                QByteArray line_str = QString("PRM_Path"+QString::number(my_ite->first)+QString::number(my_ite->second[i])).toLatin1();
    //                viewer_thr->DrawLine(P0,P1,PointT(0,0.0,1.0),2,line_str);
    //                shapes_name_3.push_back(line_str);
    //            }
    //        }

    //        for(uint i=0;i<PRM_Planning.space.PRM_Points.size();i++)
    //        {
    //            float r = 1,g = 0;
    //            for(uint k=0;k<PRM_Planning.space.goal_point.size();k++)
    //            {
    //                if(PRM_Planning.space.PRM_Points[i] == PRM_Planning.space.goal_point[k])
    //                {
    //                    r=0;g=1;
    //                }
    //            }

    //            if(PRM_Planning.space.PRM_Points[i] == PRM_Planning.nearest_goal_point)
    //            {
    //                r=1;g=1;
    //            }
    //            QByteArray sph_str = QString("PRM_node"+QString::number(i)).toLatin1();
    //            viewer_thr->DrawSphere(CV2PCL(PRM_Planning.space.PRM_Points[i]),0.05,r,g,0,0,sph_str);
    //            shapes_name_3.push_back(sph_str);
    //        }

    //        viewer_thr->DrawSphere(CV2PCL(PRM_Planning.last_point),0.1,0,0,1,1,"last_prm");

    //    }

    if(start_nav > -1 && cloud->points.size()> START_POINTS/2)
    {
        select_aroud_points();
        if(robot_aroud_cloud->points.size() >= search_setpoint)
        {
            //            remove_outlier(robot_aroud_cloud,robot_aroud_cloud,5,0.5);

            PointCloudT::Ptr _robot_aroud_cloud (new PointCloudT());
            PointCloudT::Ptr  _wall_cloud (new PointCloudT());
            _robot_aroud_cloud->points.resize(robot_aroud_cloud->points.size());
            _robot_aroud_cloud->points = robot_aroud_cloud->points ;
            Define_area_OutPut = Define_area(0,0,5,_robot_aroud_cloud,_wall_cloud,PointT(1,1,1),DIM_DIVIDE_METHOD,PLANE_FIT_METHOD,LOCAL_MEAN_CORRECT_METHOD,0.6);


            //            Define_area_OutPut = Define_area(1,1,_robot_aroud_cloud,PointT(4,4,4),NUM_DIVIDE_METHOD,PLANE_FIT_METHOD,LOCAL_MEAN_CORRECT_METHOD,0.5);

            if( Define_area_OutPut.Plane_fit_dist !=0 )
            {
                store_corrected_map_point(_robot_aroud_cloud,Observed_cloud,Robot_POS,(0.6*search_radius));

                if(Frontier_thr_run_counter%4 == 0 && listner_thr->only_grab_map == 0)
                    viewer_thr->Draw_Visitedcloud2viewer(_robot_aroud_cloud,"Visited_Around_POINTS",0,255,100,6);
                if(listner_thr->only_grab_map >0)
                    viewer_thr->Draw_cloud2viewer(_robot_aroud_cloud,"Around_POINTS",0,255,100,6);
                //                viewer_thr->Draw_cloud2viewer(Observed_cloud.second,"OBS_POINTS",255,0,100,6);
                //                viewer_thr->Draw_cloud2viewer(robot_aroud_cloud,"R_Around_POINTS",255,255,255,7);


                if(Floor_Ceil_Level.x > Define_area_OutPut.Y_Area.x)
                    Floor_Ceil_Level.x = Define_area_OutPut.Y_Area.x;
                if(Floor_Ceil_Level.y < Define_area_OutPut.Y_Area.y)
                    Floor_Ceil_Level.y = Define_area_OutPut.Y_Area.y;

                Frontier::_Space_Robot_Property SRP_tmp;
                SRP_tmp.X_Area = Define_area_OutPut.X_Area;
                SRP_tmp.Y_Area = Define_area_OutPut.Y_Area;
                SRP_tmp.Robot_POS = Robot_POS;
                SRP_tmp.Robot_Heading = Robot_Heading;
                SRP_tmp.Grid_dim = 0.1;
                SRP_tmp.frontierDist = 0.5;
                SRP_tmp.DevisionNum = 4;
                SRP_tmp._floor = Define_area_OutPut.Y_Area.y - 0.7;
                SRP_tmp._ceil = Define_area_OutPut.Y_Area.x;
                SRP_tmp.Level_dist = 0.8;
                SRP_tmp.Max_Obst_height = 1.5;


                if(Frontier_thr_run_counter%4 == 0)
                    frontier_thr->CreatOccupancyGrid(robot_aroud_cloud,_robot_aroud_cloud,_wall_cloud,SRP_tmp);

                Frontier_thr_run_counter++;
                Navigation_Fcn(_robot_aroud_cloud,Observed_cloud.second);
            }
        }
        int passed_Time = ltimer.nsecsElapsed()/1000000.0;
        //        cout<<" S_C:"<<listner_thr->cloud_scale<<" el_Time::"<< passed_Time <<" msec"<<endl;

    }
}

void Planning::thr_start()
{
    emit thr_start_signal();
}

void Planning::select_aroud_points()
{
    robot_aroud_cloud->points.clear();
    SEG::search_Area(cloud,robot_aroud_cloud,Robot_POS,search_radius);

    if(robot_aroud_cloud->points.size() < search_setpoint)
        search_radius +=0.1;

    if(search_radius >= MAX_SEARCH_R)
    {
        search_radius = MAX_SEARCH_R;
        search_setpoint = robot_aroud_cloud->points.size();
    }
}

void Planning::Navigation_Fcn(PointCloudT::Ptr _cloud, PointCloudT::Ptr _cloud2)
{
    if(Navigation_Mode == NAV_DECISION_MAKING_MODE)
    {
        if(viewer_thr->Robot_Axes_For_View.size() > 1)
        {
            viewer_thr->Robot_Axes_For_View.clear();
            viewer_thr->Robot_Axes_For_View.resize(1);
            viewer_thr->Robot_Axes_For_View[0] = Robot_Heading;
        }

        if(Request_Frontier_Flag == REQUEST_FRONTIER_NON)
        {
            frontier_thr->FrontierDetermination();
            Request_Frontier_Flag = REQUEST_FRONTIER_WATING_FOR_RESPONSE;
        }
        else if(Request_Frontier_Flag == REQUEST_FRONTIER_RESPONSE_RECEIVED)
        {
            Request_Frontier_Flag = REQUEST_FRONTIER_NON;
            candidate_frontier.clear();
            select_best_frontier(frontier_thr->Frontier_points,Robot_POS,Robot_Heading,candidate_frontier);

            qDebug()<<"fr_si"<<frontier_thr->Frontier_points.size()<<"ca_si"<<candidate_frontier.size();
            if(candidate_frontier.size() > 0)
            {
                qDebug("Nav_Init");
                //                viewer_thr->DrawSphere(candidate_frontier.first.second,0.1,0,1,0,0,"candid_frontier");
                backwardSetPoint.second = 1;
                Navigation_Mode = NAV_INIT_MODE;
            }else
                Navigation_Mode = NAV_HOLD_POSITION;
        }
    }

    if(Navigation_Mode == NAV_INIT_MODE )
    {
        Robot_SetPoints.clear();
        Initiate_Plannings(_cloud,_cloud2);
        Navigation_Mode = NAV_SETPOINT_DETERMIN_MODE;
    }

    if(Navigation_Mode == NAV_SETPOINT_DETERMIN_MODE)
    {
        viewer_thr->RemoveFromViewer(shapes_name_3);
        shapes_name_3.clear();
        for(uint i=0;i<candidate_frontier.size();i++)
        {
            potential_field_iteration = 0;
            SetpointPlanning.space.Robot_pos = PCL2CV(candidate_frontier[i].first.second);
            QByteArray sph_str = QString("ROBOT_GOALPOINT_"+QString::number(i)).toLatin1();
            while(SetpointPlanning.RunPotentialPlanning(0.001,!candidate_frontier[i].second) == 1 && potential_field_iteration<500 )
            {
                potential_field_iteration++;
                //                boost::this_thread::sleep (boost::posix_time::microseconds (10000));
            }

            viewer_thr->DrawSphere(CV2PCL(SetpointPlanning.space.Robot_pos),0.08,1,0,1,0,sph_str);
            candidate_frontier[i].first.second = CV2PCL(SetpointPlanning.space.Robot_pos);

            shapes_name_3.push_back(sph_str);

            //            if(candidate_frontier[i].second == 0)
            //                i = candidate_frontier.size();
        }

        std::vector<std::pair<PointT,bool> > tmp_frontiers;
        for(uint i=0;i<candidate_frontier.size();i++)
        {
            tmp_frontiers.push_back(make_pair(candidate_frontier[i].first.second,0));
        }

        frontier_thr->CheckFrontierValidation(frontier_thr->Base_Cloud_Grid,tmp_frontiers,frontier_thr->Space_Property.Grid_dim);

        SetPointForNAV = candidate_frontier[0].first.second;
        if( fabs(SetPointForNAV.y - Robot_POS.y) < 1.5)
        {
            if(candidate_frontier[0].second == 0)
            {
                if(tmp_frontiers[0].second == 0)
                {
                    Navigation_Mode = NAV_LONG_PATH_TRAJECTORY_DETERMIN;
                }
                else
                {
                    Navigation_Mode = NAV_DECISION_MAKING_MODE;
                    frontier_thr->Frontier_points[candidate_frontier[0].first.first].second = 1;
                }
            }
            else
            {
                std::vector<std::pair<pair<int, PointT>,bool> >_candidate_frontier;
                for(uint i=0;i<candidate_frontier.size();i++)
                {
                    if(tmp_frontiers[i].second == 0)
                        _candidate_frontier.push_back(candidate_frontier[i]);
                    else
                        frontier_thr->Frontier_points[candidate_frontier[i].first.first].second = 1;
                }
                candidate_frontier.clear();
                candidate_frontier.resize(_candidate_frontier.size());
                candidate_frontier =  _candidate_frontier;
                if(candidate_frontier.size() > 0)
                {
                    candidate_frontier[0].second = 1;
                    Navigation_Mode = NAV_LONG_PATH_TRAJECTORY_DETERMIN;
                }
                else
                    Navigation_Mode = NAV_DECISION_MAKING_MODE;
            }
        }
        else
        {
            potential_field_iteration = 0;
            Robot_SetPoints.clear();
            SetpointBlockCounter++;
            Navigation_Mode = NAV_CORRECT_ROBOT_POS_MODE;
            qDebug()<<"----greater than 1.0 m----"<<fabs(SetPointForNAV.y - Robot_POS.y);
        }
    }


    if(Navigation_Mode == NAV_TRAJECTORY_DETERMIN)
    {
        rrt_planning.space.goal_point = PCL2CV(SetPointForNAV);
        rrt_planning.space.start_point = PCL2CV(Robot_POS);

        if(rrt_planning.PlanPathRRT() >-1)
        {
            SetpointBlockCounter = 0;

            if(VirtualRobotPOS.size()>0)
            {
                for(int i=1;i<rrt_planning.RRT_PATH.size();i++)
                    VirtualRobotPOS.push_back(rrt_planning.RRT_PATH[i]);

                rrt_planning.RRT_PATH.clear();
                rrt_planning.RRT_PATH.resize(VirtualRobotPOS.size());
                rrt_planning.RRT_PATH = VirtualRobotPOS;
                VirtualRobotPOS.clear();
            }

            rrt_planning.SmoothPath();
            rrt_planning.PathBreakPoint(5,0.2);
            for(int i=0;i<rrt_planning.BREAK_POINT.size();i++)
            {
                Robot_SetPoints.push_back(CV2PCL(rrt_planning.BREAK_POINT[i]));
            }
            BreakDownFarSetpoints(Robot_SetPoints);
            //                        buildAccessiblePoints(Robot_SetPoints,accessiblePoints);
            DrawPlanningOutPut();
            //            accessiblePoints.erase(accessiblePoints.end() - 1);
            Navigation_Mode = NAV_SEND_ROBOT_SETPOINT_MODE;
            nav_counter = 0;
        }else
        {
            SetpointBlockCounter++;
            Navigation_Mode = NAV_CORRECT_ROBOT_POS_MODE;
        }
    }

    if(Navigation_Mode == NAV_LONG_PATH_TRAJECTORY_DETERMIN)
    {
        PointT _robot_pos = Robot_POS;
        std::vector<PointT> SetPointForNAV_vec;
        if(last_setpoint.second == 1)
            _robot_pos = last_setpoint.first;
        for(uint i=0;i<candidate_frontier.size();i++)
        {
            SetPointForNAV_vec.push_back(candidate_frontier[i].first.second);
        }


        if(findLongPath(PRM_PointCloud,_robot_pos,SetPointForNAV_vec,accessiblePoints,Robot_SetPoints,0))
        {
            last_setpoint.second = 0;
            SetPointForNAV = CV2PCL(PRM_Planning.nearest_goal_point);
            viewer_thr->DrawSphere(SetPointForNAV,0.08,0.8,1,1,0,"_nearest_frontier");
            buildAccessiblePoints(Robot_SetPoints,accessiblePoints);
            BreakDownFarSetpoints(Robot_SetPoints);
            DrawPlanningOutPut();
            EndPathSetPointForPublish = Robot_SetPoints[Robot_SetPoints.size()-1];

            if(Robot_SetPoints.size() > 9 && Point2PointDist(EndPathSetPointForPublish,Robot_POS) > 2*MIN_SETPOINT_DIST )
                THR_For_generate_next_frontier = 2*MIN_SETPOINT_DIST;
            else if(Robot_SetPoints.size() > 5 && Point2PointDist(EndPathSetPointForPublish,Robot_POS) > 1.5*MIN_SETPOINT_DIST )
                THR_For_generate_next_frontier = 1.5*MIN_SETPOINT_DIST;
            else
                THR_For_generate_next_frontier = 0.7*MIN_SETPOINT_DIST;

            Navigation_Mode = NAV_SEND_ROBOT_SETPOINT_MODE;

        }else
        {
            std::map<int,std::vector<uint> >::iterator my_ite = PRM_Planning.PRM_Graph.begin();
            for(;my_ite!=PRM_Planning.PRM_Graph.end();my_ite++)
            {
                for(uint i=0;i< my_ite->second.size();i++)
                {
                    PointT P0 = CV2PCL(PRM_Planning.space.PRM_Points[my_ite->first]);
                    PointT P1 = CV2PCL(PRM_Planning.space.PRM_Points[my_ite->second[i]]);
                    QByteArray line_str = QString("_PRM_Path_"+QString::number(my_ite->first)+QString::number(my_ite->second[i])).toLatin1();
                    viewer_thr->DrawLine(P0,P1,PointT(0,0.0,1.0),2,line_str);
                    shapes_name_3.push_back(line_str);
                }
            }
            //            int frontier_idx = 0;
            //            for(uint i=0;i<candidate_frontier.size();i++)
            //            {
            //                if( PCL2CV(SetPointForNAV) == PCL2CV(candidate_frontier[i].first.second))
            //                    frontier_idx = i;
            //            }
            if(frontier_thr->Frontier_points.size() > 1)
                frontier_thr->Frontier_points[candidate_frontier[0].first.first].second = 1;
            qDebug("Cannot find long path");
            Navigation_Mode = NAV_DECISION_MAKING_MODE;
        }
    }

    if(Navigation_Mode == NAV_SEND_ROBOT_SETPOINT_MODE)
    {
        if(Robot_SetPoints.size() >0)
        {
            SetPointForPublish = Robot_SetPoints[0];
            viewer_thr->DrawSphere(SetPointForPublish,rrt_planning.space.Robot_radius*0.4,1,0,0,1,"ROBOT_Set_POINT");
            if(vec_mag((PCL2CV(Robot_POS) - PCL2CV(Last_Robot_POS))) > 10 )
            {
                Last_Robot_POS = Robot_POS;
                //                pub_setpint(Robot_POS,PUB_SEND_NAV_RE_CALIBRATION);
                qDebug("SEND CAMERA RE CALIBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
            }
            Navigation_Mode = NAV_ROBOT_ATTITUDE_EVAL_MODE;
        }
    }
    if(Navigation_Mode == NAV_ROBOT_ATTITUDE_EVAL_MODE)
    {
        cv::Point3f cv_tmp_vec;
        if(last_SetPointForPublish.second == 1)
            cv_tmp_vec = PCL2CV(SetPointForPublish) - PCL2CV(last_SetPointForPublish.first);
        else
            cv_tmp_vec = PCL2CV(SetPointForPublish) - PCL2CV(Robot_POS);

        cv_tmp_vec.y = 0;
        float dist = vec_mag(cv_tmp_vec);
        cv_tmp_vec = cv_tmp_vec*(1/vec_mag(cv_tmp_vec));

        PointT tmp_vec = CV2PCL(cv_tmp_vec);
        float attitude_var = RobotPointAngle(Robot_Heading,tmp_vec);

        cv_tmp_vec = PCL2CV(EndPathSetPointForPublish) - PCL2CV(Robot_POS);
        cv_tmp_vec.y = 0;
        cv_tmp_vec = cv_tmp_vec*(1/vec_mag(cv_tmp_vec));
        float End_attitude_var = RobotPointAngle(Robot_Heading,CV2PCL(cv_tmp_vec));

        if( fabs(attitude_var) > ATTITIDE_FLUCTUATION && fabs(End_attitude_var) > ATTITIDE_FLUCTUATION && dist > 0.5)
        {
            Yaw_setpoint = RobotPointAngle(PointT(0,0,1),tmp_vec);
            Robot_Heading_Setpoint = tmp_vec;

            if( fabs(attitude_var) > 150 && Robot_SetPoints.size() > 3 && backwardSetPoint.second == 1)
            {
                backwardSetPoint = FindMinRotationPoint(Robot_SetPoints);
            }
            dist = vec_mag(PCL2CV(backwardSetPoint.first) - PCL2CV(Robot_POS));

            if(backwardSetPoint.second == 0 && dist < SETPOINT_VICINITY_THR)
            {
                viewer_thr->RemoveFromViewer("BestAnglePoint");
                backwardSetPoint.second = 1;
                Navigation_Mode = NAV_SEND_YAW_SETPOINT_MODE;
            }
            else if(backwardSetPoint.second == 1)
                Navigation_Mode = NAV_SEND_YAW_SETPOINT_MODE;
            else
            {
                viewer_thr->DrawSphere(backwardSetPoint.first,rrt_planning.space.Robot_radius*0.5,0,1,0,0,"BestAnglePoint");
                Navigation_Mode = NAV_PUBLISH_ROBOT_SETPOINT_MODE;
            }

        }
        else
            Navigation_Mode = NAV_PUBLISH_ROBOT_SETPOINT_MODE;
    }
    if(Navigation_Mode == NAV_SEND_YAW_SETPOINT_MODE)
    {
        pub_setpint(Robot_Heading_Setpoint,PUB_SEND_NAV_YAW_SETPOINT);

        if(viewer_thr->Robot_Axes_For_View.size() == 1)
        {
            //            viewer_thr->Robot_Axes_For_View.push_back(PointT(sin(Yaw_setpoint*(3.14/180)),0,cos(Yaw_setpoint*(3.14/180))));
            viewer_thr->Robot_Axes_For_View.push_back(Robot_Heading_Setpoint);

        }

        //        float dist = fabs(Yaw_setpoint - listner_thr->Robot_ANGLE.z);
        float dist = acos(Vec2VecAngle(Robot_Heading_Setpoint,Robot_Heading))*(180/3.1415);
        if( dist <= 6.0)
        {
            if( nav_counter > (1000*HOLD_POS_TIME)/loop_time )
            {
                //                if(vec_mag((PCL2CV(EndPathSetPointForPublish) - PCL2CV(Robot_POS)))<(MAX_SETPOINT_DIST*0.8))
                //                    Navigation_Mode = NAV_DECISION_MAKING_MODE;
                //                else
                {
                    Navigation_Mode = NAV_PUBLISH_ROBOT_SETPOINT_MODE;
                }
                viewer_thr->RemoveFromViewer("ROBOT_GOALPOINT");
                viewer_thr->Robot_Axes_For_View.erase(viewer_thr->Robot_Axes_For_View.begin()+1);
                nav_counter = 0;
            }
            else
                nav_counter++;
        }
        else
            nav_counter = 0;
    }
    if(Navigation_Mode == NAV_PUBLISH_ROBOT_SETPOINT_MODE)
    {

        if(Point2PointDist(EndPathSetPointForPublish,Robot_POS) < THR_For_generate_next_frontier )
        {
            qDebug()<<"Lower MIN_DIST"<<Point2PointDist(EndPathSetPointForPublish,Robot_POS);
            //            int frontier_idx = 0;
            //            for(uint i=0;i<candidate_frontier.size();i++)
            //            {
            //                if( PCL2CV(SetPointForNAV) == PCL2CV(candidate_frontier[i].first.second))
            //                    frontier_idx = i;
            //            }
            //            frontier_thr->Frontier_points[candidate_frontier[frontier_idx].first.first].first = Robot_POS;
            //            frontier_thr->Frontier_points[candidate_frontier[frontier_idx].first.first].second = 1;
            //            if(Robot_SetPoints.size() < 2)
            //                pub_setpint(Robot_POS,PUB_SEND_NAV_POS_SETPOINT);
            last_setpoint = make_pair(SetPointForPublish,1);
            Navigation_Mode = NAV_DECISION_MAKING_MODE;
        }
        else
        {
            updateAccessiblePoints(Robot_POS,accessiblePoints);
            pub_setpint(SetPointForPublish,PUB_SEND_NAV_POS_SETPOINT);
            PointT dist_vec;
            dist_vec.x = SetPointForPublish.x - Robot_POS.x;
            dist_vec.y = SetPointForPublish.y - Robot_POS.y;
            dist_vec.z = SetPointForPublish.z - Robot_POS.z;

            float dist = vec_mag(dist_vec);

            if( dist < SETPOINT_VICINITY_THR)
            {
                if( nav_counter > (1000*HOLD_POS_TIME)/loop_time )
                {
                    last_SetPointForPublish.first = SetPointForPublish;
                    last_SetPointForPublish.second = 1;

                    if(Robot_SetPoints.size() > 0 )
                        Robot_SetPoints.erase(Robot_SetPoints.begin());
                    Navigation_Mode = NAV_SEND_ROBOT_SETPOINT_MODE;
                    nav_counter = 0;
                }
                else
                    nav_counter++;
            }
            else
                nav_counter = 0;

            if(Robot_SetPoints.size() == 0)
            {
                //                Navigation_Mode = NAV_DECISION_MAKING_MODE;
                //                int frontier_idx = 0;
                //                for(uint i=0;i<candidate_frontier.size();i++)
                //                {
                //                    if( PCL2CV(SetPointForNAV) == PCL2CV(candidate_frontier[i].first.second))
                //                        frontier_idx = i;
                //                }
                //                frontier_thr->Frontier_points[candidate_frontier[frontier_idx].first.first].first = SetPointForNAV;
                //                frontier_thr->Frontier_points[candidate_frontier[frontier_idx].first.first].second = 1;
                viewer_thr->RemoveFromViewer("candid_frontier");
                viewer_thr->RemoveFromViewer("ROBOT_Set_POINT");
            }
        }

    }

    if(Navigation_Mode == NAV_CORRECT_ROBOT_POS_MODE)
    {
        CorrectRobotPos();
    }

}

void Planning::Initiate_Plannings(PointCloudT::Ptr _cloud,PointCloudT::Ptr _cloud2)
{

    POINT area1,area2;
    PointCloudT::Ptr tmp_cloud (new PointCloudT());
    cv::Point2f X,Y,Z;

    tmp_cloud->points.clear();
    tmp_cloud->points.resize(_cloud->points.size());
    tmp_cloud->points = _cloud->points;

    PointT _tmp_point = Robot_POS;
    if(VirtualRobotPOS.size() > 0)
        Robot_POS = CV2PCL(VirtualRobotPOS[VirtualRobotPOS.size()-1]);
    if(last_setpoint.second == 1)
        Robot_POS = last_setpoint.first;


    X.x = (fabs(Robot_POS.x - Define_area_OutPut.X_Area.x) < 0.6)?(Robot_POS.x - 0.6):(Define_area_OutPut.X_Area.x);
    X.y = (fabs(Robot_POS.x - Define_area_OutPut.X_Area.y) < 0.6)?(Robot_POS.x + 0.6):(Define_area_OutPut.X_Area.y);

    Y.x = (fabs(Robot_POS.y - Define_area_OutPut.Y_Area.x) < 0.6)?(Robot_POS.y - 1):(Define_area_OutPut.Y_Area.x - 1);
    Y.y = (fabs(Robot_POS.y - Define_area_OutPut.Y_Area.y) < 0.6)?(Robot_POS.y + 0.6):(Define_area_OutPut.Y_Area.y + 1);

    Z.x = (fabs(Robot_POS.z - Define_area_OutPut.Z_Area.x) < 0.6)?(Robot_POS.z - 0.6):(Define_area_OutPut.Z_Area.x);
    Z.y = (fabs(Robot_POS.z - Define_area_OutPut.Z_Area.y) < 0.6)?(Robot_POS.z + 0.6):(Define_area_OutPut.Z_Area.y);


    area1 = POINT(X.x,Y.x,Z.x);
    area2 = POINT(X.y,Y.y,Z.y);
    PRM_Planning.space.Global_Floor_Ceil_Level = Floor_Ceil_Level;

    if(CheckInsidePoint(area1,area2,candidate_frontier[0].first.second) == 0 || Point2PointDist(candidate_frontier[0].first.second,Robot_POS) > (1.5*MAX_SETPOINT_DIST) )
    {
        area1 = POINT(-10000,Floor_Ceil_Level.x,-10000);
        area2 = POINT(10000,Floor_Ceil_Level.y,10000);

        tmp_cloud->points.clear();
        tmp_cloud->points.resize(_cloud2->points.size());
        tmp_cloud->points = _cloud2->points;

        PRM_PointCloud->points.clear();
        PRM_PointCloud->points.resize(_cloud2->points.size());
        PRM_PointCloud->points = _cloud2->points;

        candidate_frontier[0].second = 1;
        SetpointPlanning.space.Goal_coeff = 0;
        qDebug("out_range");
    }
    else
    {
        PRM_PointCloud->points.clear();
        PRM_PointCloud->points.resize(_cloud->points.size());
        PRM_PointCloud->points = _cloud->points;

        SetpointPlanning.space.Goal_coeff = 0.2;
        Y.x = (fabs(Robot_POS.y - Define_area_OutPut.Y_Area.x) < 0.6)?(Robot_POS.y - 1):(Define_area_OutPut.Y_Area.x - 1.5);
        Y.y = (fabs(Robot_POS.y - Define_area_OutPut.Y_Area.y) < 0.6)?(Robot_POS.y + 0.6):(Define_area_OutPut.Y_Area.y - 0.3);
        area1 = POINT(X.x,Y.x,Z.x);
        area2 = POINT(X.y,Y.y,Z.y);
    }



    if(accessiblePoints.size() == 0)
    {
        accessiblePoints.push_back(Robot_POS);
    }
    potential_field_iteration = 0;
    SetpointPlanning.space.admissible_area.first = area1;
    SetpointPlanning.space.admissible_area.second = area2;

    RobotpointPlanning.space.admissible_area.first = area1;
    RobotpointPlanning.space.admissible_area.second = area2;

    rrt_planning.space.admissible_area.first = area1;
    rrt_planning.space.admissible_area.second = area2;


    //    if(candidate_setpoint.second.x < MAX_GOAL_DIST && candidate_setpoint.second.x > MIN_GOAL_DIST)
    //        SetpointPlanning.space.Goal_radius = MIN_GOAL_DIST;
    //    else if(VirtualRobotPOS.size() > 0)
    //        SetpointPlanning.space.Goal_radius = 0.5*(MIN_GOAL_DIST+MAX_GOAL_DIST);
    //    else
    //        SetpointPlanning.space.Goal_radius = MAX_GOAL_DIST;



    SetpointPlanning.space.Start_pos = PCL2CV(Robot_POS);
    SetpointPlanning.space.Goal_pos = PCL2CV(Robot_POS);
    SetpointPlanning.space.Robot_pos = PCL2CV(candidate_frontier[0].first.second);

    Robot_POS = _tmp_point;
    rrt_planning.space.Obs_pos.clear();
    SetpointPlanning.space.Obs_pos.clear();
    RobotpointPlanning.space.Obs_pos.clear();
    for(int i=0;i<tmp_cloud->points.size();i++)
    {
        rrt_planning.space.Obs_pos.push_back(PCL2CV(tmp_cloud->points[i]));
        SetpointPlanning.space.Obs_pos.push_back(PCL2CV(tmp_cloud->points[i]));
        RobotpointPlanning.space.Obs_pos.push_back(PCL2CV(tmp_cloud->points[i]));
    }
}

void Planning::DrawPlanningOutPut()
{
    viewer_thr->RemoveFromViewer(shapes_name);
    shapes_name.clear();
    //    int nav_time = (int)(ltimer3.nsecsElapsed()/1000000);
    //    qDebug()<<"PATH"<<rrt_planning.RRT_PATH.size()<<"SMOOTH"<<rrt_planning.SMOOTH_PATH.size()
    //           <<"Time"<<nav_time<<"ms"<<"Iter"<<nav_iterate;

    //    viewer_thr->DrawSphere(CV2PCL(rrt_planning.space.start_point),rrt_planning.space.Robot_radius,0.0,1,0,1,"start_point");
    //    viewer_thr->DrawSphere(CV2PCL(rrt_planning.space.goal_point),rrt_planning.space.Robot_radius,1,0,0,1,"goal_point");

    for(int i=1;i<Robot_SetPoints.size();i++)
    {
        QByteArray line_str = QString("SMOOTH_Path"+QString::number(i)).toLatin1();
        viewer_thr->DrawLine(Robot_SetPoints[i-1],Robot_SetPoints[i],PointT(0.8,0.0,1.0),3,line_str);
        shapes_name.push_back(line_str);
    }

    for(int i=0;i<accessiblePoints.size();i++)
    {
        QByteArray line_str = QString("acc_points"+QString::number(i)).toLatin1();
        //        viewer_thr->DrawLine(accessiblePoints[i-1],accessiblePoints[i],PointT(0.0,0.9,1.0),2,line_str);
        viewer_thr->DrawSphere(accessiblePoints[i],rrt_planning.space.Robot_radius*0.3,0.3,1,0.7,1,line_str);
        shapes_name.push_back(line_str);
    }

    //    for(int i=0;i<rrt_planning.BREAK_POINT.size();i++)
    //    {
    //        QByteArray line_str = QString("BREAK_Point"+QString::number(i)).toLatin1();
    //        viewer_thr->DrawSphere(CV2PCL(rrt_planning.BREAK_POINT[i]),rrt_planning.space.Robot_radius*0.3,0.3,1,0.7,0,line_str);
    //        //        viewer_thr->DrawSphere(accessiblePoints[i],rrt_planning.space.Robot_radius*0.3,0.3,1,0.7,0,line_str);
    //        shapes_name.push_back(line_str);

    //    }
}

void Planning::CorrectRobotPos()
{
    //    if(!motion_flag)
    {
        if(SetpointBlockCounter == 1)
        {
            RobotpointPlanning.space.Robot_pos = PCL2CV(Robot_POS);
            VirtualRobotPOS.clear();
            //        VirtualRobotPOS.push_back(RobotpointPlanning.space.Robot_pos);
        }
        else
            RobotpointPlanning.space.Robot_pos = VirtualRobotPOS[VirtualRobotPOS.size()-1];

        //        RobotpointPlanning.space.admissible_area.first.z = PCL2CV(Robot_POS).z - 1.5;
        motion_flag = 1;
    }

    while(RobotpointPlanning.RunPotentialPlanning(0.001,!candidate_frontier[0].second));
    //        viewer_thr->DrawSphere(CV2PCL(RobotpointPlanning.space.Robot_pos),0.1,1,0,0,0,"Robot_point_planning");
    //    else
    {
        motion_flag = 0;
        //    Robot_SetPoints.push_back(CV2PCL(RobotpointPlanning.space.Robot_pos));
        VirtualRobotPOS.push_back(RobotpointPlanning.space.Robot_pos);

        if(SetpointBlockCounter <= 2)
            Navigation_Mode = NAV_DECISION_MAKING_MODE;
        //        Navigation_Mode = NAV_SEND_ROBOT_SETPOINT_MODE;
        //    else if(SetpointBlockCounter == 3)
        //        Navigation_Mode = NAV_INIT_MODE;
        else
        {
            //            frontier_thr->Frontier_points[candidate_frontier[0].first.first].first = SetPointForNAV;
            //            frontier_thr->Frontier_points[candidate_frontier[0].first.first].second = 1;
            Navigation_Mode = NAV_DECISION_MAKING_MODE;
            SetpointBlockCounter = 0;
            VirtualRobotPOS.clear();
        }
    }
}

void Planning::pub_setpint(PointT _point,float _type)
{
    //Publish OutPut to the GUI program..............
    pcl::PointCloud<pcl::PointXYZ> pub_cloud;
    pub_cloud.push_back(PointT(_type,_type,_type));
    pub_cloud.push_back(_point);
    pub_cloud.push_back(PointT(2.0,0,0));
    Navigation_OutPut_Pub.publish(pub_cloud);
}

void Planning::compute_PA_cloud(PointCloudT::Ptr _cloud)
{
    int sz = _cloud->points.size();
    cv::Mat data_pts = cv::Mat(sz, 3, CV_64FC1);
    for (int i = 0; i < sz; i++)
    {
        data_pts.at<double>(i, 0) = (double)(_cloud->points[i].x);
        data_pts.at<double>(i, 1) = 0;//(double)(robot_aroud_cloud->points[i].y);
        data_pts.at<double>(i, 2) = (double)(_cloud->points[i].z);
    }

    //Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize(10);
    std::vector<Point3d> eigen_vecs(3);
    for(int i=1;i<2;i++)
    {
        QByteArray cyl_str = QString("CL_PA_"+QString::number(i)).toLatin1();
        eigen_vecs[i] = Point3d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1),
                                pca_analysis.eigenvectors.at<double>(i, 2));

        cylinder_coeff.values[3] = (float)(eigen_vecs[i].x);
        cylinder_coeff.values[4] = (float)(eigen_vecs[i].y);
        cylinder_coeff.values[5] = (float)(eigen_vecs[i].z);
        cylinder_coeff.values[0] = Robot_POS.x;
        cylinder_coeff.values[1] = Robot_POS.y;
        cylinder_coeff.values[2] = Robot_POS.z;
        cylinder_coeff.values[6] = 0.02;
        cylinder_coeff.values[7] = (i*50+100)/255.;
        cylinder_coeff.values[8] = (i*60+150)/255.;
        cylinder_coeff.values[9] = (i*70+90)/255.;
        //                shapes_name_2.push_back(cyl_str);
        viewer_thr->DrawCylinder(cylinder_coeff,cyl_str);
    }

}

void Planning::find_farest_point(PointCloudT::Ptr _cloud,PointT robot,PointT heading,float thr,bool disp)
{
    //    float att0 = cos(ATTITIDE_FLUCTUATION*(3.14/180));
    //    float att1 = cos(90*(3.14/180));
    //    float att2 = cos(110*(3.14/180));

    float att0 = ATTITIDE_FLUCTUATION;
    float att1 = 30;
    float att2 = 100;
    float att3 = -100;

    std::vector<std::pair<PointT,cv::Point2f> > _candidate_points(3,make_pair(PointT(0,0,0),0));
    float dist = 0;
    float attitude = 0;
    PointT tmp_vec;
    for(uint i=0;i<_cloud->points.size();i++)
    {
        tmp_vec.x = _cloud->points[i].x - robot.x;
        tmp_vec.y = 0;//robot.y - _cloud->points[i].y;
        tmp_vec.z = _cloud->points[i].z - robot.z;
        dist = vec_mag(tmp_vec);

        if(dist > thr/2)
        {
            attitude = RobotPointAngle(heading,tmp_vec);

            if(fabs(attitude) <= att0)
            {
                if(_candidate_points[0].second.x < dist)
                    _candidate_points[0] = make_pair(_cloud->points[i],cv::Point2f(dist,attitude));
            }
            else if(attitude <= att2 && attitude > att1)
            {
                if(_candidate_points[1].second.x < dist)
                    _candidate_points[1] = make_pair(_cloud->points[i],cv::Point2f(dist,attitude));
            }else if(attitude >= att3 && attitude < -att1)
            {
                if(_candidate_points[2].second.x < dist)
                    _candidate_points[2] = make_pair(_cloud->points[i],cv::Point2f(dist,attitude));
            }
        }
    }

    for(uint i=0;i<3;i++)
    {
        if(_candidate_points[i].second.x != 0)
        {
            _candidate_points[i].first.y = robot.y;

            _candidate_points[i].first = CV2PCL(0.2*PCL2CV(robot) +0.80*PCL2CV(_candidate_points[i].first));
            //            _candidate_points[i].second.y = acos(_candidate_points[i].second.y)*180/3.14;
            if(disp)
            {
                float r = (float)i*0.3+0.3;
                QByteArray sp_str = QString("far_points"+QString::number(i)).toLatin1();
                viewer_thr->DrawSphere(_candidate_points[i].first,0.1,r,0,0.5,0,sp_str);
            }
        }
    }
    candidate_points = _candidate_points;
}

void Planning::select_best_frontier(std::vector<std::pair<PointT,bool> > &Frontier_points, PointT robot_pos, PointT heading, std::vector<std::pair<pair<int, PointT>, bool> > &out_point)
{
    cv::Point3f robot_point = PCL2CV(robot_pos);
    std::pair<int,cv::Point2f> frontiers_Min_Range(make_pair(-1,cv::Point2f(0,1000)));//-->(dits,angle)
    std::pair<int,cv::Point2f> frontiers_Min_dist(make_pair(-1,cv::Point2f(1000,1000)));
    std::vector<std::pair<double,int> > cost_vec;
    std::vector<std::pair<double,int> > low_cost_vec;

    for(uint i=0;i<Frontier_points.size();i++)
    {
        if(Frontier_points[i].second == 0)
        {
            float angle = fabs(RobotPointAngle(PCL2CV(heading),(PCL2CV(Frontier_points[i].first)-robot_point)));
            float dist = vec_mag((PCL2CV(Frontier_points[i].first)-robot_point));

            if( fabs(angle) <= fabs(frontiers_Min_Range.second.y))
                frontiers_Min_Range = make_pair(i,cv::Point2f(dist,angle));

            double exp_arg = 3*(pow((angle*(3.1415/180.0)),3) - 1);
            double cost = exp(exp_arg)*dist;
            qDebug()<< angle << dist << cost;
            if(dist > 1)
                cost_vec.push_back(make_pair(cost,i));
            else
                low_cost_vec.push_back(make_pair(cost,i));
            //            if(dist <= frontiers_Min_dist.second.x)
            //                frontiers_Min_dist = make_pair(i,cv::Point2f(dist,angle));
        }
    }

    out_point.clear();

    if(cost_vec.size() > 0)
    {
        std::sort(cost_vec.begin(),cost_vec.end(),double_compare_min);
        PointT _tmp_point;
        std::pair<int, PointT> tmp_pair;


        _tmp_point = Frontier_points[cost_vec[0].second].first;
        _tmp_point.y = (Define_area_OutPut.Y_Area.y + Define_area_OutPut.Y_Area.x)/2 + 0.3;
//        if(fabs(Define_area_OutPut.Y_Area.y-Define_area_OutPut.Y_Area.x) < 3)
//            _tmp_point.y = (Define_area_OutPut.Y_Area.y + Define_area_OutPut.Y_Area.x)/3 + 0.4;
//        else
//        {
//            _tmp_point.y = (Define_area_OutPut.Y_Area.y + Define_area_OutPut.Y_Area.x + Robot_POS.y)/3 + 0.6;
//            qDebug()<<"floor"<<Define_area_OutPut.Y_Area.x<<Define_area_OutPut.Y_Area.y;
//        }
        tmp_pair = make_pair(cost_vec[0].second,_tmp_point);
        out_point.push_back(make_pair(tmp_pair,0));
    }
    else if(low_cost_vec.size() > 0)
    {
        std::sort(low_cost_vec.begin(),low_cost_vec.end(),double_compare_min);
        PointT _tmp_point;
        std::pair<int, PointT> tmp_pair;


        _tmp_point = Frontier_points[low_cost_vec[low_cost_vec.size()-1].second].first;
        _tmp_point.y = (Define_area_OutPut.Y_Area.y + Define_area_OutPut.Y_Area.x)/2 + 0.3;
//        if(fabs(Floor_Ceil_Level.y-Floor_Ceil_Level.x) < 2.8)
//            _tmp_point.y = (Floor_Ceil_Level.y + Floor_Ceil_Level.x + Robot_POS.y)/3 + 0.4;
//        else
//            _tmp_point.y = (Floor_Ceil_Level.y + Floor_Ceil_Level.x + Robot_POS.y)/3 + 0.8;        tmp_pair = make_pair(low_cost_vec[low_cost_vec.size()-1].second,_tmp_point);
        out_point.push_back(make_pair(tmp_pair,0));
    }


    //    if(fabs(frontiers_Min_Range.second.y) <= ROBOT_FIELD_OF_VIEW && frontiers_Min_Range.second.x < search_radius)
    //    {
    //        _tmp_point = Frontier_points[frontiers_Min_Range.first].first;
    //        _tmp_point.y = (Floor_Ceil_Level.y + Floor_Ceil_Level.x + Robot_POS.y)/3;
    //        tmp_pair = make_pair(frontiers_Min_Range.first,_tmp_point);
    //        out_point.push_back(make_pair(tmp_pair,0));
    //    }
    //    else
    //    {
    //        for(uint i=0;i<dist_vec.size();i++)
    //        {
    //            _tmp_point = Frontier_points[dist_vec[i].second].first;
    //            _tmp_point.y = (Floor_Ceil_Level.y + Floor_Ceil_Level.x + Robot_POS.y)/3;
    //            qDebug()<<"ssssssetttttt"<<_tmp_point.y;
    //            tmp_pair = make_pair(dist_vec[i].second,_tmp_point);
    //            out_point.push_back(make_pair(tmp_pair,0));
    //        }
    //    }
}

bool Planning::check_setpoint_validation(PointCloudT::Ptr _cloud,std::vector<PointT> _accessiblePoits,PointT new_point,float max_dist,bool disp)
{
    float collosion_dist = 0.4;
    bool Consist2AccessiblePoints = 0;
    //    std::vector<PointT> path_points;
    //    path_points.push_back(PointT(1,0,0));
    //    path_points.push_back(PointT(1,0,1));
    //    path_points.push_back(PointT(1.5,0,2));
    //    path_points.push_back(PointT(1,-0.5,3));
    //    path_points.push_back(PointT(1,-0.5,4));

    //    std::vector<PointT> _accessiblePoits;

    //    validate_tmp_counter = (validate_tmp_counter>path_points.size())?(path_points.size()):(validate_tmp_counter);

    //    for(int i=1;i<validate_tmp_counter;i++)
    //    {
    //        QByteArray line_str = QString("validate_Path"+QString::number(i)).toLatin1();
    //        viewer_thr->DrawLine(path_points[i-1],path_points[i],PointT(0.0,0.9,1.0),2,line_str);
    //    }

    //    for(uint i=0;i<validate_tmp_counter;i++)
    //    {
    //        _accessiblePoits.push_back(path_points[i]);
    //    }

    //    if(validate_tmp_counter>0)
    //        viewer_thr->DrawSphere(path_points[validate_tmp_counter-1],rrt_planning.space.Robot_radius*0.3,0.3,1,0.7,0,"validate_sph");

    if(_accessiblePoits.size()>1)
    {
        std::vector<PointT> nearest_points(2);
        std::vector<std::pair<float,int> >Path2RobotDist;
        float min_dist  = 100000000;

        for(uint i=1;i<_accessiblePoits.size();i++)
        {
            PointT M = Point2LineReflection(_accessiblePoits[i-1],_accessiblePoits[i],new_point);
            float dist_vec1 = vec_mag(PCL2CV(M) - PCL2CV(_accessiblePoits[i-1]));
            float dist_vec2 = vec_mag(PCL2CV(_accessiblePoits[i]) - PCL2CV(M));
            float dist = vec_mag(PCL2CV(_accessiblePoits[i]) - PCL2CV(_accessiblePoits[i-1]));
            float distToLine = vec_mag(PCL2CV(new_point) - PCL2CV(M));
            float disttoPoint1 = vec_mag(PCL2CV(new_point) - PCL2CV(_accessiblePoits[i-1]));
            float disttoPoint2 = vec_mag(PCL2CV(new_point) - PCL2CV(_accessiblePoits[i]));

            if(distToLine <= max_dist)
            {

                if(dist_vec1 <= dist && dist_vec2 <= dist)
                {
                    if(distToLine < min_dist)
                    {
                        min_dist = distToLine;
                        nearest_points[0] = _accessiblePoits[i];
                        nearest_points[1] = _accessiblePoits[i-1];
                        Consist2AccessiblePoints = 1;
                    }
                }else if( (disttoPoint1 <= max_dist || disttoPoint2 <= max_dist) && !Consist2AccessiblePoints)
                {
                    if((i-1)!=0 && i != (_accessiblePoits.size()-1))
                    {
                        nearest_points[0] = _accessiblePoits[i-1];
                        nearest_points[1] = _accessiblePoits[i];
                        Consist2AccessiblePoints = 1;
                    }
                }
            }
        }

        if(Consist2AccessiblePoints)
        {
            for(uint i=0;i<nearest_points.size();i++)
            {
                cv::Point3f P0 = PCL2CV(new_point);
                cv::Point3f P1 = PCL2CV(nearest_points[i]);
                float dist = vec_mag((P0 - P1));
                float m = ceil(dist/0.1);
                for(uint j=1;j<m;j++)
                {
                    //        Robot_last_pos = Robot_pos;
                    cv::Point3f _Robot_pos = ( 1-((float)j/m) )*P0 + ((float)j/m)*P1;

                    for(uint k=0;k<_cloud->points.size();k++)
                    {
                        float dist = vec_mag((_Robot_pos-PCL2CV(cloud->points[k])));
                        if(dist < collosion_dist)
                        {
                            Consist2AccessiblePoints = 0;
                            k = _cloud->points.size();
                        }
                    }
                    if(!Consist2AccessiblePoints)
                        j = m;
                }
                if(Consist2AccessiblePoints)
                    i = nearest_points.size();
            }

        }

        if(disp)
        {
            if(Consist2AccessiblePoints)
            {
                for(uint i=0;i<nearest_points.size();i++)
                {
                    QByteArray shape_str = QString("nearest_point"+QString::number(i)).toLatin1();
                    viewer_thr->DrawSphere(nearest_points[i],rrt_planning.space.Robot_radius*0.3,1,0.5,0.7,0,shape_str);
                }
            }
            else
            {
                for(uint i=0;i<nearest_points.size();i++)
                {
                    QByteArray shape_str = QString("nearest_point"+QString::number(i)).toLatin1();
                    viewer_thr->RemoveFromViewer(shape_str);
                }
            }
        }
    }

    return (!Consist2AccessiblePoints);
}

void Planning::store_corrected_map_point(PointCloudT::Ptr _cloud,std::pair<std::vector<PointT>,PointCloudT::Ptr> &out_cloud,PointT robot_pos,float search_R)
{
    if(_cloud->points.size() > 1500 && out_cloud.first.size() == 0)
    {
        for(uint k=0;k<_cloud->points.size();k++)
            out_cloud.second->points.push_back(_cloud->points[k]);
        out_cloud.first.push_back(robot_pos);
    }

    if(out_cloud.first.size() > 0)
    {
        bool near_point_observ = 0;
        for(uint i=0;i<out_cloud.first.size();i++)
        {
            float dist = vec_mag( (PCL2CV(robot_pos) - PCL2CV(out_cloud.first[i])) );

            if(dist <= search_R)
            {
                near_point_observ = 1;
                i = out_cloud.first.size();
            }
        }
        if(!near_point_observ)
        {
            for(uint k=0;k<_cloud->points.size();k++)
                out_cloud.second->points.push_back(_cloud->points[k]);

            out_cloud.first.push_back(robot_pos);
        }
    }
}

bool Planning::findLongPath(PointCloudT::Ptr _cloud, PointT robot_pos, std::vector<PointT> goal_pos, std::vector<PointT> &_accessiblePoints, vector<PointT>& Out_setpoints, bool disp)
{
    bool start_find = 0;
    std::vector<bool> goal_find(goal_pos.size(),0);
    cv::Point3f cv_tmp_point;
    PRM_Planning.space.start_point = PCL2CV(robot_pos);
    PRM_Planning.space.goal_point.clear();
    for(uint i=0;i<goal_pos.size();i++)
        PRM_Planning.space.goal_point.push_back(PCL2CV(goal_pos[i]));

    PRM_Planning.space.Obs_pos.clear();
    PRM_Planning.space.PRM_Points.clear();
    for(int i=0;i<_cloud->points.size();i++)
    {
        PRM_Planning.space.Obs_pos.push_back(PCL2CV(_cloud->points[i]));
    }
    for(uint i=0;i<_accessiblePoints.size();i++)
    {
        cv_tmp_point = PCL2CV(_accessiblePoints[i]);
        PRM_Planning.space.PRM_Points.push_back(cv_tmp_point);

        if( cv_tmp_point == PCL2CV(robot_pos))
            start_find = 1;

        for(uint j=0;j<goal_pos.size();j++)
        {
            if(cv_tmp_point == PCL2CV(goal_pos[j]))
                goal_find[j] = 1;
        }
    }

    if(PRM_Planning.GetOptimizePath())
    {
        //        if(!start_find)
        //            _accessiblePoints.push_back(robot_pos);
        //        for(uint i=0;i< PRM_Planning.Inserted_PRM_Points.size();i++)
        //            _accessiblePoints.push_back(CV2PCL(PRM_Planning.Inserted_PRM_Points[i]));

        //        for(uint j=0;j<goal_pos.size();j++)
        //        {
        //            if(PCL2CV(goal_pos[j]) == PRM_Planning.nearest_goal_point)
        //            {
        //                if(goal_find[j] == 0)
        //                    _accessiblePoints.push_back(goal_pos[j]);
        //            }
        //        }

        for(int i=(PRM_Planning.OptimizePath.size()-1);i >=0;i--)
        {
            PointT P0 = CV2PCL(PRM_Planning.OptimizePath[i]);
            Out_setpoints.push_back(P0);
        }

        if(disp)
        {
            for(uint i=1;i< PRM_Planning.OptimizePath.size();i++)
            {
                PointT P0 = CV2PCL(PRM_Planning.OptimizePath[i-1]);
                PointT P1 = CV2PCL(PRM_Planning.OptimizePath[i]);
                QByteArray line_str = QString("Op_PRM_Path"+QString::number(i)).toLatin1();
                viewer_thr->DrawLine(P0,P1,PointT(1,0,1),4,line_str);

            }
        }
        return 1;
    }

    return 0;


}

bool Planning::CheckInsidePoint(POINT A0, POINT A1, PointT P_test)
{
    if(P_test.x >A0.x && P_test.y >A0.y  && P_test.z >A0.z)
    {
        if(P_test.x <A1.x && P_test.y <A1.y  && P_test.z <A1.z)
        {
            return 1;
        }
        else
            return 0;
    }
    else
        return 0;
}

void Planning::BreakDownFarSetpoints(std::vector<PointT> &Setpoints)
{
    cv::Point3f _P0,_P1,_new_point;
    std::vector<PointT> ModifiedSetpoints;
    for(uint i=1;i<Setpoints.size();i++)
    {
        ModifiedSetpoints.push_back(Setpoints[i-1]);
        _P0 = PCL2CV(Setpoints[i-1]);
        _P1 = PCL2CV(Setpoints[i]);
        float dist = vec_mag((_P0-_P1));
        float m = ceil(dist/MAX_ACCESSIBLE_EDGE_LEN);
        for(uint j=1;j<m;j++)
        {
            _new_point = ( 1-((float)j/m) )*_P0 + ((float)j/m)*_P1;
            ModifiedSetpoints.push_back(CV2PCL(_new_point));
        }

    }
    ModifiedSetpoints.push_back(Setpoints[Setpoints.size()-1]);
    Setpoints.clear();
    Setpoints.resize(ModifiedSetpoints.size());
    Setpoints = ModifiedSetpoints;
}

void Planning::buildAccessiblePoints(std::vector<PointT> Robot_SetPoints, std::vector<PointT> &accessiblePoints)
{
    if(accessiblePoints.size() > 0)
    {
        POINT _P0,_P1;
        for(int i=1;i<Robot_SetPoints.size();i++)
        {
            _P0 = PCL2CV(Robot_SetPoints[i-1]);
            _P1 = PCL2CV(Robot_SetPoints[i]);
            cv::Point3f cv_tmp_point=  _P1-_P0 ;
            float dist = vec_mag(cv_tmp_point);
            //            if(dist < MAX_ACCESSIBLE_EDGE_LEN)
            //                accessiblePoints.push_back(Robot_SetPoints[i]);
            //            else
            {
                POINT _new_point;
                float thr_2 = (MAX_ACCESSIBLE_EDGE_LEN*MAX_ACCESSIBLE_EDGE_LEN)*(0.8*0.8);
                float m = ceil(dist/MAX_ACCESSIBLE_EDGE_LEN);
                for(uint j=0;j<=m;j++)
                {
                    _new_point = ( 1-((float)j/m) )*_P0 + ((float)j/m)*_P1;
                    float dist_2;
                    bool observe_near = 0;
                    for(uint k=0;k<accessiblePoints.size();k++)
                    {
                        dist_2 = vec_mag_2((PCL2CV(accessiblePoints[k]) - _new_point)) ;
                        if(dist_2<thr_2)
                        {
                            observe_near = 1;
                            k = accessiblePoints.size();
                        }

                    }
                    if(!observe_near)
                        accessiblePoints.push_back(CV2PCL(_new_point));
                }
            }
        }

    }
    else
        accessiblePoints.push_back(Robot_POS);
}

void Planning::updateAccessiblePoints(PointT _robot_pos, std::vector<PointT> &accessiblePoints)
{
    float thr = 0.8;
    float thr_2 = thr*thr;
    float observed_near = 0;
    PointT nearest_point;
    float min_dist = 1000000;
    float dist_2;
    cv::Point3f cv_robot_pos = PCL2CV(_robot_pos);

    if(accessiblePoints.size() == 0)
        accessiblePoints.push_back(_robot_pos);

    for(uint i=0;i<accessiblePoints.size();i++)
    {
        dist_2 = vec_mag_2( (PCL2CV(accessiblePoints[i]) - cv_robot_pos) );

        if(dist_2 < thr_2)
            observed_near = 1;
        else if(dist_2 < min_dist)
        {
            min_dist = dist_2;
            nearest_point = accessiblePoints[i];
        }
    }

    if(!observed_near)
        accessiblePoints.push_back(_robot_pos);
}

std::pair<PointT,bool> Planning::FindMinRotationPoint(std::vector<PointT> Setpoints)
{

    std::vector<std::pair<float,int> > low_cost_vec;
    for(uint i=0;i<Setpoints.size()-1;i++)
    {
        PointT tmp_heading = Robot_Heading;
        float cost = 0;
        for(uint j=i+1;j<Setpoints.size();j++)
        {
            cv::Point3f cv_tmp_vec = PCL2CV(Setpoints[j]) - PCL2CV(Setpoints[i]);
            cv_tmp_vec.y = 0;
            cv_tmp_vec = cv_tmp_vec*(1/vec_mag(cv_tmp_vec));

            PointT tmp_vec = CV2PCL(cv_tmp_vec);
            float attitude_var = fabs(RobotPointAngle(tmp_heading,tmp_vec));


            if(attitude_var > ATTITIDE_FLUCTUATION)
            {
                tmp_heading =  tmp_vec;
                cost = cost + attitude_var;
            }

        }

        for(uint j=1;j<=i;j++)
            cost = cost + vec_mag( (PCL2CV(Setpoints[j]) - PCL2CV(Setpoints[j-1])));

        //        cost = cost + sqrt(10*i);
        low_cost_vec.push_back(make_pair(cost,i));
    }
    std::sort(low_cost_vec.begin(),low_cost_vec.end(),float_compare_min);

    return make_pair(Setpoints[low_cost_vec[0].second],0);

}

Planning::Define_area_struct Planning::Define_area(int disp, bool print, int cluster_num,PointCloud<PointXYZ>::Ptr _cloud,PointCloud<PointXYZ>::Ptr Wall_cloud, PointT _dim, uint divide_method, uint method, uint correction_method, float modify)
{
    Define_area_struct Fcn_outPut;
    PointT dim = _dim;
    float x_min,x_max,y_min,y_max,z_min,z_max;
    x_min = y_min = z_min =  100000000000;
    x_max = y_max = z_max = -100000000000;
    int Cluster_num,X_block_num,Y_block_num,Z_block_num;
    PointT cloud_mean_pos;
    int cluster_mean_size = 0;
    int counter = 0,mean_points = 0,less_mean_counter = 0;
    float max_deviation = 0,mean_deviation = 0;
    float Plane_dist_fit = 0;
    int max_point = 40;

    pcl::PointCloud<pcl::PointXYZ>::Ptr Wrong_pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i=0;i<_cloud->points.size();i++)
    {
        x_min = (_cloud->points[i].x<x_min)?(_cloud->points[i].x):(x_min);
        y_min = (_cloud->points[i].y<y_min)?(_cloud->points[i].y):(y_min);
        z_min = (_cloud->points[i].z<z_min)?(_cloud->points[i].z):(z_min);

        x_max = (_cloud->points[i].x>x_max)?(_cloud->points[i].x):(x_max);
        y_max = (_cloud->points[i].y>y_max)?(_cloud->points[i].y):(y_max);
        z_max = (_cloud->points[i].z>z_max)?(_cloud->points[i].z):(z_max);
    }
    cloud_mean_pos.x = (x_min+x_max)/2;
    cloud_mean_pos.y = (y_min+y_max)/2;
    cloud_mean_pos.z = (z_min+z_max)/2;

    //    viewer.removeShape("mean_pos");
    //    viewer.addSphere(cloud_mean_pos,0.05,"mean_pos");

    std::vector<cv::Vec3f> property;


    if(divide_method == NUM_DIVIDE_METHOD)
    {
        float max_dist = max(max((x_max-x_min),(y_max-y_min)),(z_max-z_min));

        if(max_dist>4)
        {
            dim.x = dim.x+1;
            dim.y = dim.y+1;
            dim.z = dim.z+1;
            //            qDebug()<<"max_dist"<<max_dist;
        }

        property.push_back(cv::Vec3f(x_min,x_max,((x_max-x_min)/dim.x)+(x_max-x_min)/10000));
        property.push_back(cv::Vec3f(y_min,y_max,((y_max-y_min)/dim.y)+(y_max-y_min)/10000));
        property.push_back(cv::Vec3f(z_min,z_max,((z_max-z_min)/dim.z)+(z_max-z_min)/10000));

        X_block_num = (int)(dim.x);
        Y_block_num = (int)(dim.y);
        Z_block_num = (int)(dim.z);
    }
    else
    {
        Cluster_num = 50;

        while(Cluster_num >300 || mean_points < 20)
        {
            property.clear();
            float mod_X = (float)fmod((double)(x_max-x_min),(double)dim.x);
            float mod_Y = (float)fmod((double)(y_max-y_min),(double)dim.y);
            float mod_Z = (float)fmod((double)(z_max-z_min),(double)dim.z);

            if(mod_X<(0.5*dim.x))
            {
                X_block_num = (int)((double)(x_max-x_min)/dim.x);

                dim.x =  dim.x + 1.0001*mod_X/((float)X_block_num);
            }
            if(mod_Y<(0.5*dim.y))
            {
                Y_block_num = (int)((double)(y_max-y_min)/dim.y);

                dim.y =  dim.y + 1.0001*mod_Y/((float)Y_block_num);
            }
            if(mod_Z<(0.5*dim.z))
            {
                Z_block_num = (int)((double)(z_max-z_min)/dim.z);

                dim.z =  dim.z + 1.0001*mod_Z/((float)Z_block_num);
            }

            property.push_back(cv::Vec3f(x_min,x_max,dim.x+(x_max-x_min)/10000));
            property.push_back(cv::Vec3f(y_min,y_max,dim.y+(y_max-y_min)/10000));
            property.push_back(cv::Vec3f(z_min,z_max,dim.z+(z_max-z_min)/10000));

            X_block_num = (int)ceil((double)(x_max-x_min)/dim.x);
            Y_block_num = (int)ceil((double)(y_max-y_min)/dim.y);
            Z_block_num = (int)ceil((double)(z_max-z_min)/dim.z);

            X_block_num = (X_block_num < 1)?(1):(X_block_num);
            Y_block_num = (Y_block_num < 1)?(1):(Y_block_num);
            Z_block_num = (Z_block_num < 1)?(1):(Z_block_num);

            Cluster_num = int(X_block_num*Y_block_num*Z_block_num);
            mean_points = _cloud->points.size()/Cluster_num;

            if(Cluster_num >300 || mean_points < 20)
            {
                dim.x = dim.x*1.1;
                dim.y = dim.y*1.05;
                dim.z = dim.z*1.05;
            }


        }

        //        qDebug()<<(x_max-x_min)<<(y_max-y_min)<<(z_max-z_min);

        //        qDebug()<<fmod((double)(x_max-x_min),(double)dim.x)<<fmod((double)(y_max-y_min),(double)dim.y)<<fmod((double)(z_max-z_min),(double)dim.z);
    }

    Cluster_num = int(X_block_num*Y_block_num*Z_block_num);
    mean_points = _cloud->points.size()/Cluster_num;

    uint i,j,k;
    std::vector<PointCloud<PointXYZ>::Ptr> cloud_vec;
    cloud_vec.resize(Cluster_num);

    for(int q =0;q<Cluster_num;q++)
        cloud_vec[q] = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    for(int c=0;c<_cloud->points.size();c++)
    {
        i = (uint)((_cloud->points[c].x-property[0][0])/property[0][2]);
        j = ((uint)X_block_num)*(uint)((_cloud->points[c].y-property[1][0])/property[1][2]);
        k = ((uint)(X_block_num*Y_block_num))*(int)((_cloud->points[c].z-property[2][0])/property[2][2]);

        cloud_vec[(i+j+k)]->points.push_back(_cloud->points[c]);
    }

    for(int c=0;c<Cluster_num;c++)
    {
        //        if(cloud_vec[c]->points.size() >= (mean_points*0.6))
        {
            remove_outlier_filter(cloud_vec[c],cloud_vec[c],0.3,5);
            cloud_denser(cloud_vec[c],cloud_vec[c],0.2,3,10);
            //            remove_outlier_filter(cloud_vec[c],cloud_vec[c],0.3,5);
            //            cloud_denser(cloud_vec[c],cloud_vec[c],0.2,3,20);
        }
    }

    //find the blocks that its height(y) are lower(higher) the the other.......
    std::vector<std::pair<float,int> > block_height;
    for(int c=0;c<Cluster_num;c++)
    {
        if(cloud_vec[c]->points.size() >= (mean_points*0.5) || cloud_vec[c]->points.size() >= max_point)
        {
            float sum_tmp = 0;
            for(int k=0;k<cloud_vec[c]->points.size();k++)
                sum_tmp = sum_tmp+cloud_vec[c]->points[k].y;
            block_height.push_back(std::pair<float,int>((sum_tmp/cloud_vec[c]->points.size()),c));
        }
    }

    std::sort(block_height.begin(),block_height.end(),float_compare);
    //    for(int i=0;i<block_height.size();i++)
    //        qDebug()<< block_height[i];

    float min_block_height = block_height[0].first;
    int min_bound = 0;
    for(int i=1;i<block_height.size();i++)
    {
        if(fabs(block_height[0].first-block_height[i].first) <= 0.2)
            min_block_height += block_height[i].first;
        else
        {
            min_block_height /= i;
            min_bound = i;
            i = block_height.size();
        }
    }

    //make the points more dense and pricise inside of each block...........
    for(int i=0;i<min_bound;i++)
    {
        if(cloud_vec[block_height[i].second]->points.size() < (2*mean_points))
        {
            float Block_x_min = 100000,Block_z_min = 100000;
            float Block_x_max = -100000,Block_z_max = -100000;
            for(int j=0;j<cloud_vec[block_height[i].second]->points.size();j++)
            {
                Block_x_min = min(cloud_vec[block_height[i].second]->points[j].x,Block_x_min);
                Block_z_min = min(cloud_vec[block_height[i].second]->points[j].z,Block_z_min);

                Block_x_max = max(cloud_vec[block_height[i].second]->points[j].x,Block_x_max);
                Block_z_max = max(cloud_vec[block_height[i].second]->points[j].z,Block_z_max);
            }
            while(cloud_vec[block_height[i].second]->points.size() < (4*mean_points))
            {
                //                float offset = 0.2;
                //                int l=5,d=5;
                //                float del_l = (1.0-offset)*((x_max-x_min)/(float)l);
                //                float del_d = (1.0-offset)*((z_max-z_min)/(float)d);
                PointT add_point(0,0,0);
                //                for(l;l>0;l--)
                //                {
                //                   for(d;d>0;d--)
                //                   {
                //                     add_point.x = del_l*l + x_min +offset +(x_max-x_min);
                //                     add_point.y = min_block_height;
                //                     add_point.z = del_d*d + z_min +offset +(z_max-z_min);
                //                   }
                //                }

                add_point.x = 0.5*(Block_x_min+Block_x_max) + (Block_x_max-Block_x_min)*0.9*RAND_NUM*0.5;
                add_point.y = min_block_height;
                add_point.z = 0.5*(Block_z_min+Block_z_max) + (Block_z_max-Block_z_min)*0.9*RAND_NUM*0.5;
                //                qDebug()<<add_point.x<<add_point.y<<add_point.z<<(max_x-min_x)*(float)fabs(((double)(rand())/(double)RAND_MAX));
                cloud_vec[block_height[i].second]->points.push_back(add_point);
                counter--;
            }
            //            QByteArray Points_str = QString("ADD_POINTS"+QString::number(i)).toLatin1();
            //            add_cloud2viewer(viewer,cloud_vec[block_height[i].second],Points_str.data(),255,255,255,8);

        }
        for(int k=0;k<cloud_vec[block_height[i].second]->points.size();k++)
        {

            float dist = cloud_vec[block_height[i].second]->points[k].y - min_block_height;
            if(dist <0.1)
                cloud_vec[block_height[i].second]->points[k].y = min_block_height + (0.5*dist);
        }
    }

    //Display the min plane............
    //        {
    //            pcl::ModelCoefficients coeff_tmp;
    //            coeff_tmp.values.resize(11);
    //            coeff_tmp.values[0] = 0;coeff_tmp.values[1] = -1;coeff_tmp.values[2] = 0;
    //            coeff_tmp.values[3] = 0;coeff_tmp.values[4] = min_block_height;coeff_tmp.values[5] = 2;
    //            coeff_tmp.values[6] = coeff_tmp.values[7] = 0.6;
    //            coeff_tmp.values[8] = 255;coeff_tmp.values[9] = 255;coeff_tmp.values[10] = 255;
    //            draw_plane(viewer,coeff_tmp,"mean_min_plane");
    //        }

    //Fit plane to every block..................
    x_min = y_min = z_min =  100000000000;
    x_max = y_max = z_max = -100000000000;
    std::vector<std::pair<pcl::ModelCoefficients,PointCloudT::Ptr> > plane_property;
    PointCloudT::Ptr Nr_cloud(new PointCloudT);



    for(int c=0;c<Cluster_num;c++)
    {
        if(cloud_vec[c]->points.size() >= (mean_points*0.6) ||  cloud_vec[c]->points.size() >= max_point)
        {
            PointCloudT::Ptr _cloud_tmp(new PointCloudT);
            float max_dim = max(max(property[0][0],property[0][1]),property[0][2]);
            //            qDebug()<<max_dim/10;
            Plane_dist_fit = max_dim/10;
            Plane_dist_fit = 0.1;
            bool obv_flag = 0;

            for(int m=0;m<min_bound;m++)
            {
                if(c == block_height[m].second)
                    obv_flag =1;
            }
            if(obv_flag)
            {
                Plane_dist_fit /= 2;
            }
            //            remove_outlier(cloud_vec[c],_cloud_tmp,(mean_points/3),(max_dim/2));
            _cloud_tmp->points.resize(cloud_vec[c]->points.size());
            _cloud_tmp->points = cloud_vec[c]->points;

            for(int t=0;t<_cloud_tmp->points.size();t++)
            {
                x_min = (_cloud_tmp->points[t].x<x_min)?(_cloud_tmp->points[t].x):(x_min);
                y_min = (_cloud_tmp->points[t].y<y_min)?(_cloud_tmp->points[t].y):(y_min);
                z_min = (_cloud_tmp->points[t].z<z_min)?(_cloud_tmp->points[t].z):(z_min);

                x_max = (_cloud_tmp->points[t].x>x_max)?(_cloud_tmp->points[t].x):(x_max);
                y_max = (_cloud_tmp->points[t].y>y_max)?(_cloud_tmp->points[t].y):(y_max);
                z_max = (_cloud_tmp->points[t].z>z_max)?(_cloud_tmp->points[t].z):(z_max);
            }

            if(_cloud_tmp->points.size()>(mean_points*0.6) || _cloud_tmp->points.size() >= max_point)
            {
                if(method == PLANE_FIT_METHOD)
                    plane_fit(_cloud_tmp,Plane_dist_fit,100,&plane_property);
                else if(method == NORMAL_CAL_METHOD)
                {
                    for(int j=0;j<cloud_vec[c]->points.size();j++)
                        Nr_cloud->points.push_back(_cloud_tmp->points[j]);
                    //                QByteArray Nr_str = QString("CPlane"+QString::number(c)).toLatin1();
                    //                Compute_Normal(viewer,cloud_vec[c],50,Nr_str.data());
                }
            }
            Plane_dist_fit =max_dim/10;
        }
        else
            less_mean_counter++;
        counter+=cloud_vec[c]->points.size();
    }

    Fcn_outPut.X_Area = cv::Point2f(x_min,x_max);
    Fcn_outPut.Y_Area = cv::Point2f(y_min,y_max);
    Fcn_outPut.Z_Area = cv::Point2f(z_min,z_max);


    if(print)
    {
        cout <<" Len:"<<(x_max-x_min)<<" X("<<x_min<<","<<x_max<<")";
        cout <<" Y("<<y_min<<","<<y_max<<")";
    }

    if(method == NORMAL_CAL_METHOD)
    {
        //        Compute_Normal(viewer,Nr_cloud,60,"Normals");
        //        viewer_thr->Draw_cloud2viewer(Nr_cloud,"N_points",50,220,55,3);
    }

    if(method == PLANE_FIT_METHOD)
    {
        // Clustring the Planes.............
        pcl::ModelCoefficients tmp_coeff;
        std::vector<pcl::ModelCoefficients> tmp_coeff_vec;
        tmp_coeff.values.resize(7);

        Eigen::Vector3f R_Point;



        for(int k=0;k<plane_property.size();k++)
        {
            PointT tmp_point;
            tmp_point = mean_cloud(plane_property[k].second);
            R_Point[0] = cloud_mean_pos.x - tmp_point.x ;
            R_Point[1] = cloud_mean_pos.y - tmp_point.y ;
            R_Point[2] = cloud_mean_pos.z - tmp_point.z ;

            Eigen::Vector3f vec_tmp(plane_property[k].first.values[0],plane_property[k].first.values[1],plane_property[k].first.values[2]);
            if(R_Point.dot(vec_tmp)<= 0)
            {
                plane_property[k].first.values[0] = -plane_property[k].first.values[0];
                plane_property[k].first.values[1] = -plane_property[k].first.values[1];
                plane_property[k].first.values[2] = -plane_property[k].first.values[2];
            }

            tmp_coeff.values[0] = plane_property[k].first.values[0];
            tmp_coeff.values[1] = plane_property[k].first.values[1];
            tmp_coeff.values[2] = plane_property[k].first.values[2];

            tmp_coeff.values[3] = tmp_point.x;
            tmp_coeff.values[4] = tmp_point.y;
            tmp_coeff.values[5] = tmp_point.z;
            tmp_coeff.values[6] = 0;
            tmp_coeff_vec.push_back(tmp_coeff);
        }

        if(tmp_coeff_vec.size() >= cluster_num)
            clustring(tmp_coeff_vec,cluster_num,3);
        else
        {
            Fcn_outPut.Plane_fit_dist = 0;
            return Fcn_outPut;
        }
        //Categorize the different Plane to various Vectors..........
        std::vector<std::vector<std::pair<pcl::ModelCoefficients,int> > > clustered_coeff_vec;
        clustered_coeff_vec.resize(cluster_num);

        std::vector<PointT> cloud_PA;
        std::vector<PointT> cluster_centers;
        cloud_PA.resize(cluster_num);
        cluster_centers.reserve(cluster_num);
        for(int k=0;k<tmp_coeff_vec.size();k++)
        {
            int cluster_idx = (int)(tmp_coeff_vec[k].values[6]);
            std::pair<pcl::ModelCoefficients,int> pair_tmp;
            pair_tmp.first =tmp_coeff_vec[k];
            pair_tmp.second = k;
            clustered_coeff_vec[cluster_idx].push_back(pair_tmp);

            cloud_PA[cluster_idx].x = cloud_PA[cluster_idx].x + tmp_coeff_vec[k].values[0];
            cloud_PA[cluster_idx].y = cloud_PA[cluster_idx].y + tmp_coeff_vec[k].values[1];
            cloud_PA[cluster_idx].z = cloud_PA[cluster_idx].z + tmp_coeff_vec[k].values[2];


        }


        if(Cluster_num >10)
            cluster_mean_size = ( 0.5*(tmp_coeff_vec.size()/cluster_num) >= 2)?( 0.5*(tmp_coeff_vec.size()/cluster_num)):(2);
        else
            cluster_mean_size = 1;

        // Determine the Outlier and compensate The principal Axes...........

        std::vector<std::set<int> > outlier,final_outlier;
        outlier.resize(cluster_num);
        final_outlier.resize(cluster_num);
        for(int i=0;i<cluster_num;i++)
        {
            //            qDebug()<<"class_size"<<i<<clustered_coeff_vec[i].size();
            float class_size = clustered_coeff_vec[i].size();
            Eigen::Vector3f PA(cloud_PA[i].x/class_size,cloud_PA[i].y/class_size,cloud_PA[i].z/class_size);
            cluster_centers[i] = PointT(0,0,0);
            for(int k=0;k<class_size;k++)
            {
                Eigen::Vector3f query( clustered_coeff_vec[i][k].first.values[0],clustered_coeff_vec[i][k].first.values[1],clustered_coeff_vec[i][k].first.values[2]);

                float dot_pro = fabs(PA.dot(query)/(sqrt(query.dot(query))*sqrt(PA.dot(PA))));

                if(dot_pro<0.96 || class_size<cluster_mean_size )// 15 degree
                {
                    //                    qDebug()<<"small_c"<<i<<clustered_coeff_vec[i].size();
                    outlier[i].insert(k);
                    cloud_PA[i].x = cloud_PA[i].x - clustered_coeff_vec[i][k].first.values[0];
                    cloud_PA[i].y = cloud_PA[i].y - clustered_coeff_vec[i][k].first.values[1];
                    cloud_PA[i].z = cloud_PA[i].z - clustered_coeff_vec[i][k].first.values[2];


                }
                else
                {
                    cluster_centers[i].x = cluster_centers[i].x+clustered_coeff_vec[i][k].first.values[3];
                    cluster_centers[i].y = cluster_centers[i].y+clustered_coeff_vec[i][k].first.values[4];
                    cluster_centers[i].z = cluster_centers[i].z+clustered_coeff_vec[i][k].first.values[5];
                }

            }

            if((class_size-outlier[i].size()) >0)
            {
                float denum = (float)(class_size-outlier[i].size());
                cluster_centers[i].x = (cluster_centers[i].x/denum);
                cluster_centers[i].y = (cluster_centers[i].y/denum);
                cluster_centers[i].z = (cluster_centers[i].z/denum);

                cloud_PA[i].x = (cloud_PA[i].x/denum);
                cloud_PA[i].y = (cloud_PA[i].y/denum);
                cloud_PA[i].z = (cloud_PA[i].z/denum);

                denum = vec_mag(cloud_PA[i]);
                //                qDebug()<<"class"<<i<<"mag"<<denum;

                cloud_PA[i].x = (cloud_PA[i].x/denum);
                cloud_PA[i].y = (cloud_PA[i].y/denum);
                cloud_PA[i].z = (cloud_PA[i].z/denum);

            }
            else
            {
                cloud_PA[i].x = 0;
                cloud_PA[i].y = 0;
                cloud_PA[i].z = 0;
            }
        }


        //eliminate the minor principal axes and merge by the other major ones.................
        std::vector<std::pair<std::pair<int,int>,float> > axes_product;
        int PA_outlier = 0;
        for(int i=0;i<cluster_num;i++)
        {
            std::pair<std::pair<int,int>,float> tmp;
            Eigen::Vector3f V1(cloud_PA[i].x,cloud_PA[i].y,cloud_PA[i].z);
            float V1_mag = vec_mag(cloud_PA[i]);
            if(V1_mag !=0)
            {
                for(int j=i+1;j<cluster_num;j++)
                {

                    float V2_mag = vec_mag(cloud_PA[j]);
                    if(V2_mag !=0)
                    {
                        Eigen::Vector3f V2(cloud_PA[j].x,cloud_PA[j].y,cloud_PA[j].z);

                        tmp.first.first = i;
                        tmp.first.second = j;
                        tmp.second = V2.dot(V1)/(V1_mag*V2_mag);
                        axes_product.push_back(tmp);

                        if(tmp.second>0.707)
                            PA_outlier++;
                        else if(tmp.second>0.5)
                        {
                            float local_max = -100;
                            for(int k=0;k<cluster_num;k++)
                            {
                                float V3_mag = vec_mag(cloud_PA[k]);
                                Eigen::Vector3f V3(cloud_PA[k].x,cloud_PA[k].y,cloud_PA[k].z);
                                if(k != j && V3_mag !=0)
                                {

                                    float dot_tmp = V2.dot(V3)/(V3_mag*V2_mag);
                                    local_max = max(dot_tmp,local_max);
                                }
                                if(k != i && V3_mag !=0)
                                {
                                    float dot_tmp = V1.dot(V3)/(V3_mag*V1_mag);
                                    local_max = max(dot_tmp,local_max);
                                }
                            }
                            if(local_max <= tmp.second)
                            {
                                PA_outlier++;
                            }
                        }
                    }
                }
            }
        }

        std::sort(axes_product.begin(),axes_product.end(),dot_product_compare);

        //        for(int i=0;i<axes_product.size();i++)
        //        {
        //            qDebug()<<axes_product[i].first.first<<axes_product[i].first.second<<axes_product[i].second;
        //        }

        for(int i=0;i<PA_outlier;i++)
        {
            int row = axes_product[i].first.first;
            int col = axes_product[i].first.second;
            float  _min,min_1 = 10,min_2 = 10;
            float min1_abs=10,min2_abs=10;
            int class_num,class_num_1 = 10,class_num_2 = 20;
            int class_num_1_abs = 10,class_num_2_abs = 20;
            bool miss_1 = 0,miss_2 = 0;

            for(int j=PA_outlier;j<axes_product.size();j++)
            {

                if(row == axes_product[j].first.first || row == axes_product[j].first.second)
                {
                    if(axes_product[j].second <= min_1)
                    {
                        min_1 = axes_product[j].second;
                        class_num_1 = row;
                    }

                    if(fabs(axes_product[j].second) <= min1_abs)
                    {
                        min1_abs = fabs(axes_product[j].second);
                        class_num_1_abs = row;
                    }

                    if(axes_product[j].second>0.34)//70 degree
                    {
                        bool exist = 0;
                        for(int m=0;m<PA_outlier;m++)
                        {
                            if(m!=i)
                            {
                                if(row == axes_product[m].first.first || row != axes_product[m].first.second)
                                    exist = 1;
                            }
                        }
                        if(!exist)
                            miss_1  = 1;
                    }
                }

                if(col == axes_product[j].first.first || col == axes_product[j].first.second)
                {
                    if(axes_product[j].second <= min_2)
                    {
                        min_2 = axes_product[j].second;
                        class_num_2 = col;
                    }

                    if(fabs(axes_product[j].second) <= min2_abs)
                    {
                        min2_abs = fabs(axes_product[j].second);
                        class_num_2_abs = col;
                    }

                    if(axes_product[j].second>0.34)//70 degree
                    {
                        bool exist = 0;
                        for(int m=0;m<PA_outlier;m++)
                        {
                            if(m!=i)
                            {
                                if(col == axes_product[m].first.first || col != axes_product[m].first.second)
                                    exist = 1;
                            }
                        }
                        if(!exist)
                            miss_2  = 1;
                    }
                }
            }

            if( ((int)miss_1+(int)miss_2) > 0 )
            {

                class_num = ((int)(!miss_1))*class_num_1 + ((int)(!miss_2))*class_num_2;
                _min = ((int)(!miss_1))*min_1 + ((int)(!miss_2))*min_2;

                if(fabs(_min)<0.9)//25 degree
                {
                    _min = ((int)(!miss_1))*min1_abs + ((int)(!miss_2))*min2_abs;
                    class_num = ((int)(!miss_1))*class_num_1_abs + ((int)(!miss_2))*class_num_2_abs;
                }
            }
            else
            {
                //                qDebug("SHiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiT");
                _min = min(min_1,min_2);


                if(_min == min_1)
                    class_num  =class_num_1;
                else
                    class_num  =class_num_2;

                if(fabs(_min)<0.9)//25 degree
                {
                    _min = min(fabs(min1_abs),fabs(min2_abs));

                    if(_min == fabs(min1_abs))
                    {
                        class_num  =class_num_1_abs;
                        _min == min1_abs;

                    }
                    else
                    {
                        class_num  =class_num_2_abs;
                        _min == min2_abs;
                    }
                }
            }

            Eigen::Vector3f PA(cloud_PA[class_num].x,cloud_PA[class_num].y,cloud_PA[class_num].z);
            if(class_num == row)
            {
                int class_size = clustered_coeff_vec[col].size();
                cloud_PA[col] = cloud_PA[class_num];
                for(int k=0;k<class_size;k++)
                {
                    Eigen::Vector3f query( clustered_coeff_vec[col][k].first.values[0],clustered_coeff_vec[col][k].first.values[1],clustered_coeff_vec[col][k].first.values[2]);

                    float dot_pro = fabs(PA.dot(query)/(sqrt(query.dot(query))*sqrt(PA.dot(PA))));

                    if(dot_pro<0.97)// 15 degree
                        outlier[col].insert(k);
                }
            }
            else if(class_num == col)
            {
                int class_size = clustered_coeff_vec[row].size();
                cloud_PA[row] = cloud_PA[class_num];
                for(int k=0;k<class_size;k++)
                {
                    Eigen::Vector3f query( clustered_coeff_vec[row][k].first.values[0],clustered_coeff_vec[row][k].first.values[1],clustered_coeff_vec[row][k].first.values[2]);

                    float dot_pro = fabs(PA.dot(query)/(sqrt(query.dot(query))*sqrt(PA.dot(PA))));

                    if(dot_pro<0.97)// 15 degree
                        outlier[row].insert(k);
                }
            }

            //            qDebug()<<"outlier"<<PA_outlier<<"Selected_calss"<<class_num<<_min<<miss_1<<miss_2;
        }

        //Add Wrong points to a specific Array..............
        std::set<int> wrong_idx;
        for(int i=0;i<cluster_num;i++)
        {
            vector<int> outlier_vec(outlier[i].begin(),outlier[i].end());
            for(int k=0;k<outlier_vec.size();k++)
            {
                int idx = clustered_coeff_vec[i][outlier_vec[k]].second;
                if(wrong_idx.find(idx) == wrong_idx.end())
                {
                    wrong_idx.insert(idx);
                    for(int q=0;q<plane_property[idx].second->points.size();q++)
                        Wrong_pointCloud->points.push_back(plane_property[idx].second->points[q]);
                }
            }
        }

        //Determin the correct category for the outliers....
        std::vector<pcl::ModelCoefficients> corrected_clustered_coeff_vec;
        {

            if(disp < 3)
                cloud_classifyer(outlier,clustered_coeff_vec,corrected_clustered_coeff_vec,cloud_PA,cluster_centers,1);
            else
                cloud_classifyer(outlier,clustered_coeff_vec,corrected_clustered_coeff_vec,cloud_PA,cluster_centers,0);

            // Check again for outliers................
            for(int i=0;i<cluster_num;i++)
            {
                if( (clustered_coeff_vec[i].size()< cluster_mean_size) && (clustered_coeff_vec[i].size()>0) )
                {
                    for(int k=0;k<clustered_coeff_vec[i].size();k++)
                        final_outlier[i].insert(k);
                }
            }

            for(int i=0;i<cluster_num;i++)
            {
                vector<int> final_outlier_vec(final_outlier[i].begin(),final_outlier[i].end());
                for(int k=0;k<final_outlier_vec.size();k++)
                {
                    int idx = clustered_coeff_vec[i][final_outlier_vec[k]].second;
                    if(wrong_idx.find(idx) == wrong_idx.end())
                    {
                        wrong_idx.insert(idx);
                        for(int q=0;q<plane_property[idx].second->points.size();q++)
                            Wrong_pointCloud->points.push_back(plane_property[idx].second->points[q]);
                    }
                }
            }
            //             Invoke The K-NN classifyer again........
            if(disp < 3)
                cloud_classifyer(final_outlier,clustered_coeff_vec,corrected_clustered_coeff_vec,cloud_PA,cluster_centers,1);
            else
                cloud_classifyer(final_outlier,clustered_coeff_vec,corrected_clustered_coeff_vec,cloud_PA,cluster_centers,0);
        }

        //Final correction...............
        PA_axes_correction(cloud_PA,0.8);
        if(disp < 3)
        {

            for(int i=0;i<cluster_num;i++)
            {
                //            if(clustered_coeff_vec[i].size()<cluster_mean_size && clustered_coeff_vec[i].size()>0)
                //                qDebug()<<"class_size"<<i<<clustered_coeff_vec[i].size();

                for(int k=0;k<clustered_coeff_vec[i].size();k++)
                {
                    clustered_coeff_vec[i][k].first.values[0] = (cloud_PA[i].x);
                    clustered_coeff_vec[i][k].first.values[1] = (cloud_PA[i].y);
                    clustered_coeff_vec[i][k].first.values[2] = (cloud_PA[i].z);
                }

            }

            //            if((counter-_cloud->points.size()) != 0)
            //            {
            //                qDebug()<<"Input_Output contrary"<< (counter-_cloud->points.size());
            //                Fcn_outPut.Plane_fit_dist = 0;
            //                return Fcn_outPut;
            //            }

            //compute the mean of each class center.................
            std::vector<PointT> mean_class_center;
            mean_class_center.resize(cluster_num);
            for(int i=0;i<cluster_num;i++)
            {
                PointT mean_tmp(0,0,0);
                for(int k=0;k<clustered_coeff_vec[i].size();k++)
                {
                    mean_tmp.x += clustered_coeff_vec[i][k].first.values[3]/((float)clustered_coeff_vec[i].size());
                    mean_tmp.y += clustered_coeff_vec[i][k].first.values[4]/((float)clustered_coeff_vec[i].size());
                    mean_tmp.z += clustered_coeff_vec[i][k].first.values[5]/((float)clustered_coeff_vec[i].size());
                }
                mean_class_center[i] = mean_tmp;
            }

            //correct the pointcloud................
            _cloud->points.clear();
            uint corrected_point_counter = 0;
            PointCloudT::Ptr each_block_corrected_cloud (new PointCloudT);
            for(int i=0;i<cluster_num;i++)
            {
                uint each_cluster_point_size = 0,point_must_add =0;
                for(int k=0;k<clustered_coeff_vec[i].size();k++)
                {
                    each_cluster_point_size += plane_property[clustered_coeff_vec[i][k].second].second->points.size();
                }
                //                point_must_add = (each_cluster_point_size<3000)?((3000-each_cluster_point_size)/(clustered_coeff_vec[i].size()+1)):(mean_points);

                pcl::ModelCoefficients each_block_coeff;
                for(int k=0;k<clustered_coeff_vec[i].size();k++)
                {
                    int each_cluster_points_size = 0;
                    int each_block_size = plane_property[clustered_coeff_vec[i][k].second].second->points.size();
                    point_must_add = (each_block_size<500)?((500-each_block_size)):(mean_points);
                    each_block_coeff.values.clear();
                    each_block_coeff.values.resize(6);
                    each_block_coeff.values[0] = clustered_coeff_vec[i][k].first.values[0];
                    each_block_coeff.values[1] = clustered_coeff_vec[i][k].first.values[1];
                    each_block_coeff.values[2] = clustered_coeff_vec[i][k].first.values[2];
                    each_block_coeff.values[3] = mean_class_center[i].x;
                    each_block_coeff.values[4] = mean_class_center[i].y;
                    each_block_coeff.values[5] = mean_class_center[i].z;

                    PointT each_block_center(0,0,0);
                    PointT each_block_min(100000,100000,10000);
                    PointT each_block_max(-100000,-100000,-10000);

                    each_block_corrected_cloud->points.clear();
                    for(int j=0;j<each_block_size;j++)
                    {
                        each_cluster_points_size++;
                        PointT each_block_point = plane_property[clustered_coeff_vec[i][k].second].second->points[j];
                        PointT Dist_vec(0,0,0);



                        if(correction_method == GLOBAL_MEAN_CORRECT_METHOD)
                            Dist_vec = find2planeDist(each_block_coeff,each_block_point);
                        else if(correction_method == LOCAL_MEAN_CORRECT_METHOD)
                            Dist_vec = find2planeDist(clustered_coeff_vec[i][k].first,each_block_point);

                        //                    max_deviation += vec_mag(Dist_vec);//max(max_deviation,vec_mag(Dist_vec));
                        //                    max_deviation /=2;
                        each_block_point.x += Dist_vec.x*modify;
                        each_block_point.y += Dist_vec.y*modify;
                        each_block_point.z += Dist_vec.z*modify;


                        float each_point_dist = 0.1;
                        if(correction_method == GLOBAL_MEAN_CORRECT_METHOD)
                            each_point_dist = pointPlaneDist(each_block_coeff,each_block_point);
                        else if(correction_method == LOCAL_MEAN_CORRECT_METHOD)
                            each_point_dist = pointPlaneDist(clustered_coeff_vec[i][k].first,each_block_point);

                        corrected_point_counter++;
                        mean_deviation += each_point_dist;
                        max_deviation = max(max_deviation,each_point_dist);
                        //                    max_deviation /=2;

                        each_block_corrected_cloud->points.push_back(each_block_point);


                        each_block_center.x += each_block_point.x;
                        each_block_center.y += each_block_point.y;
                        each_block_center.z += each_block_point.z;

                        each_block_min.x = min(each_block_min.x,each_block_point.x);
                        each_block_min.y = min(each_block_min.y,each_block_point.y);
                        each_block_min.z = min(each_block_min.z,each_block_point.z);

                        each_block_max.x = max(each_block_max.x,each_block_point.x);
                        each_block_max.y = max(each_block_max.y,each_block_point.y);
                        each_block_max.z = max(each_block_max.z,each_block_point.z);

                    }

                    each_block_center.x /= (float)each_cluster_points_size;
                    each_block_center.y /= (float)each_cluster_points_size;
                    each_block_center.z /= (float)each_cluster_points_size;

                    //                    if(pointPlaneDist(each_block_coeff,each_block_center) < 0.3)
                    {

                        each_block_coeff.values[3] = each_block_center.x;
                        each_block_coeff.values[4] = each_block_center.y;
                        each_block_coeff.values[5] = each_block_center.z;




                        float x_len,y_len,z_len;
                        x_len = each_block_max.x-each_block_min.x;
                        y_len = each_block_max.y-each_block_min.y;
                        z_len = each_block_max.z-each_block_min.z;
                        float extra_area = 1;
                        for(int p=0;p<point_must_add;p++)
                        {
                            PointT rand_point,extra_point;
                            rand_point.x = each_block_center.x + x_len*0.5*RAND_NUM*extra_area;
                            rand_point.y = each_block_center.y + y_len*0.5*RAND_NUM*extra_area;
                            rand_point.z = each_block_center.z+ z_len*0.5*RAND_NUM*extra_area;

                            if(correction_method == GLOBAL_MEAN_CORRECT_METHOD)
                                extra_point = findPointReflection_2(each_block_coeff,rand_point);
                            else if(correction_method == LOCAL_MEAN_CORRECT_METHOD)
                                extra_point = findPointReflection_2(clustered_coeff_vec[i][k].first,rand_point);

                            each_block_corrected_cloud->points.push_back(extra_point);
                            //                    each_cluster_points_size--;
                        }
                    }


                    remove_outlier_filter(each_block_corrected_cloud,each_block_corrected_cloud,0.2,10);

                    for(int p=0;p<each_block_corrected_cloud->points.size();p++)
                        _cloud->points.push_back(each_block_corrected_cloud->points[p]);

                    PointT floor_normal(0,1,0);
                    PointT each_block_normal(each_block_coeff.values[0],each_block_coeff.values[1],each_block_coeff.values[2]);

                    if( fabs(Vec2VecAngle(floor_normal,each_block_normal)) < 0.7)
                    {
                        for(int p=0;p<each_block_corrected_cloud->points.size();p++)
                            Wall_cloud->points.push_back(each_block_corrected_cloud->points[p]);
                    }

                }

            }

            mean_deviation /= corrected_point_counter;
            mean_deviation *=4;

            //        if(correction_method == LOCAL_MEAN_CORRECT_METHOD)
            max_deviation = (max_deviation+mean_deviation)/2;
            //        else
            //        max_deviation = mean_deviation;

            //        viewer.removeAllShapes();

        }
        //Insert Wrong Point to the Oroginal Cloud...............
        int w_num = Wrong_pointCloud->points.size();
        remove_outlier_filter(Wrong_pointCloud,Wrong_pointCloud,0.2,7);
        for(int i=0;i<Wrong_pointCloud->points.size();i++)
        {
            _cloud->points.push_back(Wrong_pointCloud->points[i]);
        }

        //        cout<<" Diff:"<<w_num-Wrong_pointCloud->points.size()<<"/"<<w_num;

        //        compute_PA_cloud(_cloud);
        //exhibit the Pricipal Axes................
        if(disp > 0)
        {
            viewer_thr->RemoveFromViewer(shapes_name_2);
            shapes_name_2.clear();



            for(int i=0;i<cluster_num;i++)
            {
                pcl::ModelCoefficients cylinder_coeff;
                cylinder_coeff.values.resize(10);
                QByteArray cyl_str = QString("cyl"+QString::number(i)).toLatin1();
                float accept = cloud_PA[i].x + cloud_PA[i].y + cloud_PA[i].z;
                if(clustered_coeff_vec[i].size()>0 && accept != 0)
                {

                    cylinder_coeff.values[3] = cloud_PA[i].x;
                    cylinder_coeff.values[4] = cloud_PA[i].y;
                    cylinder_coeff.values[5] = cloud_PA[i].z;

                    cylinder_coeff.values[0] = cloud_mean_pos.x;
                    cylinder_coeff.values[1] = cloud_mean_pos.y;
                    cylinder_coeff.values[2] = cloud_mean_pos.z;
                    cylinder_coeff.values[6] = 0.02;
                    cylinder_coeff.values[7] = (i*100+10)/255.;
                    cylinder_coeff.values[8] = (i*100+20)/255.;
                    cylinder_coeff.values[9] = (i*100+30)/255.;
                    shapes_name_2.push_back(cyl_str);

                    //                qDebug()<<"cyl"<<i<<cloud_PA[i].x<<cloud_PA[i].y<<cloud_PA[i].z;

                    //                viewer.removeShape(cyl_str.data());
                    viewer_thr->DrawCylinder(cylinder_coeff,cyl_str);

                }
            }


        }
        // Display Planes, Normals and outliers ...............


        if(disp >1 )
        {
            pcl::ModelCoefficients tmp_coeff;
            tmp_coeff.values.resize(11);
            PointCloudT::Ptr Plane_center_cloud(new PointCloudT);
            pcl::PointCloud<pcl::Normal>::Ptr Plane_normal(new pcl::PointCloud<pcl::Normal>);
            for(int i=0;i<cluster_num;i++)
            {

                for(int k=0;k<clustered_coeff_vec[i].size();k++)
                {
                    QByteArray Plane_str = QString("CPlane"+QString::number(i)+QString::number(k)).toLatin1();
                    //                    QByteArray Plane_Nr_str = QString("Plane_Nr"+QString::number(i)+QString::number(k)).toLatin1();
                    tmp_coeff.values[0] = clustered_coeff_vec[i][k].first.values[0];
                    tmp_coeff.values[1] = clustered_coeff_vec[i][k].first.values[1];
                    tmp_coeff.values[2] = clustered_coeff_vec[i][k].first.values[2];

                    tmp_coeff.values[3] = clustered_coeff_vec[i][k].first.values[3];
                    tmp_coeff.values[4] = clustered_coeff_vec[i][k].first.values[4];
                    tmp_coeff.values[5] = clustered_coeff_vec[i][k].first.values[5];

                    Plane_normal->points.push_back(pcl::Normal(tmp_coeff.values[0],tmp_coeff.values[1],tmp_coeff.values[2]));
                    Plane_center_cloud->points.push_back(pcl::PointXYZ(tmp_coeff.values[3],tmp_coeff.values[4],tmp_coeff.values[5]));

                    tmp_coeff.values[6] = 0.3;
                    tmp_coeff.values[7] = 0.3;

                    tmp_coeff.values[8] = i*100+10;
                    tmp_coeff.values[9] = i*80+20;
                    tmp_coeff.values[10] = i*60+30;

                    viewer_thr->DrawPlane(tmp_coeff,Plane_str);


                    QByteArray Plane_points_str = QString("CPlane_points"+QString::number(i)+QString::number(k)).toLatin1();
                    //                    add_cloud2viewer(viewer,plane_property[clustered_coeff_vec[i][k].second].second,Plane_points_str.data(),k*100+50,k*100,k*80,5);

                }
                if(disp >3)
                {
                    vector<int> outlier_vec(outlier[i].begin(),outlier[i].end());
                    for(int k=0;k<outlier_vec.size();k++)
                    {
                        QByteArray Plane_str = QString("CPlane"+QString::number(i)+QString::number(outlier_vec[k])).toLatin1();
                        viewer_thr->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.,i*80/255.0,0.,Plane_str.data());
                    }

                    //                    vector<int> miss_understand_vec(miss_understand[i].begin(),miss_understand[i].end());
                    //                    for(int k=0;k<miss_understand_vec.size();k++)
                    //                    {
                    //                        QByteArray Plane_str = QString("CPlane"+QString::number(i)+QString::number(miss_understand_vec[k])).toLatin1();
                    //                        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.,0,1.,Plane_str.data());
                    //                    }
                }
            }
            viewer_thr->DrawNormal(Plane_center_cloud,Plane_normal,1,0.06,"Plane_Nr_str");

            if(disp >2)
            {
                Plane_center_cloud->clear();
                Plane_normal->clear();
                for(int i=0;i<corrected_clustered_coeff_vec.size();i++)
                {
                    QByteArray Plane_str = QString("CrPlane"+QString::number(i)).toLatin1();
                    QByteArray Plane_Nr_str = QString("CrPlane_Nr"+QString::number(i)).toLatin1();
                    tmp_coeff.values[0] = corrected_clustered_coeff_vec[i].values[0];
                    tmp_coeff.values[1] = corrected_clustered_coeff_vec[i].values[1];
                    tmp_coeff.values[2] = corrected_clustered_coeff_vec[i].values[2];

                    tmp_coeff.values[3] = corrected_clustered_coeff_vec[i].values[3];
                    tmp_coeff.values[4] = corrected_clustered_coeff_vec[i].values[4];
                    tmp_coeff.values[5] = corrected_clustered_coeff_vec[i].values[5];

                    Plane_normal->points.push_back(pcl::Normal(tmp_coeff.values[0],tmp_coeff.values[1],tmp_coeff.values[2]));
                    Plane_center_cloud->points.push_back(pcl::PointXYZ(tmp_coeff.values[3],tmp_coeff.values[4],tmp_coeff.values[5]));

                    tmp_coeff.values[6] = 0.3;
                    tmp_coeff.values[7] = 0.3;

                    tmp_coeff.values[8] = 0;
                    tmp_coeff.values[9] = 250;
                    tmp_coeff.values[10] = 0;

                    viewer_thr->DrawPlane(tmp_coeff,Plane_str);
                    viewer_thr->DrawNormal(Plane_center_cloud,Plane_normal,1,0.06,Plane_Nr_str);
                }
            }
        }

    }



    if(print)
        cout<<" mean_p:"<<mean_points<<" less_mean:"<<less_mean_counter<<"/"<<Cluster_num<<" Points:"<< _cloud->points.size();

    //    cout<<" mean_c:"<<cluster_mean_size<<"  mean_p:"<<mean_points<<" less_mean:"<<less_mean_counter<<"/"<<Cluster_num<<" Nr_points:"<<Nr_cloud->points.size()<<" All_points:"<< _cloud->points.size();

    Fcn_outPut.Plane_fit_dist = max_deviation;
    return Fcn_outPut;

    //    return Plane_dist_fit;
}

void Planning::request_response_frontier()
{
    emit request_response_frontier_signal();
}


void Planning::Run_THR()
{
    MainTimer->start(loop_time);
}

void Planning::request_response_frontier_slot()
{
    Request_Frontier_Flag = REQUEST_FRONTIER_RESPONSE_RECEIVED;
}

//PointCloudT::Ptr tmp_cloud (new PointCloudT());
//for(uint i=0;i<SetpointPlanning.border_pos.size();i++)
//    tmp_cloud->points.push_back(CV2PCL(SetpointPlanning.border_pos[i]));
//viewer_thr->Draw_cloud2viewer(tmp_cloud,"Border_POINTS",0,105,250,6);

