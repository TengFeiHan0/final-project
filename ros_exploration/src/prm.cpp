#include "prm.h"

PRM::PRM(QObject *parent) : QObject(parent)
{
    //    rrt_planning.space.Obs_radius = 0.2;
    //    rrt_planning.space.Robot_radius = 0.3;
    //    rrt_planning.space.Border_radius = 0.1;

    //    rrt_planning.RRT_Param.goal_THR = 0.1;
    //    rrt_planning.RRT_Param.Max_rrt_iteration = 20000;
    //    rrt_planning.RRT_Param.Max_smooth_iteration = 1000;
    //    rrt_planning.RRT_Param.resulotion = 0.05;
    //    rrt_planning.RRT_Param.step_len = 0.1;

}

PRM::~PRM()
{

}

bool PRM::GetOptimizePath()
{
    RoadMapConstruction();
    return Run_A_Star();
}

bool PRM::Run_A_Star()
{
    OpenSet.clear();
    CloseSet.clear();
    OptimizePath.clear();
    OpenSet.insert(start_point_idx);
    for(uint i=0;i<space.PRM_Points.size();i++)
    {
        G_Score[i] = 1000000;
        F_Score[i] = 1000000;
    }
    G_Score[start_point_idx] = 0;
    F_Score[start_point_idx] = H_dist(space.start_point,space.goal_point);

    uint current_idx;
    while(OpenSet.size() > 0)
    {
        current_idx = FindLowestFScore();

        for(uint q=0;q<goal_point_idx.size();q++)
        {
            if(current_idx == goal_point_idx[q])
            {
                nearest_goal_point = space.PRM_Points[current_idx];
                ReconstructPath(CameFrom,current_idx);
                return true;
            }
        }
        OpenSet.erase(OpenSet.find(current_idx));
        CloseSet.insert(current_idx);

        for(uint i=0;i<PRM_Graph[current_idx].size();i++)
        {
            int neighbor_idx = PRM_Graph[current_idx][i];
            if(CloseSet.find(neighbor_idx) != CloseSet.end())
                continue;
            float tentative_gScore = G_Score[current_idx] + H_dist(space.PRM_Points[current_idx],space.PRM_Points[neighbor_idx]);

            if(OpenSet.find(neighbor_idx) == OpenSet.end() )
            {
                OpenSet.insert(neighbor_idx);
            }
            else if(tentative_gScore >= G_Score[neighbor_idx])
                continue;

            CameFrom[neighbor_idx] = current_idx;
            G_Score[neighbor_idx] = tentative_gScore;
            F_Score[neighbor_idx] = tentative_gScore + H_dist(space.PRM_Points[neighbor_idx],space.goal_point);

        }
    }
    last_point = space.PRM_Points[current_idx];
    return false;
}

void PRM::ReconstructPath(std::map<int, int> _CameFrom, int _current)
{
    int current = _current;
    OptimizePath.clear();
    OptimizePath.push_back(space.PRM_Points[current]);
    while(current != start_point_idx)
    {
        current = _CameFrom[current];
        OptimizePath.push_back(space.PRM_Points[current]);
    }
}

bool PRM::goal_PRM_points_collision_correction(PRM::POINT _goal_point)
{
    std::vector<std::pair<float,int> > neighbor_points;
    float dist = 0;
    for(uint j=0;j<space.PRM_Points.size();j++)
    {
        dist =vec_len( (_goal_point-space.PRM_Points[j]));
        if(dist >= PRM_Param.Min_Graph_Edge_len)
            neighbor_points.push_back(make_pair(dist,j));
    }

    std::sort(neighbor_points.begin(),neighbor_points.end(),float_compare_min);
    int K_nearest = (space.PRM_Points.size()<PRM_Param.K_nearest)?(space.PRM_Points.size()):(PRM_Param.K_nearest);
    for(uint k=0;k<K_nearest;k++)
    {
        if(InCollision_Edge(_goal_point,space.PRM_Points[neighbor_points[k].second]) == 0)
        {
            return 1;
        }
    }

    POINT _start_point = space.PRM_Points[neighbor_points[0].second];
    std::vector<POINT> Around_cloud;
    search_Area(_goal_point,3,Around_cloud);


    POINT area1,area2;
    area1.x = min(_start_point.x,_goal_point.x) - 1.5;
    //    area1.y = min(_start_point.y,_goal_point.y) - 1.5;
    area1.y = space.Global_Floor_Ceil_Level.x;
    area1.z = min(_start_point.z,_goal_point.z) - 1.5;


    area2.x = max(_start_point.x,_goal_point.x) + 1.5;
    //    area2.y = max(_start_point.y,_goal_point.y) + 1.5;
    area2.y = space.Global_Floor_Ceil_Level.y;
    area2.z = max(_start_point.z,_goal_point.z) + 1.5;

    rrt_planning.space.admissible_area.first = area1;
    rrt_planning.space.admissible_area.second = area2;
    rrt_planning.space.Obs_pos.clear();
    rrt_planning.space.Obs_pos.resize(Around_cloud.size());
    rrt_planning.space.Obs_pos = Around_cloud;

    rrt_planning.space.goal_point = _goal_point;
    rrt_planning.space.start_point = _start_point;

    if(rrt_planning.PlanPathRRT() >-1)
    {

        rrt_planning.SmoothPath();
        rrt_planning.PathBreakPoint(5,0.2);
        for(int i=0;i<rrt_planning.BREAK_POINT.size();i++)
        {
            if(rrt_planning.BREAK_POINT[i] != _goal_point && rrt_planning.BREAK_POINT[i] != _start_point)
            {
                space.PRM_Points.push_back(rrt_planning.BREAK_POINT[i]);
                Inserted_PRM_Points.push_back(rrt_planning.BREAK_POINT[i]);
            }
        }
        return 1;
    }else
        return 0;

}

void PRM::search_Area(POINT curr_POS, float dist,std::vector<POINT> &selected_cloud)
{
    float dist_2 = dist*dist;
    for(uint i=0;i<space.Obs_pos.size();i++)
    {

        POINT dist_vec = space.Obs_pos[i] - curr_POS;
        if(vec_len_2(dist_vec)< dist_2 )
        {
            selected_cloud.push_back(space.Obs_pos[i]);
        }
    }
}

void PRM::RoadMapConstruction()
{
    if((rrt_planning.space.Robot_radius +rrt_planning.space.Obs_radius) < (space.Robot_radius+space.Obs_radius) )
    {
        rrt_planning.space.Robot_radius += 1.2*((space.Robot_radius+space.Obs_radius) - (rrt_planning.space.Robot_radius +rrt_planning.space.Obs_radius));
        qDebug()<<"PRM_RRT radius correction";
    }

    Inserted_PRM_Points.clear();
    for(uint i=0;i<space.goal_point.size();i++)
        goal_PRM_points_collision_correction(space.goal_point[i]);

    PRM_Graph.clear();
    bool start_find = 0;
    goal_point_idx.resize(space.goal_point.size());
    for(uint i=0;i<space.goal_point.size();i++)
        goal_point_idx[i] = -1;
    for(uint i=0;i<space.PRM_Points.size();i++)
    {
        if(space.PRM_Points[i] == space.start_point)
        {
            start_find = 1;
            start_point_idx = i;
        }

        for(uint j=0;j<space.goal_point.size();j++)
        {

            if(space.PRM_Points[i] == space.goal_point[j])
            {
                goal_point_idx[j]=i;
            }
        }
    }



    if(!start_find)
    {
        start_point_idx = space.PRM_Points.size();
        space.PRM_Points.push_back(space.start_point);
    }

    for(uint i=0;i<space.goal_point.size();i++)
    {
        if(goal_point_idx[i] == -1)
        {
            goal_point_idx[i] = space.PRM_Points.size();
            space.PRM_Points.push_back(space.goal_point[i]);
        }
    }

    for(uint i=0;i<space.PRM_Points.size();i++)
    {
        std::vector<std::pair<float,int> > neighbor_points;
        float dist = 0;
        for(uint j=0;j<space.PRM_Points.size();j++)
        {

            if( j!=i)
            {
                dist =vec_len( (space.PRM_Points[i]-space.PRM_Points[j]));
                if(dist >= PRM_Param.Min_Graph_Edge_len)
                    neighbor_points.push_back(make_pair(dist,j));
            }
        }
        std::sort(neighbor_points.begin(),neighbor_points.end(),float_compare_min);
        for(uint k=0;k<neighbor_points.size();k++)
        {
            if(k < PRM_Param.K_nearest || neighbor_points[k].first >= PRM_Param.edge_min_dist)
            {
                if(InCollision_Edge(space.PRM_Points[i],space.PRM_Points[neighbor_points[k].second]) == 0)
                {
                    bool observed = 0;
                    for(uint q=0;q<PRM_Graph[i].size();q++)
                    {
                        if(PRM_Graph[i][q] == neighbor_points[k].second)
                            observed = 1;
                    }
                    if(!observed)
                        PRM_Graph[i].push_back(neighbor_points[k].second);

                    observed = 0;
                    for(uint q=0;q<PRM_Graph[neighbor_points[k].second].size();q++)
                    {
                        if(PRM_Graph[neighbor_points[k].second][q] == i)
                            observed = 1;
                    }
                    if(!observed)
                        PRM_Graph[neighbor_points[k].second].push_back(i);
                }
            }

            if(k >= (PRM_Param.K_nearest-1) && neighbor_points[k].first >= PRM_Param.edge_min_dist)
                k = neighbor_points.size();
        }
    }

    for(int j =0;j<goal_point_idx.size();j++)
    {
        if(InCollision_Edge(space.PRM_Points[goal_point_idx[j]],space.PRM_Points[start_point_idx]) == 0)
        {
            int i=goal_point_idx[j];
            bool observed = 0;
            for(uint q=0;q<PRM_Graph[i].size();q++)
            {
                if(PRM_Graph[i][q] == start_point_idx)
                    observed = 1;
            }
            if(!observed)
                PRM_Graph[i].push_back(start_point_idx);

            observed = 0;
            for(uint q=0;q<PRM_Graph[start_point_idx].size();q++)
            {
                if(PRM_Graph[start_point_idx][q] == i)
                    observed = 1;
            }
            if(!observed)
                PRM_Graph[start_point_idx].push_back(i);
        }
    }

    //    for(uint i=0;i<space.PRM_Points.size();i++)
    //    {
    //        for(uint j=i+1;j<space.PRM_Points.size();j++)
    //        {
    //            bool find_node = 0;

    //            for(uint k=0;k<PRM_Graph[i].size();k++)
    //            {
    //                if(PRM_Graph[i][k] == j)
    //                {
    //                    find_node = 1;
    //                    k = PRM_Graph[i].size();
    //                }
    //            }
    //            if(!find_node)
    //            {
    //                if(InCollision_Edge(space.PRM_Points[i],space.PRM_Points[j]) == 0)
    //                {
    //                    bool observed = 0;
    //                    PRM_Graph[i].push_back(j);

    //                    observed = 0;
    //                    for(uint q=0;q<PRM_Graph[j].size();q++)
    //                    {
    //                        if(PRM_Graph[j][q] == i)
    //                            observed = 1;
    //                    }
    //                    if(!observed)
    //                        PRM_Graph[j].push_back(i);
    //                }
    //            }
    //        }
    //    }
}

bool PRM::InCollision_Node(POINT Robot_pos)
{
    if(Robot_pos.x >space.admissible_area.first.x && Robot_pos.y >space.admissible_area.first.y
        #ifdef PRM_3D
            && Robot_pos.z >space.admissible_area.first.z
        #endif
            )
    {
        if(Robot_pos.x <space.admissible_area.second.x && Robot_pos.y <space.admissible_area.second.y
        #ifdef PRM_3D
                && Robot_pos.z <space.admissible_area.second.z
        #endif
                )
        {
            float thr_2 = (space.Robot_radius + space.Obs_radius)*(space.Robot_radius + space.Obs_radius);
            for(uint i=0;i<space.Obs_pos.size();i++)
            {
                float dist = vec_len( (Robot_pos-space.Obs_pos[i]) );
                if(dist < thr_2 )
                {
                    //            Robot_pos = Robot_last_pos;
                    return 1;
                }
            }

            for(uint i=0;i<border_pos.size();i++)
            {
                float dist = vec_len_2( (Robot_pos-border_pos[i]) );
                if(dist < thr_2 )
                {
                    //            Robot_pos = Robot_last_pos;
                    return 1;
                }
            }
        }
        else
        {
            qDebug("Area2");
            return 1;

        }
    }
    else
    {
        qDebug("Area1");
        return 1;
    }

    return 0;
}

bool PRM::InCollision_Edge(PRM::POINT P0, PRM::POINT P1)
{
    POINT Robot_pos;
    float dist = vec_len( P0 - P1);
    if(dist > PRM_Param.resulotion)
    {
        float m = ceil(dist/PRM_Param.resulotion);
        for(uint i=1;i<m;i++)
        {
            //        Robot_last_pos = Robot_pos;
            Robot_pos = ( 1-((float)i/m) )*P0 + ((float)i/m)*P1;
            if(InCollision_Node(Robot_pos))
                return 1;
        }
    }
    return 0;
}

float PRM::vec_len(PRM::POINT vec)
{
#ifdef PRM_3D
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
#else
    return sqrt(vec.x*vec.x + vec.y*vec.y);
#endif
}

float PRM::vec_len_2(PRM::POINT vec)
{
#ifdef PRM_3D
    return (vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
#else
    return (vec.x*vec.x + vec.y*vec.y);
#endif
}

float PRM::H_dist(PRM::POINT P0, PRM::POINT P1)
{
    POINT _vec = P1-P0;
    return vec_len(_vec);
}

float PRM::H_dist(PRM::POINT P0, std::vector<PRM::POINT> P1)
{
    std::vector<float> dist_vec;
    for(uint i=0;i<P1.size();i++)
    {
        POINT _vec = P1[i]-P0;
        dist_vec.push_back(vec_len(_vec));
    }
    std::sort(dist_vec.begin(),dist_vec.end(),float_raw_compare);

    return dist_vec[0];
}

int PRM::FindLowestFScore()
{
    std::vector<std::pair<float,int> > F_vec;
    std::map<int,float>::iterator ite = F_Score.begin();
    for(;ite!=F_Score.end();ite++)
    {
        if(OpenSet.find(ite->first) != OpenSet.end())
            F_vec.push_back(make_pair(ite->second,ite->first));
    }

    std::sort(F_vec.begin(),F_vec.end(),float_compare_min);
    return F_vec[0].second;
}


//Draw PRM Graph............................................................................................
//std::map<int,std::vector<uint> >::iterator my_ite = PRM_Planning.PRM_Graph.begin();
//for(;my_ite!=PRM_Planning.PRM_Graph.end();my_ite++)
//{
//    for(uint i=0;i< my_ite->second.size();i++)
//    {
//        PointT P0 = CV2PCL(PRM_Planning.space.PRM_Points[my_ite->first]);
//        PointT P1 = CV2PCL(PRM_Planning.space.PRM_Points[my_ite->second[i]]);
//        QByteArray line_str = QString("PRM_Path"+QString::number(my_ite->first)+QString::number(my_ite->second[i])).toLatin1();
//        viewer_thr->DrawLine(P0,P1,PointT(0,0.0,1.0),2,line_str);

//    }
//    QByteArray sph_str = QString("PRM_node"+QString::number(my_ite->first)).toLatin1();
//    viewer_thr->DrawSphere(CV2PCL(PRM_Planning.space.PRM_Points[my_ite->first]),0.05,1,0,0,0,sph_str);
//}
