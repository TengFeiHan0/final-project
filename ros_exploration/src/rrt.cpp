#include "rrt.h"

RRT::RRT(QObject *parent) : QObject(parent)
{

}

RRT::~RRT()
{

}

int RRT::PlanPathRRT(void)
{

    RRT_Tree.clear();
    RRT_PATH.clear();
    SMOOTH_PATH.clear();
    ROBOT_SETPOINT.clear();
    BREAK_POINT.clear();
    ALL_PATH.clear();

    RRT_Tree.push_back(pair<POINT,int>(space.start_point,-1));
    Robot_pos = space.start_point;

    AddBorderPoints(0.3);
    last_admissible_area.first = space.admissible_area.first;
    last_admissible_area.second = space.admissible_area.second;

    for(int iter = 0;iter<RRT_Param.Max_rrt_iteration;iter++)
    {
        //        Robot_last_pos = Robot_pos;
        Robot_pos = RRT_Tree[RRT_Tree.size()-1].first;

        Robot_pos += make_random_point(RRT_Param.step_len);

        POINT rand_point = Robot_pos;

        if( InCollision_Node() )
            continue;



        pair<POINT,int> New_Node;
        float minDist = 100000;
        for(uint i=0;i<RRT_Tree.size();i++)
        {
            float dist = vec_len( RRT_Tree[i].first - Robot_pos);
            if(i == 0 || dist<minDist)
            {
                minDist = dist;
                New_Node.first = RRT_Tree[i].first;
                New_Node.second = i;
            }
        }
        if(InCollision_Edge(rand_point,New_Node.first))
            continue;


        //        line(screen,New_Node.first,rand_point,Scalar(200,200,200),2);
        ALL_PATH.push_back(pair<POINT,POINT>(New_Node.first,rand_point));
        New_Node.first = rand_point;
        RRT_Tree.push_back(New_Node);

        float dist = vec_len((space.goal_point - rand_point));

        //                qDebug()<<"dist"<<dist<<"ite"<<iter;


        if(dist < RRT_Param.goal_THR)
        {
            if(InCollision_Edge(rand_point,space.goal_point))
                continue;
            RRT_Tree.push_back(pair<POINT,int>(space.goal_point,RRT_Tree.size()-1));

            vector<POINT> tmp_PATH;
            tmp_PATH.push_back(space.goal_point);
            int k = RRT_Tree[RRT_Tree.size()-1].second;

            while(k>-1)
            {
                tmp_PATH.push_back(RRT_Tree[k].first);
                k=RRT_Tree[k].second;
            }

            for(int q=(tmp_PATH.size()-1);q>=0;q--)
                RRT_PATH.push_back(tmp_PATH[q]);

            return iter;
        }

    }

    return -1;
}

int RRT::SmoothPath(void)
{
    SMOOTH_PATH.clear();
    SMOOTH_PATH.resize(RRT_PATH.size());
    SMOOTH_PATH = RRT_PATH;

    int i = 0,j = 0;

    for(int iter=0;iter<RRT_Param.Max_smooth_iteration;iter++)
    {

        uint m = SMOOTH_PATH.size();
        vector<float> path_len;
        path_len.resize(m);
        path_len[0] = 0;

        for(uint i=1;i<m;i++)
        {
            path_len[i] = vec_len(SMOOTH_PATH[i]-SMOOTH_PATH[i-1]) + path_len[i-1];
        }


        float s1 = ((float)rand()/RAND_MAX) * path_len[m-1];
        float s2 = ((float)rand()/RAND_MAX) * path_len[m-1];
        if(s2 < s1)
        {
            float tmp = s2;
            s2 = s1;
            s1 = tmp;
        }
        for(uint k=1;k<m;k++)
        {
            if(s1 < path_len[k])
            {
                i = k-1;
                break;
            }
        }
        for(uint k=(i+1);k<m;k++)
        {
            if(s2 < path_len[k])
            {
                j = k-1;
                break;
            }
        }


        if(j <= i)
            continue;

        float t1 = (s1 - path_len[i])/(path_len[i+1]-path_len[i]);
        POINT gamma1 = (1 - t1)*SMOOTH_PATH[i] + t1*SMOOTH_PATH[i+1];
        float t2 = (s2 - path_len[j])/(path_len[j+1]-path_len[j]);
        POINT gamma2 = (1 - t2)*SMOOTH_PATH[j] + t2*SMOOTH_PATH[j+1];

        if(InCollision_Edge(gamma1,gamma2))
            continue;

        vector<POINT> new_path(SMOOTH_PATH.begin(),SMOOTH_PATH.begin()+i+1);

        new_path.push_back(gamma1);
        new_path.push_back(gamma2);
        for(uint q=j+1;q<m;q++)
            new_path.push_back(SMOOTH_PATH[q]);

        SMOOTH_PATH.clear();
        SMOOTH_PATH.resize(new_path.size());
        SMOOTH_PATH = new_path;

    }
    return 1;
}

int RRT::Cal_Robot_SetPoints(float max_dist)
{
    POINT last_setpoint = SMOOTH_PATH[0];
    ROBOT_SETPOINT.clear();
    ROBOT_SETPOINT.push_back(last_setpoint);
    for(int i=1;i<SMOOTH_PATH.size();i++)
    {
        float dist = vec_len( SMOOTH_PATH[i-1] - SMOOTH_PATH[i]);
        float m = ceil(dist/RRT_Param.resulotion);
        for(uint j=1;j<m;j++)
        {
            POINT P2 = ( 1-((float)j/m) )*SMOOTH_PATH[i-1] + ((float)j/m)*SMOOTH_PATH[i];

            if(vec_len(last_setpoint-P2) > max_dist)
            {
                last_setpoint = P2;
                ROBOT_SETPOINT.push_back(last_setpoint);
            }
        }
    }

    float dist = vec_len( last_setpoint - SMOOTH_PATH[SMOOTH_PATH.size()-1]);
    if(dist > RRT_Param.goal_THR)
        ROBOT_SETPOINT.push_back(SMOOTH_PATH[SMOOTH_PATH.size()-1]);

    return ROBOT_SETPOINT.size();
}

int RRT::PathBreakPoint(float angle,float dist)
{
    //    POINT MID_Point;
    //    Point_set = SMOOTH_PATH[1];

    float Angle = fabs(cos(angle*(3.1415/180.0)));

    POINT V0,V1;
    vector<POINT> tmp_vec;

    tmp_vec.push_back(SMOOTH_PATH[0]);

    for(int i=2;i<SMOOTH_PATH.size();i++)
    {
        V0 = SMOOTH_PATH[i-1] - SMOOTH_PATH[i-2];
        V1 = SMOOTH_PATH[i] - SMOOTH_PATH[i-1];

        float vec_dot = fabs(V0.dot(V1)/(vec_len(V0)*vec_len(V1)));
        if(vec_dot < Angle)
            tmp_vec.push_back(SMOOTH_PATH[i-1]);
    }
    tmp_vec.push_back(SMOOTH_PATH[SMOOTH_PATH.size()-1]);

    BREAK_POINT.push_back(SMOOTH_PATH[0]);
    for(int i=1;i<tmp_vec.size();i++)
    {
        if(vec_len(tmp_vec[i]-BREAK_POINT[BREAK_POINT.size()-1]) < dist)
            continue;
        BREAK_POINT.push_back(tmp_vec[i]);
    }
}

POINT RRT::make_random_point(float step_len)
{
    POINT rand_point;
    POINT goal = space.goal_point - Robot_pos;

    float theta_0 = atan2(goal.x,goal.y);

    float rand_theta = 2*(0.5-(float)rand()/RAND_MAX)*160.0*(3.1415/180.0) + theta_0;

#ifdef RRT_3D
    float alpha_0 = asin(goal.z/vec_len(goal));
    float rand_alpha = 2.0*(0.5-(float)rand()/RAND_MAX)*90.0*(3.1415/180.0) + alpha_0;
    float r = step_len*cos(rand_alpha);
    rand_point.z = step_len*sin(rand_alpha);
#else
    float r = step_len;
#endif

    rand_point.x = r*sin(rand_theta);
    rand_point.y = r*cos(rand_theta);


    return rand_point;
}

bool RRT::InCollision_Node(void)
{

    if(Robot_pos.x >space.admissible_area.first.x && Robot_pos.y >space.admissible_area.first.y
        #ifdef RRT_3D
            && Robot_pos.z >space.admissible_area.first.z
        #endif
            )
    {
        if(Robot_pos.x <space.admissible_area.second.x && Robot_pos.y <space.admissible_area.second.y
        #ifdef RRT_3D
                && Robot_pos.z <space.admissible_area.second.z
        #endif
                )
        {
            float dist_2 = (space.Robot_radius + space.Obs_radius)*(space.Robot_radius + space.Obs_radius);
            for(uint i=0;i<space.Obs_pos.size();i++)
            {
                float dist = vec_len_2( (Robot_pos-space.Obs_pos[i]) );
                if(dist <  dist_2)
                {
                    //            Robot_pos = Robot_last_pos;
                    return 1;
                }
            }

            for(uint i=0;i<border_pos.size();i++)
            {
                float dist = vec_len_2( (Robot_pos-border_pos[i]) );
                if(dist < dist_2)
                {
                    //            Robot_pos = Robot_last_pos;
                    return 1;
                }
            }
        }
        else
            return 1;
    }
    else
        return 1;

    return 0;
}

float RRT::vec_len(POINT vec)
{
#ifdef RRT_3D
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
#else
    return sqrt(vec.x*vec.x + vec.y*vec.y);
#endif
}

float RRT::vec_len_2(POINT vec)
{
#ifdef RRT_3D
    return (vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
#else
    return (vec.x*vec.x + vec.y*vec.y);
#endif
}

bool RRT::InCollision_Edge(POINT P0, POINT P1)
{

    float dist = vec_len( P0 - P1);
    float m = ceil(dist/RRT_Param.resulotion);
    for(uint i=1;i<m;i++)
    {
        //        Robot_last_pos = Robot_pos;
        Robot_pos = ( 1-((float)i/m) )*P0 + ((float)i/m)*P1;
        if(InCollision_Node())
            return 1;
    }

//    float dist_2 = (space.Robot_radius + space.Border_radius)*(space.Robot_radius + space.Border_radius);
//    for(uint i=0;i<border_pos.size();i++)
//    {
//        float _dist = vec_len_2( (Robot_pos-border_pos[i]) );
//        if(_dist <  dist_2)
//        {
//            //            Robot_pos = Robot_last_pos;
//            return 1;
//        }
//    }

    return 0;
}

void RRT::AddBorderPoints(float step)
{
    if(last_admissible_area.first != space.admissible_area.first || last_admissible_area.second != space.admissible_area.second)
    {
        border_pos.clear();

        float sliding_var_x,sliding_var_y,sliding_var_z;
        POINT tmp_POINT_1,tmp_POINT_2;

        // X axis points......
        sliding_var_y = space.admissible_area.first.y;
        sliding_var_z = space.admissible_area.first.z;
        while(sliding_var_y<space.admissible_area.second.y)
        {
            while(sliding_var_z<space.admissible_area.second.z)
            {
                tmp_POINT_1.x = space.admissible_area.first.x;
                tmp_POINT_2.x = space.admissible_area.second.x;
                tmp_POINT_1.y = tmp_POINT_2.y = sliding_var_y;
                tmp_POINT_1.z = tmp_POINT_2.z = sliding_var_z;

                border_pos.push_back(tmp_POINT_1);
                border_pos.push_back(tmp_POINT_2);
                sliding_var_z +=step;
            }
            sliding_var_y +=step;
            sliding_var_z = space.admissible_area.first.z;
        }

        // Y axis points......
        sliding_var_x = space.admissible_area.first.x;
        sliding_var_z = space.admissible_area.first.z;
        while(sliding_var_x<space.admissible_area.second.x)
        {
            while(sliding_var_z<space.admissible_area.second.z)
            {
                tmp_POINT_1.y = space.admissible_area.first.y;
                tmp_POINT_2.y = space.admissible_area.second.y;
                tmp_POINT_1.x = tmp_POINT_2.x = sliding_var_x;
                tmp_POINT_1.z = tmp_POINT_2.z = sliding_var_z;

                border_pos.push_back(tmp_POINT_1);
                border_pos.push_back(tmp_POINT_2);
                sliding_var_z +=step;

            }
            sliding_var_x +=step;
            sliding_var_z = space.admissible_area.first.z;
        }

        // Z axis points......
        sliding_var_y = space.admissible_area.first.y;
        sliding_var_x = space.admissible_area.first.x;
        while(sliding_var_y<space.admissible_area.second.y)
        {
            while(sliding_var_x<space.admissible_area.second.x)
            {
                tmp_POINT_1.z = space.admissible_area.first.z;
                tmp_POINT_2.z = space.admissible_area.second.z;
                tmp_POINT_1.y = tmp_POINT_2.y = sliding_var_y;
                tmp_POINT_1.x = tmp_POINT_2.x = sliding_var_x;

                border_pos.push_back(tmp_POINT_1);
                border_pos.push_back(tmp_POINT_2);
                sliding_var_x +=step;

            }
            sliding_var_y +=step;
            sliding_var_x = space.admissible_area.first.x;
        }
    }

}


