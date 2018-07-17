#include "potentialfield.h"

PotentialField::PotentialField(QObject *parent) : QObject(parent)
{
    space.Goal_coeff = space.Obs_coeff = space.Start_coeff = space.Border_coeff = 0;
    space.step = 1;

    last_admissible_area.first = last_admissible_area.second = POINT_0;
    border_pos.clear();
}

PotentialField::~PotentialField()
{

}

bool PotentialField::RunPotentialPlanning(float THR,bool create_border_points)
{
    if(create_border_points)
        AddBorderPoints(0.05);
    else
        border_pos.clear();

    last_admissible_area.first = space.admissible_area.first;
    last_admissible_area.second = space.admissible_area.second;

    POINT Force_vec = CalcPlanningForce();
    float force_mag = vec_len(Force_vec);

    if(force_mag > THR)
    {
        if(force_mag > 2*space.Max_Force)
        {
           Force_vec = Force_vec*2*(space.Max_Force/force_mag);
        }

//        qDebug()<<"F_ABS"<<vec_len(Force_vec);
        space.Robot_pos += space.step*Force_vec;
        return 1;
    }
    return 0;
}

POINT PotentialField::CalcPlanningForce()
{
    POINT Attractive(POINT_0),Repulsive(POINT_0),Force_vec(POINT_0);
    POINT pos_tmp;
    float coeff_tmp;
    float dist = 0;
    float force_mag = 0;

    //compute the all obstacle repulsive force..........
    coeff_tmp = 1/space.Obs_radius;
    for(uint i=0;i<space.Obs_pos.size();i++)
    {
        pos_tmp = space.Obs_pos[i];
        dist = vec_len(pos_tmp-space.Robot_pos)+0.01;

        if(dist < space.Obs_radius)
        {
            Repulsive += space.Obs_coeff*( (1/dist) - coeff_tmp)*(1/pow(dist,3))*(space.Robot_pos-pos_tmp);
        }

    }
    force_mag = vec_len(Repulsive);
    if(force_mag>0.4*space.Max_Force)
        Repulsive = Repulsive*(0.4*space.Max_Force/force_mag);
    Force_vec += Repulsive;
    Repulsive = POINT_0;

    //compute the all border repulsive force..........
    coeff_tmp = 1/space.Border_radius;
    for(uint i=0;i<border_pos.size();i++)
    {
        pos_tmp = border_pos[i];
        dist = vec_len(pos_tmp-space.Robot_pos)+0.01;

        if(dist < space.Border_radius)
        {
            Repulsive += space.Border_coeff*( (1/dist) - coeff_tmp)*(1/pow(dist,3))*(space.Robot_pos-pos_tmp);
        }

    }
    force_mag = vec_len(Repulsive);
    if(force_mag>0.4*space.Max_Force)
        Repulsive = Repulsive*(0.4*space.Max_Force/force_mag);
    Force_vec += Repulsive;
    Repulsive = POINT_0;

    //compute the Start Point repulsive force..............
    coeff_tmp = 1/space.Start_radius;
    pos_tmp = space.Start_pos;
    dist = vec_len(pos_tmp-space.Robot_pos)+0.01;

    if(dist < space.Start_radius)
    {
        Repulsive += space.Start_coeff*( (1/dist) - coeff_tmp)*(1/pow(dist,3))*(space.Robot_pos-pos_tmp);
    }
    force_mag = vec_len(Repulsive);
    if(force_mag>0.4*space.Max_Force)
        Repulsive = Repulsive*(0.4*space.Max_Force/force_mag);
    Force_vec += Repulsive;
    Repulsive = POINT_0;

    //compute the Goal Point attractive force..............
    dist = vec_len(space.Goal_pos-space.Robot_pos);

    if(dist > space.Goal_radius)
    {
        Attractive = space.Goal_coeff*(space.Robot_pos-space.Goal_pos);
    }
    force_mag = vec_len(Attractive);
    if(force_mag>space.Max_Force)
        Attractive = Attractive*(space.Max_Force/force_mag);
    Force_vec -= Attractive;
    Attractive = POINT_0;

    return Force_vec;
}

void PotentialField::AddBorderPoints(float step)
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

void PotentialField::AddBorderRandomPoints(int Num)
{
    if(last_admissible_area.first != space.admissible_area.first || last_admissible_area.second != space.admissible_area.second)
    {
        border_pos.clear();

        float Dx,Dy,Dz;
        POINT tmp_POINT_1,tmp_POINT_2;

        Dx = space.admissible_area.second.x - space.admissible_area.first.x;
        Dy = space.admissible_area.second.y - space.admissible_area.first.y;
        Dz = space.admissible_area.second.z - space.admissible_area.first.z;

        // X axis points......
        for(int i=0;i<Num;i++)
        {
            tmp_POINT_1.x = space.admissible_area.first.x;
            tmp_POINT_2.x = space.admissible_area.second.x;
            tmp_POINT_1.y = tmp_POINT_2.y = RAND_NUM*Dy + space.admissible_area.first.y ;
            tmp_POINT_1.z = tmp_POINT_2.z = RAND_NUM*Dz + space.admissible_area.first.z;
            border_pos.push_back(tmp_POINT_1);
            border_pos.push_back(tmp_POINT_2);

            tmp_POINT_1.y = space.admissible_area.first.y;
            tmp_POINT_2.y = space.admissible_area.second.y;
            tmp_POINT_1.x = tmp_POINT_2.x = RAND_NUM*Dx + space.admissible_area.first.x ;
            tmp_POINT_1.z = tmp_POINT_2.z = RAND_NUM*Dz + space.admissible_area.first.z;
            border_pos.push_back(tmp_POINT_1);
            border_pos.push_back(tmp_POINT_2);

            tmp_POINT_1.z = space.admissible_area.first.z;
            tmp_POINT_2.z = space.admissible_area.second.z;
            tmp_POINT_1.y = tmp_POINT_2.y = RAND_NUM*Dy + space.admissible_area.first.y ;
            tmp_POINT_1.x = tmp_POINT_2.x = RAND_NUM*Dx + space.admissible_area.first.x;
            border_pos.push_back(tmp_POINT_1);
            border_pos.push_back(tmp_POINT_2);

        }

    }
}

float PotentialField::vec_len(POINT vec)
{
    return sqrt(vec.dot(vec));
}

