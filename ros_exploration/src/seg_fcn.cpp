#include "seg_fcn.h"

namespace SEG{

bool dot_product_compare(std::pair<std::pair<int,int>,float> i,std::pair<std::pair<int,int>,float> j)
{
    return i.second>j.second;
}
bool float_compare(std::pair<float,int> i, std::pair<float,int> j)
{
    return i.first>j.first;
}

bool float_compare_min(std::pair<float,int> i, std::pair<float,int> j)
{
    return i.first<j.first;
}

bool double_compare_min(std::pair<double,int> i, std::pair<double,int> j)
{
    return i.first<j.first;
}

bool float_raw_compare(float i, float j)
{
    return i<j;
}
void viewer_init(visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0.93,0.93,0.93);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
}

void add_cloud2viewer(visualization::PCLVisualizer &viewer,PointCloud<PointXYZ>::ConstPtr cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size)
{
    viewer.removePointCloud(pc_name.data());
    visualization::PointCloudColorHandlerCustom<PointXYZ> points_color (cloud,r,g,b);

    viewer.addPointCloud<PointXYZ>(cloud,points_color,pc_name.data());
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,points_size,pc_name.data());
}

void LoadFromeFile(PointCloud<pcl::PointXYZ>::Ptr _cloud,QByteArray fileName)
{
    QFile EXTdata(fileName);

    if (EXTdata.open(QFile::ReadOnly))
    {
        QTextStream in(&EXTdata);
        qDebug()<<"Loadind cloud....";
        QByteArray datain;
        datain=in.readAll().toLatin1();

        QDataStream DataRead(&datain,QIODevice::ReadOnly);

        QByteArray StartPacket;
        int len;
        PointT tmp_point;
        QByteArray EndPacket;

        DataRead >> StartPacket;
        if( QString::compare(StartPacket,"St") == 0 )
        {

            DataRead >> len;
            for(int j=0;j<len;j++)
            {
                float val;
                DataRead >> val;
                tmp_point.x = val;

                DataRead >> val;
                tmp_point.y = val;

                DataRead >> val;
                tmp_point.z = val;

                _cloud->points.push_back(tmp_point);

            }
            DataRead >> EndPacket;

            if( QString::compare(EndPacket,"En") == 0 )
            {
                qDebug()<<"Cloud loaded";
            }
        }

        EXTdata.close();
    }
}

void Save2File(PointCloud<PointXYZ>::Ptr cloud)
{
    QByteArray loggedData;

    QDataStream DataPacket(&loggedData,QIODevice::WriteOnly);

    QByteArray StartPacket = "St";
    int len =cloud->points.size();
    QByteArray EndPacket = "En";

    DataPacket << StartPacket;
    DataPacket << len;

    for(int j=0;j<len;j++)
    {
        float val = 0;

        val = cloud->points[j].x;
        DataPacket << val;

        val = cloud->points[j].y;
        DataPacket << val;

        val = cloud->points[j].z;
        DataPacket << val;

    }

    DataPacket << EndPacket;

    QFile EXTdata("DataLoggedFile");

    if (EXTdata.open(QFile::WriteOnly)) {
        QTextStream out(&EXTdata);
        out << loggedData;
        EXTdata.close();
        qDebug("Cloud Saved");
    }
}

float define_area(visualization::PCLVisualizer &viewer,int disp,bool print,PointCloud<PointXYZ>::Ptr _cloud,PointT _dim,uint divide_method,uint method,uint correction_method,float modify)
{
    PointT dim = _dim;
    int cluster_num = 5;
    float x_min,x_max,y_min,y_max,z_min,z_max;
    x_min = y_min = z_min =  100000000000;
    x_max = y_max = z_max = -100000000000;
    int Cluster_num,X_block_num,Y_block_num,Z_block_num;
    PointT cloud_mean_pos;
    int cluster_mean_size = 0;
    int counter = 0,mean_points = 0,less_mean_counter = 0;
    float max_deviation = 0,mean_deviation = 0;
    float Plane_dist_fit = 0;

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
    if(print)
        cout <<" Len:"<<(x_max-x_min);
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

        while(Cluster_num >200 || mean_points < 20)
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

            property.push_back(cv::Vec3f(x_min,x_max,dim.x));
            property.push_back(cv::Vec3f(y_min,y_max,dim.y));
            property.push_back(cv::Vec3f(z_min,z_max,dim.z));

            X_block_num = (int)ceil((double)(x_max-x_min)/dim.x);
            Y_block_num = (int)ceil((double)(y_max-y_min)/dim.y);
            Z_block_num = (int)ceil((double)(z_max-z_min)/dim.z);

            Cluster_num = int(X_block_num*Y_block_num*Z_block_num);
            mean_points = _cloud->points.size()/Cluster_num;

            if(Cluster_num >200 || mean_points < 20)
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

    int i,j,k;
    std::vector<PointCloud<PointXYZ>::Ptr> cloud_vec;
    cloud_vec.resize(Cluster_num);

    for(int k =0;k<Cluster_num;k++)
        cloud_vec[k] = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    for(int c=0;c<_cloud->points.size();c++)
    {
        i = (int)((_cloud->points[c].x-property[0][0])/property[0][2]);
        j = ((int)X_block_num)*(int)((_cloud->points[c].y-property[1][0])/property[1][2]);
        k = ((int)(X_block_num*Y_block_num))*(int)((_cloud->points[c].z-property[2][0])/property[2][2]);

        cloud_vec[(i+j+k)]->points.push_back(_cloud->points[c]);
    }

    //find the blocks that its height(y) are lower(higher) the the other.......
    std::vector<std::pair<float,int> > block_height;
    for(int c=0;c<Cluster_num;c++)
    {
        if(cloud_vec[c]->points.size() >= (mean_points*0.5))
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
    std::vector<std::pair<pcl::ModelCoefficients,PointCloudT::Ptr> > plane_property;
    PointCloudT::Ptr Nr_cloud(new PointCloudT);
    for(int c=0;c<Cluster_num;c++)
    {
        if(cloud_vec[c]->points.size()<(mean_points*0.6))
            less_mean_counter++;
        else
        {
            PointCloudT::Ptr _cloud_tmp(new PointCloudT);
            float max_dim = max(max(property[0][0],property[0][1]),property[0][2]);
            //            qDebug()<<max_dim/10;
            Plane_dist_fit = max_dim/10;
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

            if(_cloud_tmp->points.size()>(mean_points*0.6))
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
        counter+=cloud_vec[c]->points.size();
    }

    if(method == NORMAL_CAL_METHOD)
    {
        Compute_Normal(viewer,Nr_cloud,60,"Normals");
        add_cloud2viewer(viewer,Nr_cloud,"N_points",50,220,55,3);
    }

    if(method == PLANE_FIT_METHOD)
    {
        // Clustring the Planes.............
        pcl::ModelCoefficients tmp_coeff;
        std::vector<pcl::ModelCoefficients> tmp_coeff_vec;
        tmp_coeff.values.resize(7);
        PointT tmp_point;
        Eigen::Vector3f R_Point;



        for(int k=0;k<plane_property.size();k++)
        {
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

        clustring(tmp_coeff_vec,cluster_num,3);
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

        if(disp < 3)
        {
            //Final correction...............
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

            if((counter-_cloud->points.size()) != 0)
            {
                qDebug()<<"Input_Output contrary"<< (counter-_cloud->points.size());
                return 0;
            }

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
            for(int i=0;i<cluster_num;i++)
            {
                uint each_cluster_point_size = 0,point_must_add =0;
                for(int k=0;k<clustered_coeff_vec[i].size();k++)
                {
                    each_cluster_point_size += plane_property[clustered_coeff_vec[i][k].second].second->points.size();
                }
                point_must_add = (each_cluster_point_size<3000)?((3000-each_cluster_point_size)/(clustered_coeff_vec[i].size()+1)):(mean_points);

                pcl::ModelCoefficients each_block_coeff;
                for(int k=0;k<clustered_coeff_vec[i].size();k++)
                {
                    int each_cluster_points_size = 0;
                    int each_block_size = plane_property[clustered_coeff_vec[i][k].second].second->points.size();
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

                    PointCloudT each_block_corrected_cloud;
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

                        each_block_corrected_cloud.points.push_back(each_block_point);


                        each_block_center.x += each_block_point.x;
                        each_block_center.y += each_block_point.y;
                        each_block_center.z += each_block_point.z;

                        each_block_min.x = min(each_block_min.x,each_block_point.x);
                        each_block_min.y = min(each_block_min.x,each_block_point.y);
                        each_block_min.z = min(each_block_min.x,each_block_point.z);

                        each_block_max.x = max(each_block_max.x,each_block_point.x);
                        each_block_max.y = max(each_block_max.x,each_block_point.y);
                        each_block_max.z = max(each_block_max.x,each_block_point.z);

                    }

                    each_block_center.x /= (float)each_cluster_points_size;
                    each_block_center.y /= (float)each_cluster_points_size;
                    each_block_center.z /= (float)each_cluster_points_size;


                    for(int p=0;p<each_block_corrected_cloud.points.size();p++)
                        _cloud->points.push_back(each_block_corrected_cloud.points[p]);

                    if(pointPlaneDist(each_block_coeff,each_block_center) < 0.3)
                    {

                        each_block_coeff.values[3] = each_block_center.x;
                        each_block_coeff.values[4] = each_block_center.y;
                        each_block_coeff.values[5] = each_block_center.z;




                        float x_len,y_len,z_len;
                        x_len = each_block_max.x-each_block_min.x;
                        y_len = each_block_max.y-each_block_min.y;
                        z_len = each_block_max.z-each_block_min.z;
                        for(int p=0;p<point_must_add;p++)
                        {
                            PointT rand_point,extra_point;
                            rand_point.x = each_block_center.x + x_len*0.5*RAND_NUM*0.8;
                            rand_point.y = each_block_center.y + y_len*0.5*RAND_NUM*0.8;
                            rand_point.z = each_block_center.z+ z_len*0.5*RAND_NUM*0.8;

                            if(correction_method == GLOBAL_MEAN_CORRECT_METHOD)
                                extra_point = findPointReflection_2(each_block_coeff,rand_point);
                            else if(correction_method == LOCAL_MEAN_CORRECT_METHOD)
                                extra_point = findPointReflection_2(clustered_coeff_vec[i][k].first,rand_point);

                            _cloud->points.push_back(extra_point);
                            //                    each_cluster_points_size--;
                        }
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
        for(int i=0;i<Wrong_pointCloud->points.size();i++)
        {
            _cloud->points.push_back(Wrong_pointCloud->points[i]);
        }
        //exhibit the Pricipal Axes................
        if(disp > 0)
        {
            for(int i=0;i<cluster_num;i++)
            {
                pcl::ModelCoefficients cylinder_coeff;
                cylinder_coeff.values.resize(7);
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

                    //                qDebug()<<"cyl"<<i<<cloud_PA[i].x<<cloud_PA[i].y<<cloud_PA[i].z;

                    //                viewer.removeShape(cyl_str.data());
                    viewer.addCylinder(cylinder_coeff,cyl_str.data());
                    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,(i*100+10)/255.,(i*100+20)/255.,(i*100+30)/255.,cyl_str.data());
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
                    QByteArray Plane_Nr_str = QString("Plane_Nr"+QString::number(i)+QString::number(k)).toLatin1();
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

                    draw_plane(viewer,tmp_coeff,Plane_str.data());

                    viewer.addPointCloudNormals<PointXYZ,Normal>(Plane_center_cloud,Plane_normal,1,0.06,Plane_Nr_str.data());

                    QByteArray Plane_points_str = QString("CPlane_points"+QString::number(i)+QString::number(k)).toLatin1();
                    //                    add_cloud2viewer(viewer,plane_property[clustered_coeff_vec[i][k].second].second,Plane_points_str.data(),k*100+50,k*100,k*80,5);

                }
                if(disp >3)
                {
                    vector<int> outlier_vec(outlier[i].begin(),outlier[i].end());
                    for(int k=0;k<outlier_vec.size();k++)
                    {
                        QByteArray Plane_str = QString("CPlane"+QString::number(i)+QString::number(outlier_vec[k])).toLatin1();
                        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.,i*80/255.0,0.,Plane_str.data());
                    }

                    //                    vector<int> miss_understand_vec(miss_understand[i].begin(),miss_understand[i].end());
                    //                    for(int k=0;k<miss_understand_vec.size();k++)
                    //                    {
                    //                        QByteArray Plane_str = QString("CPlane"+QString::number(i)+QString::number(miss_understand_vec[k])).toLatin1();
                    //                        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.,0,1.,Plane_str.data());
                    //                    }
                }
            }

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

                    draw_plane(viewer,tmp_coeff,Plane_str.data());

                    viewer.addPointCloudNormals<PointXYZ,Normal>(Plane_center_cloud,Plane_normal,1,0.06,Plane_Nr_str.data());
                }
            }
        }

    }



    if(print)
        cout<<" mean_p:"<<mean_points<<" less_mean:"<<less_mean_counter<<"/"<<Cluster_num<<" Points:"<< _cloud->points.size();

    //    cout<<" mean_c:"<<cluster_mean_size<<"  mean_p:"<<mean_points<<" less_mean:"<<less_mean_counter<<"/"<<Cluster_num<<" Nr_points:"<<Nr_cloud->points.size()<<" All_points:"<< _cloud->points.size();

    return max_deviation;

    //    return Plane_dist_fit;

}

void remove_outlier(PointCloud<PointXYZ>::Ptr _cloud,PointCloud<pcl::PointXYZ>::Ptr _cloud_filtered,int mean_k,float thr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (_cloud);
    sor.setMeanK (mean_k);
    sor.setStddevMulThresh (thr);
    sor.filter (*cloud_filtered);

    _cloud_filtered->clear();
    _cloud_filtered->points.resize(cloud_filtered->points.size());
    _cloud_filtered->points = cloud_filtered->points;
}

void plane_fit(PointCloudT::Ptr cloud,float fit_dist,uint ITE,std::vector<std::pair<pcl::ModelCoefficients,PointCloudT::Ptr> > *plane_property)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


    _cloud->points.resize(cloud->points.size());
    _cloud->points = cloud->points;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (ITE);
    seg.setDistanceThreshold (fit_dist);

    int nr_points = (int) _cloud->points.size ();
    while (_cloud->points.size () > 0.1* nr_points && _cloud->points.size () >10)
    {
        std::pair<pcl::ModelCoefficients,pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp_pair;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::ModelCoefficients coefficients;
        // Segment the largest planar component from the remaining cloud

        seg.setInputCloud (_cloud);
        seg.segment (*inliers,coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        tmp_pair.first = coefficients;
        tmp_pair.second = cloud_plane;

        plane_property->push_back(tmp_pair);

        extract.setNegative (true);
        extract.filter (*cloud_f);
        *_cloud = *cloud_f;
    }
    //        if(nr_points>500)
    //            qDebug()<<_cloud->points.size ()<<"O_F"<<nr_points<<" "<<0.1* nr_points;
}

int cloud_segment(visualization::PCLVisualizer &viewer,bool disp,PointCloudT::Ptr cloud_filtered,float fit_dist)
{
    std::vector<std::pair<pcl::ModelCoefficients,PointCloudT::Ptr> > plane_property;
    plane_fit(cloud_filtered,fit_dist,100,&plane_property);
    pcl::ModelCoefficients tmp_coeff;
    tmp_coeff.values.resize(11);

    if(disp)
    {
        PointT tmp_point;
        for(int k=0;k<plane_property.size();k++)
        {
            QByteArray Plane_str = QString("Plane"+QString::number(k)).toLatin1();
            QByteArray Plane_cloud_str = QString("Plane_CL"+QString::number(k)).toLatin1();
            add_cloud2viewer(viewer,plane_property[k].second,Plane_str.data(),k*100+50,k*100,k*80,5);
            viewer.removeShape(Plane_str.data());
            {
                tmp_coeff.values[0] = plane_property[k].first.values[0];
                tmp_coeff.values[1] = plane_property[k].first.values[1];
                tmp_coeff.values[2] = plane_property[k].first.values[2];

                tmp_point = mean_cloud(plane_property[k].second);
                tmp_coeff.values[3] = tmp_point.x;
                tmp_coeff.values[4] = tmp_point.y;
                tmp_coeff.values[5] = tmp_point.z;

                tmp_coeff.values[6] = 0.8;
                tmp_coeff.values[7] = 0.8;

                tmp_coeff.values[8] = (210+k*10);
                tmp_coeff.values[9] = (200+k*10);
                tmp_coeff.values[10] = (190+k*10);
            }
            draw_plane(viewer,tmp_coeff,Plane_str.data());
            add_cloud2viewer(viewer,plane_property[k].second,Plane_cloud_str.data(),100*k,80*k,60*k,5);
        }
    }
    return plane_property.size();
}

void search_Area(PointCloud<PointXYZ>::Ptr _cloud,PointCloud<pcl::PointXYZ>::Ptr selected_cloud,PointXYZ curr_POS,float dist)
{
    for(int i=0;i<_cloud->points.size();i++)
    {

        PointT dist_vec;
        dist_vec.x = _cloud->points[i].x - curr_POS.x;
        dist_vec.y = _cloud->points[i].y - curr_POS.y;
        dist_vec.z = _cloud->points[i].z - curr_POS.z;

        //        if(fabs(query_point - robot_pos)< dist && ((heading)*(robot_pos - query_point)) < 1.0*dist )
        if(vec_mag(dist_vec)< dist )
        {
            selected_cloud->points.push_back(_cloud->points[i]);
        }
    }
    //    qDebug()<<"cloud size::"<<selected_cloud->points.size();
}

void draw_plane(visualization::PCLVisualizer &viewer,pcl::ModelCoefficients coeff,char* ID)
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

    viewer.addCube(T,Q,coeff.values[6],coeff.values[7],0.0001,ID);

    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                       pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,ID);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,coeff.values[8]/255.,coeff.values[9]/255.,coeff.values[10]/255.,ID);
}

PointT mean_cloud(PointCloudT::Ptr cloud)
{
    float x=0,y=0,z=0;
    float sz = (float)cloud->points.size();
    for(int i=0;i<cloud->points.size();i++)
    {
        x = x + cloud->points[i].x;
        y = y + cloud->points[i].y;
        z = z + cloud->points[i].z;
    }
    x = x/sz;
    y = y/sz;
    z = z/sz;
    return PointT(x,y,z);
}

void clustring(std::vector<pcl::ModelCoefficients> &_input,uint Cluster_num,uint col_num)
{
    int clusterCount = _input.size();
    int col_size = col_num;
    uint label_cell = _input[0].values.size()-1;
    cv::Mat points(clusterCount,col_size, CV_32F);
    cv::Mat labels;
    cv::Mat centers(clusterCount, 1, points.type());

    //    PointT goal = PointT(_input[i].values[0],_input[i].values[1],_input[i].values[2]);
    //    float theta_0 = atan2(goal.x,goal.y);
    //    float alpha_0 = asin(goal.z/vec_mag(goal));
    //    points.at<float>(i,0) = alpha_0;
    //    points.at<float>(i,1) = theta_0;

    for(int i=0;i<clusterCount;i++)
    {
        for(int j=0;j<col_size;j++)
            points.at<float>(i,j) = _input[i].values[j];
    }
    cv::kmeans(points, Cluster_num, labels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 20, 0.01), 20, cv::KMEANS_PP_CENTERS, centers);

    for(int i=0;i<labels.rows;i++)
    {
        _input[i].values[label_cell] = (float)(labels.at<int>(i,0));
    }
}

void Compute_Normal(visualization::PCLVisualizer &viewer,PointCloudT::Ptr cloud,uint K,char* ID)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    if(K>cloud->points.size())
        normalEstimation.setKSearch(K>cloud->points.size());
    else
        normalEstimation.setKSearch(K);
    //        normalEstimation.setRadiusSearch(0.9);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*cloud_normals);


    // Object for storing the CVFH descriptors.
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);
    // CVFH estimation object.
    pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(cloud);
    cvfh.setInputNormals(cloud_normals);
    cvfh.setSearchMethod(kdtree);
    // Set the maximum allowable deviation of the normals,
    // for the region segmentation step.
    cvfh.setEPSAngleThreshold(1.0 / 180.0 * M_PI); // 5 degrees.
    // Set the curvature threshold (maximum disparity between curvatures),
    // for the region segmentation step.
    cvfh.setCurvatureThreshold(1.0);
    // Set to true to normalize the bins of the resulting histogram,
    // using the total number of points. Note: enabling it will make CVFH
    // invariant to scale just like VFH, but the authors encourage the opposite.
    cvfh.setNormalizeBins(false);
    cvfh.compute(*descriptors);




    viewer.removeShape(ID);
    viewer.addPointCloudNormals<PointXYZ,Normal>(cloud,cloud_normals,1,0.02,ID);
}


void Knn(PointT query,PointCloudT::Ptr data_set,int N)
{
    float dist = 0;
    std::set<PointT> search_area;
    //    for(int i=0;i<data_set->points.size();i++)
    //        search_area.insert(data_set->points[])
}

PointT find2planeDist(pcl::ModelCoefficients p1,PointT p2)
{
    float D1,D2;
    D1 = -(p1.values[0]*p1.values[3] + p1.values[1]*p1.values[4] + p1.values[2]*p1.values[5]);
    D2 = -(p1.values[0]*p2.x + p1.values[1]*p2.y + p1.values[2]*p2.z);
    float plane_normal_mag = sqrt(pow(p1.values[0],2)+pow(p1.values[1],2)+pow(p1.values[2],2));
    float dist = 1;



    //    if(P2P.dot(Eigen::Vector3f(p2.x,p2.y,p2.z))>0)
    if(D2>D1)
        dist = (fabs(D2-D1)/plane_normal_mag);
    else
        dist = -(fabs(D2-D1)/plane_normal_mag);

    PointT plane_normal(dist*p1.values[0]/plane_normal_mag,dist*p1.values[1]/plane_normal_mag,dist*p1.values[2]/plane_normal_mag);
    return plane_normal;
}
float pointPlaneDist(pcl::ModelCoefficients p1,PointT p2)
{
    float D1,D2;
    D1 = -(p1.values[0]*p1.values[3] + p1.values[1]*p1.values[4] + p1.values[2]*p1.values[5]);
    D2 = -(p1.values[0]*p2.x + p1.values[1]*p2.y + p1.values[2]*p2.z);
    float plane_normal_mag = sqrt(pow(p1.values[0],2)+pow(p1.values[1],2)+pow(p1.values[2],2));
    float dist = (fabs(D2-D1)/plane_normal_mag);
    return dist;
}
float pointPlaneDist_2(pcl::ModelCoefficients p1,PointT p2)
{
    float D1,D2;
    D1 = p1.values[3];
    D2 = -(p1.values[0]*p2.x + p1.values[1]*p2.y + p1.values[2]*p2.z);
    float plane_normal_mag = sqrt(pow(p1.values[0],2)+pow(p1.values[1],2)+pow(p1.values[2],2));
    float dist = (fabs(D2-D1)/plane_normal_mag);
    return dist;
}

float ParallelPlaneDist(pcl::ModelCoefficients p1,pcl::ModelCoefficients p2)
{
    float D1,D2;
    D1 = p1.values[3];
    D2 = p2.values[3];
    float plane_normal_mag = sqrt(pow(p1.values[0],2)+pow(p1.values[1],2)+pow(p1.values[2],2));
    float dist = (fabs(D2-D1)/plane_normal_mag);
    return dist;
}

void correct_cloud_scale(PointCloudT::Ptr _cloud,float scale)
{
    for(int i=0;i<_cloud->points.size();i++)
    {
        _cloud->points[i].x = _cloud->points[i].x*scale;
        _cloud->points[i].y = _cloud->points[i].y*scale;
        _cloud->points[i].z = _cloud->points[i].z*scale;
    }
}

float vec_mag(PointT vec)
{

    //    Eigen::Vector3f V(vec.x,vec.y,vec.z);

    //    return sqrt(V.dot(V));

    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

float vec_mag(Eigen::Vector3f V)
{

    //    return sqrt(V.dot(V));
    return sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
}
float vec_mag(cv::Point3f vec)
{

    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

float vec_mag_2(cv::Point3f vec)
{

    return (vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

float vec_mag_2(PointT vec)
{

    return (vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

void cloud_classifyer(std::vector<std::set<int> > &outlier, std::vector<std::vector<std::pair<pcl::ModelCoefficients,int> > > &clustered_coeff_vec, std::vector<pcl::ModelCoefficients> &corrected_clustered_coeff_vec, std::vector<PointT> &cloud_PA, std::vector<PointT> &cluster_centers,bool remove_outlier_flag)
{
    cv::Mat trainingData;
    cv::Mat trainingClasses;
    cv::KNearest knn;
    cv::Mat testData(1, 3, CV_32FC1);
    int local_counter = 0;
    int cluster_num = clustered_coeff_vec.size();

    //Put all center of plane in an array and prepair data for K-NN.........
    vector<int> train_data_calss;
    for(int i=0;i<cluster_num;i++)
    {
        int class_size = clustered_coeff_vec[i].size();
        cv::Mat tmp_mat(1, 3, CV_32FC1);
        for(int k=0;k<class_size;k++)
        {
            if(outlier[i].find(k) == outlier[i].end())
            {
                tmp_mat.at<float>(0,0) = clustered_coeff_vec[i][k].first.values[3];
                tmp_mat.at<float>(0,1) = clustered_coeff_vec[i][k].first.values[4];
                tmp_mat.at<float>(0,2) = clustered_coeff_vec[i][k].first.values[5];

                trainingData.push_back(tmp_mat);
                //                trainingClasses.push_back((float)(i));
                trainingClasses.push_back((float)(local_counter));
                train_data_calss.push_back(i);

                local_counter++;
            }

        }
    }

    knn.train(trainingData, trainingClasses);
    int K_NN = 1;
    cv::Mat neighborResponse(1, K_NN, CV_32FC1);

    for(int i=0;i<cluster_num;i++)
    {
        pcl::ModelCoefficients tmp_coeff;
        tmp_coeff.values.resize(7);
        vector<int> outlier_vec(outlier[i].begin(),outlier[i].end());
        cv::Mat tmp_mat(1, 3, CV_32FC1);
        for(int k=0;k<outlier_vec.size();k++)
        {

            testData.at<float>(0,0) = clustered_coeff_vec[i][outlier_vec[k]].first.values[3];
            testData.at<float>(0,1) = clustered_coeff_vec[i][outlier_vec[k]].first.values[4];
            testData.at<float>(0,2) = clustered_coeff_vec[i][outlier_vec[k]].first.values[5];

            int _class = (int)(knn.find_nearest(testData,K_NN,0,0,&neighborResponse));

            _class = (int)(neighborResponse.at<float>(0,0));

            tmp_coeff.values[3] = trainingData.row(_class).at<float>(0,0);
            tmp_coeff.values[4] = trainingData.row(_class).at<float>(0,1);
            tmp_coeff.values[5] = trainingData.row(_class).at<float>(0,2);

            _class = train_data_calss[_class];

            tmp_coeff.values[0] = cloud_PA[_class].x;
            tmp_coeff.values[1] = cloud_PA[_class].y;
            tmp_coeff.values[2] = cloud_PA[_class].z;

            tmp_coeff.values[3] = -(tmp_coeff.values[0]*tmp_coeff.values[3] + tmp_coeff.values[1]*tmp_coeff.values[4] + tmp_coeff.values[2]*tmp_coeff.values[5]);

            PointT Dist_vec = findPointReflection(tmp_coeff,PointT(clustered_coeff_vec[i][outlier_vec[k]].first.values[3],
                    clustered_coeff_vec[i][outlier_vec[k]].first.values[4],
                    clustered_coeff_vec[i][outlier_vec[k]].first.values[5]));

            //            PointT Dist_vec = find2planeDist(tmp_coeff,PointT(clustered_coeff_vec[i][outlier_vec[k]].first.values[3],
            //                    clustered_coeff_vec[i][outlier_vec[k]].first.values[4],
            //                    clustered_coeff_vec[i][outlier_vec[k]].first.values[5]));

            tmp_coeff.values[3] = Dist_vec.x;
            tmp_coeff.values[4] = Dist_vec.y;
            tmp_coeff.values[5] = Dist_vec.z;

            //            tmp_coeff.values[3] = clustered_coeff_vec[i][outlier_vec[k]].first.values[3]+Dist_vec.x;
            //            tmp_coeff.values[4] = clustered_coeff_vec[i][outlier_vec[k]].first.values[4]+Dist_vec.y;
            //            tmp_coeff.values[5] = clustered_coeff_vec[i][outlier_vec[k]].first.values[5]+Dist_vec.z;

            //                    qDebug()<<"PL"<< (tmp_coeff.values[3]-cluster_centers[_class].x)<<
            //                                                                                       (tmp_coeff.values[4]-cluster_centers[_class].y)<<
            //                                                                                                                                         (tmp_coeff.values[5]-cluster_centers[_class].z);

            tmp_mat.at<float>(0,0) = tmp_coeff.values[3];
            tmp_mat.at<float>(0,1) = tmp_coeff.values[4];
            tmp_mat.at<float>(0,2) = tmp_coeff.values[5];

            trainingData.push_back(tmp_mat);
            trainingClasses.push_back((float)(local_counter));
            train_data_calss.push_back(_class);
            local_counter++;

            knn.train(trainingData, trainingClasses);


            if(remove_outlier_flag)
            {
                std::pair<pcl::ModelCoefficients,int> pair_tmp;
                pair_tmp.first = tmp_coeff;
                pair_tmp.second = clustered_coeff_vec[i][outlier_vec[k]].second;
                for(int j=0;j<6;j++)
                    clustered_coeff_vec[i][outlier_vec[k]].first.values[j] = 0;
                clustered_coeff_vec[_class].push_back(pair_tmp);


            }

            corrected_clustered_coeff_vec.push_back(tmp_coeff);
        }
    }
    if(remove_outlier_flag)
    {
        bool M_Ob = 0;
        for(int i=0;i<cluster_num;i++)
        {
            int calss_size = clustered_coeff_vec[i].size();
            for(int k=0;k<calss_size;k++)
            {
                M_Ob = 0;
                for(int j=0;j<6;j++)
                {
                    if(clustered_coeff_vec[i][k].first.values[j] !=0 )
                        M_Ob = 1;

                    if(j==5 && M_Ob == 0)
                    {
                        clustered_coeff_vec[i].erase(clustered_coeff_vec[i].begin()+k);
                        k--;
                        calss_size--;
                    }
                }
            }

        }
    }
}

seg_str final_segmentation(visualization::PCLVisualizer &viewer,bool disp,PointCloudT::Ptr _cloud,float fit_dist,float gap)
{
    seg_str out_put;
    plane_fit(_cloud,fit_dist,100,&out_put.plane_property);
    pcl::ModelCoefficients tmp_coeff;
    tmp_coeff.values.resize(11);
    PointT center_cloud(0,0,0);
    int cloud_num = 0;


    //calculate the center of input point cloud...............
    for(int i=0;i<out_put.plane_property.size();i++)
    {
        PointCloudT::Ptr cloud_tmp = out_put.plane_property[i].second;
        std::pair<pcl::ModelCoefficients,PointCloudT::Ptr> plane_prop_tmp;

        for(int k=0;k<cloud_tmp->points.size();k++)
        {



            center_cloud.x += cloud_tmp->points[k].x;
            center_cloud.y += cloud_tmp->points[k].y;
            center_cloud.z += cloud_tmp->points[k].z;
            cloud_num++;
        }
    }
    center_cloud.x /= cloud_num;
    center_cloud.y /= cloud_num;
    center_cloud.z /= cloud_num;
    //Determin the admissible region By Red planes............
    std::vector<pcl::ModelCoefficients> plane_gap_vec;
    std::pair<std::pair<int,int>,float> plane_normal_dot;
    plane_normal_dot.second = -100000;
    plane_gap_vec.resize(out_put.plane_property.size());
    {
        for(int i=0;i<out_put.plane_property.size();i++)
        {
            plane_gap_vec[i].values.resize(6);

            plane_gap_vec[i].values[0] = out_put.plane_property[i].first.values[0];
            plane_gap_vec[i].values[1] = out_put.plane_property[i].first.values[1];
            plane_gap_vec[i].values[2] = out_put.plane_property[i].first.values[2];
            plane_gap_vec[i].values[3] = out_put.plane_property[i].first.values[3];

            PointT reflection = findPointReflection(plane_gap_vec[i],center_cloud);

            PointT base_vec;
            base_vec.x = center_cloud.x - reflection.x;
            base_vec.y = center_cloud.y - reflection.y;
            base_vec.z = center_cloud.z - reflection.z;

            base_vec.x *= (gap/vec_mag(base_vec));
            base_vec.y *= (gap/vec_mag(base_vec));
            base_vec.z *= (gap/vec_mag(base_vec));

            plane_gap_vec[i].values[3] = reflection.x + base_vec.x;
            plane_gap_vec[i].values[4] = reflection.y + base_vec.y;
            plane_gap_vec[i].values[5] = reflection.z + base_vec.z;

            // determin Parallel Planes..........
            Eigen::Vector3f V1(out_put.plane_property[i].first.values[0],out_put.plane_property[i].first.values[1],out_put.plane_property[i].first.values[2]);

            for(int j=i+1;j<out_put.plane_property.size();j++)
            {
                Eigen::Vector3f V2(out_put.plane_property[j].first.values[0],out_put.plane_property[j].first.values[1],out_put.plane_property[j].first.values[2]);
                float dot_tmp = V2.dot(V1)/(vec_mag(V1)*vec_mag(V2));
                dot_tmp = (float)fabs((double)(dot_tmp));

                if(dot_tmp>plane_normal_dot.second)
                {
                    plane_normal_dot.first.first = i;
                    plane_normal_dot.first.second = j;
                    plane_normal_dot.second = dot_tmp;
                }
            }

        }
    }

    PointT P1(0,0,0),P2(0,0,0),SetPoint(0,0,0);
    float admissible_Len = 0;
    P1 = findPointReflection_2(plane_gap_vec[plane_normal_dot.first.first],center_cloud);
    P2 = findPointReflection_2(plane_gap_vec[plane_normal_dot.first.second],center_cloud);

    SetPoint.x = (P1.x+P2.x)/2;
    SetPoint.y = (P1.y+P2.y)/2;
    SetPoint.z = (P1.z+P2.z)/2;

    admissible_Len = vec_mag(PointT((P1.x-P2.x),(P1.y-P2.y),(P1.z-P2.z)));

    //display the planes and related point clouds..............
    if(disp)
    {
        //        viewer.addSphere(SetPoint,0.0,0.9,0.2,0.5,"SetPoint_POS");
        PointT tmp_point;
        for(int k=0;k<out_put.plane_property.size();k++)
        {
            QByteArray Plane_str = QString("Plane"+QString::number(k)).toLatin1();
            QByteArray Plane_gap_str = QString("Plane_gap"+QString::number(k)).toLatin1();
            QByteArray Plane_cloud_str = QString("Plane_CL"+QString::number(k)).toLatin1();
            {
                tmp_coeff.values[0] = plane_gap_vec[k].values[0];
                tmp_coeff.values[1] = plane_gap_vec[k].values[1];
                tmp_coeff.values[2] = plane_gap_vec[k].values[2];

                tmp_coeff.values[3] = plane_gap_vec[k].values[3];;
                tmp_coeff.values[4] = plane_gap_vec[k].values[4];;
                tmp_coeff.values[5] = plane_gap_vec[k].values[5];;

                tmp_coeff.values[6] = 0.8;
                tmp_coeff.values[7] = 0.8;

                if(k == plane_normal_dot.first.first || k == plane_normal_dot.first.second)
                {
                    tmp_coeff.values[8] = 0;
                    tmp_coeff.values[9] = 210+k*10;
                    tmp_coeff.values[10] = 0;
                }
                else
                {
                    tmp_coeff.values[8] = 210+k*10;
                    tmp_coeff.values[9] = 0;
                    tmp_coeff.values[10] = 0;
                }

                draw_plane(viewer,tmp_coeff,Plane_gap_str.data());
            }
            {
                tmp_coeff.values[0] = out_put.plane_property[k].first.values[0];
                tmp_coeff.values[1] = out_put.plane_property[k].first.values[1];
                tmp_coeff.values[2] = out_put.plane_property[k].first.values[2];

                tmp_point = mean_cloud(out_put.plane_property[k].second);
                tmp_coeff.values[3] = tmp_point.x;
                tmp_coeff.values[4] = tmp_point.y;
                tmp_coeff.values[5] = tmp_point.z;

                tmp_coeff.values[6] = 0.8;
                tmp_coeff.values[7] = 0.8;

                tmp_coeff.values[8] = (210+k*10);
                tmp_coeff.values[9] = (200+k*10);
                tmp_coeff.values[10] = (190+k*10);

                draw_plane(viewer,tmp_coeff,Plane_str.data());
                add_cloud2viewer(viewer,out_put.plane_property[k].second,Plane_cloud_str.data(),100*k,80*k,60*k,5);
            }
        }
    }

    out_put.SetPoint = SetPoint;
    out_put.admissible_Len = admissible_Len;

    return out_put;

}

PointT findPointReflection(pcl::ModelCoefficients _plane,PointT _point)
{

    float K = -(_plane.values[0]*_point.x + _plane.values[1]*_point.y + _plane.values[2]*_point.z + _plane.values[3])
            /(float)(pow(_plane.values[0],2)+pow(_plane.values[1],2)+pow(_plane.values[2],2));
    PointT reflection;
    reflection.x = (_plane.values[0]*K) + _point.x;
    reflection.y = (_plane.values[1]*K) + _point.y;
    reflection.z = (_plane.values[2]*K) + _point.z;

    return reflection;
}

PointT findPointReflection_2(pcl::ModelCoefficients _plane,PointT _point)
{

    float D = -(_plane.values[0]*_plane.values[3] + _plane.values[1]*_plane.values[4] + _plane.values[2]*_plane.values[5]);
    float K = -(_plane.values[0]*_point.x + _plane.values[1]*_point.y + _plane.values[2]*_point.z + D)
            /(float)(pow(_plane.values[0],2)+pow(_plane.values[1],2)+pow(_plane.values[2],2));
    PointT reflection;
    reflection.x = (_plane.values[0]*K) + _point.x;
    reflection.y = (_plane.values[1]*K) + _point.y;
    reflection.z = (_plane.values[2]*K) + _point.z;

    return reflection;
}

void generate_sample_pointcloud(PointCloud<PointXYZ>::Ptr cloud,PointXYZ start_point,PointXYZ end_point, uint cloud_size,float noise_cov)
{
    PointXYZ sample_point;

    float d_x = (end_point.x - start_point.x)/2.0;
    float d_y = (end_point.y - start_point.y)/2.0;
    float d_z = (end_point.z - start_point.z)/2.0;

    float x_a = (end_point.x + start_point.x)/2.0;
    float y_a = (end_point.y + start_point.y)/2.0;
    float z_a = (end_point.z + start_point.z)/2.0;

    //    // floor's points cloud
    //    if(omit.z !=0)
    //    {
    for (size_t i = 0; i < cloud_size; i++)
    {

        sample_point.x = x_a + d_x*RAND_NUM + noise_cov*RAND_NUM;
        sample_point.y = y_a + d_y*RAND_NUM + noise_cov*RAND_NUM;
        sample_point.z = z_a + d_z*RAND_NUM + noise_cov*RAND_NUM;


        cloud->points.push_back(sample_point);
        //        qDebug()<<"1"<<i;
    }
    //    }

    //    // right wall's points cloud
    //    if(omit.y !=0)
    //    {
    //        for (size_t i = 0; i < cloud_size; i++)
    //        {

    //            sample_point.x = x_a + d_x*RAND_NUM;
    //            sample_point.y = end_point.y + noise_cov*RAND_NUM;
    //            sample_point.z = z_a + d_z*RAND_NUM;

    //            cloud->points.push_back(sample_point);
    //            //        qDebug()<<"1"<<i;
    //        }
    //    }

    //    // left wall's points cloud
    //    if(omit.x !=0)
    //    {
    //        for (size_t i = 0; i < cloud_size; i++)
    //        {

    //            sample_point.x = x_a + d_x*RAND_NUM;
    //            sample_point.y = start_point.y + noise_cov*RAND_NUM;
    //            sample_point.z = z_a + d_z*RAND_NUM;

    //            cloud->points.push_back(sample_point);
    //        }
    //    }
}


bool navigation_fcn(PointT *SetPointForPub, PointT Robot_POS, PointT SetPoint_filtered, seg_str Segmentation_OutPut, float SetPointDeviation, int *start_nav
                    , float SETPOINT_VICINITY_THR, float HOLD_POS_TIME, float loop_time, uint *nav_counter)
{

    if(SetPointDeviation <= 0.1 && *start_nav == 0)
    {
        *start_nav = 1;
        *SetPointForPub = PointT(SetPoint_filtered.x,Robot_POS.y,Robot_POS.z+1);
        return 0;
    }
    else if(*start_nav >= 1)
    {
        PointT Dist2SetPoint_vec;
        float min_dist = 10000000;
        for(int i=0;i<Segmentation_OutPut.plane_property.size();i++)
        {
            float set2PlaneDist = pointPlaneDist_2(Segmentation_OutPut.plane_property[i].first,*SetPointForPub);
            min_dist = min(min_dist,set2PlaneDist);

            if( set2PlaneDist < 0.5)
            {
                PointT reflect_point = findPointReflection(Segmentation_OutPut.plane_property[i].first,Robot_POS);
                PointT Robot_ref_vec;

                Robot_ref_vec.x = Robot_POS.x - reflect_point.x;
                Robot_ref_vec.y = Robot_POS.y - reflect_point.y;
                Robot_ref_vec.z = Robot_POS.z - reflect_point.z;

                float mag = vec_mag(Robot_ref_vec);

                Robot_ref_vec.x *= (0.8/mag);
                Robot_ref_vec.y *= (0.8/mag);
                Robot_ref_vec.z *= (0.8/mag);

                SetPointForPub->x = reflect_point.x + Robot_ref_vec.x;
                SetPointForPub->y = reflect_point.y + Robot_ref_vec.y;
                SetPointForPub->z = reflect_point.z + Robot_ref_vec.z;

                *start_nav = 2;
                i = Segmentation_OutPut.plane_property.size()+1;
            }

        }
        if(min_dist > 1.0 && *start_nav == 2)
            *start_nav = 1;

        Dist2SetPoint_vec.x = Robot_POS.x-SetPointForPub->x;
        Dist2SetPoint_vec.y = Robot_POS.y-SetPointForPub->y;
        Dist2SetPoint_vec.z = Robot_POS.z-SetPointForPub->z;

        if(vec_mag(Dist2SetPoint_vec)<SETPOINT_VICINITY_THR)
        {
            (*nav_counter)++;
            if(*nav_counter > HOLD_POS_TIME/(loop_time/1000.0))
            {
                if(*start_nav == 1)
                    *start_nav = 0;
            }
        }
        else
            *nav_counter = 0;
        return 1;
    }
    else
        return 0;
}

PointT CV2PCL(cv::Point3f cvPoint)
{
    PointT pclPoint;

    pclPoint.x = cvPoint.x;
    pclPoint.y = cvPoint.y;
    pclPoint.z = cvPoint.z;

    return pclPoint;
}

cv::Point3f PCL2CV(PointT pclPoint)
{
    cv::Point3f cvPoint;

    cvPoint.x = pclPoint.x;
    cvPoint.y = pclPoint.y;
    cvPoint.z = pclPoint.z;

    return cvPoint;

}

void cloud_denser(PointCloudT::Ptr cloud_in,PointCloudT::Ptr cloud_out,float R,int thr,int max_thr)
{
    PointCloudT::Ptr _cloud (new PointCloudT);
    PointCloudT::Ptr Add_cloud (new PointCloudT);
    _cloud->points.resize(cloud_in->points.size());
    _cloud->points = cloud_in->points;
    PointT _vec,_point;
    float dist = 0,R_2 = R*R;
    int counter = 0;
    int i,j;
    int _size = cloud_in->points.size();
    int thr_2 = max_thr;
    for(i=0;i<_size;i++)
    {
        counter = 0;
        Add_cloud->points.clear();
        for(j=i+1;j<_size;j++)
        {
            _vec.x = cloud_in->points[j].x - cloud_in->points[i].x;
            _vec.y = cloud_in->points[j].y - cloud_in->points[i].y;
            _vec.z = cloud_in->points[j].z - cloud_in->points[i].z;

            dist = _vec.x*_vec.x + _vec.y*_vec.y + _vec.z*_vec.z;
            //            dist = vec_mag(_vec);

            if(dist < R_2)
            {

                _point.x = 0.5*(cloud_in->points[j].x + cloud_in->points[i].x);
                _point.y = 0.5*(cloud_in->points[j].y + cloud_in->points[i].y);
                _point.z = 0.5*(cloud_in->points[j].z + cloud_in->points[i].z);
                Add_cloud->points.push_back(_point);

                counter++;
                j = (counter > thr_2)?(_size):(j);

            }


        }
        if(Add_cloud->points.size() >= thr && (Add_cloud->points.size() <= thr_2) )
        {
            for(int q=0;q<Add_cloud->points.size();q++)
                _cloud->points.push_back(Add_cloud->points[q]);
        }
    }
    cloud_out->points.clear();
    cloud_out->points.resize(_cloud->points.size());
    cloud_out->points = _cloud->points;
}


//std::vector<unsigned char> renderToVec(vtkSmartPointer<vtkRenderWindow> &renderWindow)
void renderToVec(vtkSmartPointer<vtkRenderWindow> &renderWindow)
{
    renderWindow->Render();
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
            vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->Update();
    //  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    //  writer->SetWriteToMemory(1);
    //  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    //  writer->Write();
    //  vtkUnsignedCharArray* rawPngBuffer = writer->GetResult();
    //  unsigned char* rawPointer = rawPngBuffer->GetPointer(0);
    //  int total_size =
    //      rawPngBuffer->GetDataSize() * rawPngBuffer->GetDataTypeSize();
    //  std::vector<unsigned char> buffer(rawPointer, rawPointer + total_size);
    //  return buffer;
}


void remove_outlier_filter(PointCloudT::Ptr cloud_in,PointCloudT::Ptr cloud_out,float R,int thr)
{
    PointCloudT::Ptr _cloud (new PointCloudT);
    _cloud->points.resize(cloud_in->points.size());
    _cloud->points = cloud_in->points;
    std::vector<uint> remove_points;
    PointT _vec;
    float dist = 0,R_2 = R*R;
    int counter = 0;
    int i,j;
    int _size = cloud_in->points.size();
    remove_points.resize(_size);

    for(i=0;i<_size;i++)
    {
        counter = 0;
        for(j=i+1;j<_size;j++)
        {
            _vec.x = _cloud->points[j].x - _cloud->points[i].x;
            _vec.y = _cloud->points[j].y - _cloud->points[i].y;
            _vec.z = _cloud->points[j].z - _cloud->points[i].z;

            dist = _vec.x*_vec.x + _vec.y*_vec.y + _vec.z*_vec.z;
            //            dist = vec_mag(_vec);

            if(dist < R_2)
            {

                counter++;
                j = (counter >= thr)?(_size):(j);

            }


        }
        if(counter < thr)
            remove_points[i] = 100;
    }

    cloud_out->points.clear();
    for(i=0;i<_size;i++)
    {
        if(remove_points[i] != 100)
            cloud_out->points.push_back(_cloud->points[i]);
    }
}

void PA_axes_correction(std::vector<PointT> &cloud_PA,float inhertness)
{
    std::vector<cv::Point3f> axes;
    for(int i=0;i<cloud_PA.size();i++)
    {
        float accept = cloud_PA[i].x + cloud_PA[i].y + cloud_PA[i].z;
        if(accept !=0)
        {
            cv::Point3f tmp_point = cv::Point3f(cloud_PA[i].x,cloud_PA[i].y,cloud_PA[i].z);
            axes.push_back(tmp_point);
        }
    }

    std::vector<std::pair<float,int> > axes_dot_sum;
    std::pair<float,std::pair<int,int> > max_dot_pair;
    max_dot_pair.first = 0;
    axes_dot_sum.resize(axes.size());
    for(int i=0;i<axes.size();i++)
    {
        float sum = 0;
        for(int j=0;j<axes.size();j++)
        {
            if(i != j)
            {
                float qq = axes[i].dot(axes[j]);
                qq = qq/(vec_mag((axes[i]))*vec_mag((axes[j])));
                sum += fabs(qq);
                if(fabs(qq)>max_dot_pair.first)
                {
                    max_dot_pair.second.first = i;
                    max_dot_pair.second.second = j;
                    max_dot_pair.first = fabs(qq);
                }
            }
        }
        axes_dot_sum[i].first = sum;
        axes_dot_sum[i].second = i;
    }
    std::sort(axes_dot_sum.begin(),axes_dot_sum.end(),float_compare);

    std::vector<cv::Point3f> PA_axes;
    int axes_idx = axes_dot_sum[axes_dot_sum.size()-1].second;
    PA_axes.push_back(axes[axes_idx]);


    for(int i = axes_dot_sum.size()-2;i>=0;i--)
    {
        cv::Point3f tmp_point = axes[axes_dot_sum[i].second];
        if(fabs(tmp_point.dot(PA_axes[0])) <0.4 )
        {
            PA_axes.push_back(tmp_point);
            break;
        }
    }
    PA_axes.push_back(PA_axes[0].cross(PA_axes[1]));

    for(int i=0;i<cloud_PA.size();i++)
    {
        max_dot_pair.first = 0;
        for(int j=0;j<PA_axes.size();j++)
        {
            cv::Point3f tmp_point(cloud_PA[i].x,cloud_PA[i].y,cloud_PA[i].z);
            float qq = tmp_point.dot(PA_axes[j]);
            qq = qq/(vec_mag(tmp_point)*vec_mag(PA_axes[j]));
            if(fabs(qq)>fabs(max_dot_pair.first))
            {
                max_dot_pair.second.first = i;
                max_dot_pair.second.second = j;
                max_dot_pair.first = qq;
            }
        }

        float sign = 1;
        if(max_dot_pair.first < 0)
            sign = -1;


        cloud_PA[i].x = (1.0-inhertness)*cloud_PA[i].x + inhertness*PA_axes[max_dot_pair.second.second].x*sign;
        cloud_PA[i].y = (1.0-inhertness)*cloud_PA[i].y + inhertness*PA_axes[max_dot_pair.second.second].y*sign;
        cloud_PA[i].z = (1.0-inhertness)*cloud_PA[i].z + inhertness*PA_axes[max_dot_pair.second.second].z*sign;
    }
    //    for(int i=0;i<PA_axes.size();i++)
    //    {
    //        pcl::ModelCoefficients cylinder_coeff;
    //        cylinder_coeff.values.resize(10);
    //        QByteArray cyl_str = QString("PCA_cyl"+QString::number(i)).toLatin1();

    //        cylinder_coeff.values[3] = (float)(PA_axes[i].x);
    //        cylinder_coeff.values[4] = (float)(PA_axes[i].y);
    //        cylinder_coeff.values[5] = (float)(PA_axes[i].z);

    //        cylinder_coeff.values[0] = cloud_mean_pos.x;
    //        cylinder_coeff.values[1] = cloud_mean_pos.y;
    //        cylinder_coeff.values[2] = cloud_mean_pos.z+0.3;
    //        cylinder_coeff.values[6] = 0.02;
    //        cylinder_coeff.values[7] = (i*80)/255.;
    //        cylinder_coeff.values[8] = (i*50)/255.;
    //        cylinder_coeff.values[9] = (i*30)/255.;
    //        shapes_name_2.push_back(cyl_str);

    //        viewer_thr->DrawCylinder(cylinder_coeff,cyl_str);


    //    }
}

void generate_corridor_pointcloud(PointCloud<PointXYZ>::Ptr cloud,PointXYZ dimention,PointXYZ start_point,PointXYZ direction,uint cloud_size,float noise_cov)
{
    float x_min,x_max;
    float y_min,y_max;
    float z_min,z_max;
    if(direction.x !=0)
    {
        x_min = (direction.x == -1)?(-dimention.x+start_point.x):(start_point.x);
        x_max = (direction.x == -1)?(start_point.x+start_point.x):(dimention.x);
        y_min = -dimention.y/2+start_point.y;
        y_max = dimention.y/2+start_point.y;
        z_min = -dimention.z/2+start_point.z;
        z_max = dimention.z/2+start_point.z;
        generate_sample_pointcloud(cloud,PointT(x_min,y_min,z_min),PointT(x_max,y_max,z_min),cloud_size,noise_cov);
        generate_sample_pointcloud(cloud,PointT(x_min,y_max,z_min),PointT(x_max,y_max,z_max),cloud_size,noise_cov);
        generate_sample_pointcloud(cloud,PointT(x_min,y_min,z_max),PointT(x_max,y_max,z_max),cloud_size,noise_cov);
    }else if(direction.y !=0)
    {
        y_min = (direction.y == -1)?(-dimention.y+start_point.y):(start_point.y);
        y_max = (direction.y == -1)?(start_point.y+start_point.y):(dimention.y);
        x_min = -dimention.x/2 + start_point.x;
        x_max = dimention.x/2 + start_point.x;
        z_min = -dimention.z/2 + start_point.z;
        z_max = dimention.z/2 + start_point.z;
        generate_sample_pointcloud(cloud,PointT(x_min,y_min,z_min),PointT(x_max,y_max,z_min),cloud_size,noise_cov);
        generate_sample_pointcloud(cloud,PointT(x_min,y_min,z_min),PointT(x_min,y_max,z_max),cloud_size,noise_cov);
        generate_sample_pointcloud(cloud,PointT(x_max,y_min,z_min),PointT(x_max,y_max,z_max),cloud_size,noise_cov);
    }else if(direction.z !=0)
    {
        z_min = (direction.z == -1)?(-dimention.z+start_point.z):(start_point.z);
        z_max = (direction.z == -1)?(start_point.z+start_point.z):(dimention.z);
        x_min = -dimention.x/2 + start_point.x;
        x_max = dimention.x/2 + start_point.x;
        y_min = -dimention.y*0.5 + start_point.y;
        y_max = dimention.y*0.5 + start_point.y;
        generate_sample_pointcloud(cloud,PointT(x_min,y_max,z_min),PointT(x_max,y_max,z_max),cloud_size,noise_cov);
        generate_sample_pointcloud(cloud,PointT(x_min,y_min,z_min),PointT(x_min,y_max,z_max),cloud_size,noise_cov);
        generate_sample_pointcloud(cloud,PointT(x_max,y_min,z_min),PointT(x_max,y_max,z_max),cloud_size,noise_cov);
    }
}

float Vec2VecAngle(PointT vec1, PointT vec2)
{
    Eigen::Vector3f V1(vec1.x,vec1.y,vec1.z);
    Eigen::Vector3f V2(vec2.x,vec2.y,vec2.z);
    float angle = V1.dot(V2)/(vec_mag(V1)*vec_mag(V2));
    return angle;
}

PointT Point2LineReflection(PointT _P1, PointT _P2, PointT _P0)
{
    cv::Mat_<float> P0(1,3);
    cv::Mat_<float> P1(1,3);
    cv::Mat_<float> N(1,3);

    P0(0,0) = _P0.x; P0(0,1) = _P0.y; P0(0,2) = _P0.z;
    P1(0,0) = _P1.x; P1(0,1) = _P1.y; P1(0,2) = _P1.z;
    N(0,0) = _P2.x - _P1.x; N(0,1) = _P2.y - _P1.y; N(0,2) = _P2.z - _P1.z;

    float Mag = N.dot(N);

    cv::Mat_<float> P3(1,3);
    P3 = (2/Mag)*(P0-P1)*(N.t())*N + 2*P1 - P0;

    cv::Mat_<float> M(1,3);
    M = 0.5*(P3+P0);

    return PointT(M(0,0),M(0,1),M(0,2));
}

float RobotPointAngle(PointT _robot, PointT _vec)
{
    float Hp_V_dot = _vec.z*_robot.x - _robot.z*_vec.x;
    float H_V_dot = _vec.x*_robot.x + _robot.z*_vec.z;

    float attitude = (atan2(H_V_dot,Hp_V_dot)*(180/3.1415) - 90);

    if(attitude > 180)
        attitude = attitude - 360;
    else if(attitude < -180)
        attitude = attitude + 360;

    return attitude;
}

float RobotPointAngle(Point3f _robot, Point3f _vec)
{
    float Hp_V_dot = _vec.z*_robot.x - _robot.z*_vec.x;
    float H_V_dot = _vec.x*_robot.x + _robot.z*_vec.z;

    float attitude = (atan2(H_V_dot,Hp_V_dot)*(180/3.1415) - 90);

    if(attitude > 180)
        attitude = attitude - 360;
    else if(attitude < -180)
        attitude = attitude + 360;

    return attitude;
}

float Point2PointDist(PointT p1, PointT p2)
{
    PointT tmp_vec;
    tmp_vec.x = p1.x - p2.x;
    tmp_vec.y = p1.y - p2.y;
    tmp_vec.z = p1.z - p2.z;

    return vec_mag(tmp_vec);
}

float Point2PointDist_2(PointT p1, PointT p2)
{
    PointT tmp_vec;
    tmp_vec.x = p1.x - p2.x;
    tmp_vec.y = p1.y - p2.y;
    tmp_vec.z = p1.z - p2.z;

    return vec_mag_2(tmp_vec);
}






}
//    if(disp)
//    {
//        pcl::ModelCoefficients tmp_coeff;
//        tmp_coeff.values.resize(11);

//        PointT tmp_point;
//        for(int k=0;k<plane_property.size();k++)
//        {
//            QByteArray Plane_str = QString("CPlane"+QString::number(k)).toLatin1();
//            {
//                tmp_coeff.values[0] = plane_property[k].first.values[0];
//                tmp_coeff.values[1] = plane_property[k].first.values[1];
//                tmp_coeff.values[2] = plane_property[k].first.values[2];

//                tmp_point = mean_cloud(plane_property[k].second);
//                tmp_coeff.values[3] = tmp_point.x;
//                tmp_coeff.values[4] = tmp_point.y;
//                tmp_coeff.values[5] = tmp_point.z;

//                tmp_coeff.values[6] = 0.1;
//                tmp_coeff.values[7] = 0.1;

//                tmp_coeff.values[8] = (210+k*10)/255.0;
//                tmp_coeff.values[9] = (200+k*10)/255.0;
//                tmp_coeff.values[10] = (190+k*10)/255.0;
//            }
//            add_cloud2viewer(viewer,plane_property[k].second,Plane_str.data(),k*100+50,k*100,k*80,5);
//            viewer.removeShape(Plane_str.data());
//            draw_plane(viewer,tmp_coeff,Plane_str.data());
//        }
//    }
