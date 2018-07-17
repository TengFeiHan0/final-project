#ifndef SEG_FCN_H
#define SEG_FCN_H

#include <QObject>
#include <QDebug>


#include <iostream>

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
#include <QFile>
#include <QDataStream>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>
#include "pid_controller.h"

#include <vtkActor.h>
#include <vtkPNGWriter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

#define RAND_NUM (2*(0.5-(float)rand()/RAND_MAX))

using namespace pcl;
using namespace std;
using namespace cv;


// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace SEG {

#define PLANE_FIT_METHOD                    1
#define NORMAL_CAL_METHOD                   2
#define DIM_DIVIDE_METHOD                   3
#define NUM_DIVIDE_METHOD                   4
#define GLOBAL_MEAN_CORRECT_METHOD          5
#define LOCAL_MEAN_CORRECT_METHOD           6

//struct my_str
//{
//    PointT ref;
//    PointT query;
//};

//bool dist_compare(const my_str &i,const my_str &j)
//{
//    PointT p0 = i.ref;
//    PointT p1 = i.query;
//    PointT p2 = j.query;
//    Eigen::Vector3f vec1((p0.x-p1.x),(p0.y-p1.y),(p0.z-p1.z));
//    Eigen::Vector3f vec2((p0.x-p2.x),(p0.y-p2.y),(p0.z-p2.z));
//    float d1 = vec1.dot(vec1);
//    float d2 = vec2.dot(vec2);

//    return d1<d2;
//}

struct seg_str
{
    PointT SetPoint;
    float admissible_Len;
    std::vector<std::pair<pcl::ModelCoefficients,PointCloudT::Ptr> > plane_property;
};


// Functions for sorting..........
bool dot_product_compare(std::pair<std::pair<int, int>, float> i, std::pair<std::pair<int, int>, float> j);
bool float_compare(std::pair<float,int> i, std::pair<float,int> j);
bool float_compare_min(std::pair<float,int> i, std::pair<float,int> j);
bool double_compare_min(std::pair<double,int> i, std::pair<double,int> j);
bool float_raw_compare(float i,float j);

//Other Functions
void viewer_init(visualization::PCLVisualizer &viewer);
void add_cloud2viewer(visualization::PCLVisualizer &viewer,PointCloud<PointXYZ>::ConstPtr cloud,QByteArray pc_name,uchar r,uchar g,uchar b,uchar points_size);
void LoadFromeFile(PointCloud<pcl::PointXYZ>::Ptr _cloud,QByteArray fileName);
void Save2File(PointCloud<PointXYZ>::Ptr cloud);
float define_area(visualization::PCLVisualizer &viewer, int disp, bool print, PointCloud<PointXYZ>::Ptr _cloud, PointT _dim, uint divide_method, uint method, uint correction_method, float modify);
void remove_outlier(PointCloud<PointXYZ>::Ptr _cloud,PointCloud<pcl::PointXYZ>::Ptr _cloud_filtered,int mean_k,float thr);
void plane_fit(PointCloudT::Ptr cloud,float fit_dist,uint ITE,std::vector<std::pair<pcl::ModelCoefficients,PointCloudT::Ptr> > *plane_property);
int cloud_segment(visualization::PCLVisualizer &viewer, bool disp, PointCloudT::Ptr cloud_filtered, float fit_dist);
void search_Area(PointCloud<PointXYZ>::Ptr _cloud, PointCloud<pcl::PointXYZ>::Ptr selected_cloud, PointXYZ curr_POS, float dist);
void draw_plane(visualization::PCLVisualizer &viewer, ModelCoefficients coeff, char *ID);
PointT mean_cloud(PointCloudT::Ptr cloud);
void clustring(std::vector<ModelCoefficients> &_input, uint Cluster_num, uint col_num);
void Compute_Normal(visualization::PCLVisualizer &viewer, PointCloudT::Ptr cloud, uint K, char *ID);
PointT find2planeDist(pcl::ModelCoefficients p1,PointT p2);
float pointPlaneDist(pcl::ModelCoefficients p1,PointT p2);
float pointPlaneDist_2(pcl::ModelCoefficients p1,PointT p2);
float ParallelPlaneDist(pcl::ModelCoefficients p1,pcl::ModelCoefficients p2);
void correct_cloud_scale(PointCloudT::Ptr _cloud,float scale);
float vec_mag(PointT vec);
float vec_mag(Eigen::Vector3f V);
float vec_mag(cv::Point3f vec);
float vec_mag_2(cv::Point3f vec);
float vec_mag_2(PointT vec);
float Vec2VecAngle(PointT vec1,PointT vec2);
void cloud_classifyer(std::vector<std::set<int> > &outlier, std::vector<std::vector<std::pair<ModelCoefficients, int> > > &clustered_coeff_vec
                      , std::vector<pcl::ModelCoefficients> &corrected_clustered_coeff_vec, std::vector<PointT> &cloud_PA, std::vector<PointT> &cluster_centers, bool remove_outlier_flag);
seg_str final_segmentation(visualization::PCLVisualizer &viewer, bool disp, PointCloudT::Ptr _cloud, float fit_dist, float gap);
PointT findPointReflection(pcl::ModelCoefficients _plane,PointT _point);
PointT findPointReflection_2(pcl::ModelCoefficients _plane,PointT _point);
void generate_sample_pointcloud(PointCloud<PointXYZ>::Ptr cloud, PointXYZ start_point, PointXYZ end_point, uint cloud_size, float noise_cov);
void generate_corridor_pointcloud(PointCloud<PointXYZ>::Ptr cloud,PointXYZ dimention,PointXYZ start_point,PointXYZ direction,uint cloud_size,float noise_cov);
bool navigation_fcn(PointT *_SetPointForPub,PointT Robot_POS,PointT SetPoint_filtered,seg_str Segmentation_OutPut,float SetPointDeviation,int *start_nav
                    ,float SETPOINT_VICINITY_THR,float HOLD_POS_TIME,float loop_time,uint *nav_counter);

void cloud_denser(PointCloudT::Ptr cloud_in,PointCloudT::Ptr cloud_out,float R,int thr,int max_thr);

PointT CV2PCL(cv::Point3f cvPoint);
cv::Point3f PCL2CV(PointT pclPoint);
void renderToVec(vtkSmartPointer<vtkRenderWindow> &renderWindow);
void remove_outlier_filter(PointCloudT::Ptr cloud_in,PointCloudT::Ptr cloud_out,float R,int thr);
void PA_axes_correction(std::vector<PointT> &cloud_PA, float inhertness);
PointT Point2LineReflection(PointT _P1, PointT _P2, PointT _P0);

float RobotPointAngle(PointT _robot,PointT _vec);
float RobotPointAngle(cv::Point3f _robot,cv::Point3f _vec);
float Point2PointDist(PointT p1,PointT p2);
float Point2PointDist_2(PointT p1,PointT p2);


}
#endif // SEG_FCN_H
