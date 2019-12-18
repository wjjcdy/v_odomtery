#ifndef POSE_ESTIOMATION_3D3D_H
#define POSE_ESTIOMATION_3D3D_H
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>

#include <pcl/io/pcd_io.h>                      
#include <pcl/point_types.h>                    
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>              
#include <pcl_conversions/pcl_conversions.h>   
#include <pcl/filters/voxel_grid.h> 

using namespace std;
using namespace cv;
int pose_estimation_get(const Mat &img_1,const Mat &img_2, const Mat &depth1,const Mat &depth2,   Eigen::Matrix3d &R,  
  Eigen::Vector3d &t);

int pose_estimation_cloud_get(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                              const pcl::PointCloud<pcl::PointXYZRGB> &cloud_last,   
                              Eigen::Matrix3d &R,  
                              Eigen::Vector3d &t);
#endif