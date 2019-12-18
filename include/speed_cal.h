//
// Created by forrest on 18-3-5.
//

#ifndef PLANES_DETECTION_PLANES_DETECTION_H
#define PLANES_DETECTION_PLANES_DETECTION_H

#include <iostream>              

#include <pcl/io/pcd_io.h>                      
#include <pcl/point_types.h>                    
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>              
#include <pcl_conversions/pcl_conversions.h>   
#include <pcl/filters/voxel_grid.h> 

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <depth_image_proc/depth_conversions.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
using namespace cv;
using namespace std;
using namespace Eigen;
// #define DISPLAY  // PCL显示标志位
// 类型定义
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace planes_detection
{
    class PlanesDetection
    {
    public:

        PlanesDetection(void);
        ~PlanesDetection(void);

        bool runPlanesDetection(void);

    private:


        /**
         * @brief  coud data callback
         */
        void rgbImageCallback(const sensor_msgs::Image::ConstPtr& rgb_image_msg);

        void depthImageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg);

        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

        double normofTransform( Eigen::Matrix3d R,  Eigen::Vector3d t );

        PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, 
                                        PointCloud::Ptr newCloud, 
                                        Eigen::Isometry3d T) ; 







        ros::Subscriber sub;
        ros::Subscriber rgb_sub_;
        ros::Subscriber cloud_sub_;
        image_transport::CameraSubscriber sub_depth_image_;

        ros::Subscriber saved_flag_sub_;             // 背景depth获取命令

        ros::Publisher pub_obstale_height_;          // 发布高度
        ros::Publisher pub_obstacle_speed_;          // 发布速度

        ros::Publisher heartbeat_pub;
        
        //tf::TransformBroadcaster planes_cloud_broadcaster;
        //tf::TransformBroadcaster tf_cloud_broadcaster;
        //tf::TransformBroadcaster scan_cloud_broadcaster;

        std::string sub_image_topic; 
        std::string sub_rgb_topic;
        std::string sub_cloud_topic;
                       

        cv::Mat gray_last_;
        bool init_flag_;
        bool depth_init_flag_;
        int MAX_CORNERS_;
        double qualityLevel_;
        double minDistance_;
        std::vector<cv::Point2f>  point_feature_;  // 上次特征点
        ros::Time last_time_;                      // 上次时间刻
        double time_period_;                       // 测量周期
        double offset_sum_;                         // 周期内位移和
        double g_speed_;                           // 速度，单位为 像素/s
        double back_height_;                       // 背景高度
        double curr_height_;                       // 当前高度
        double err_height_;                        // 高度差
        double height_ref_;                        // 参考高度，超过此高度的忽略不计算
        bool saved_flag_;                          // 更新并存储背景参考深度图标志位

        cv::Mat rgb_image_;
        cv::Mat depth_image_;
        cv::Mat depth_image_refer_;

        Eigen::Isometry3d pose_curr_T_;
        vector<Eigen::Isometry3d> key_pose_T_;

        Eigen::Isometry3d pose_curr_T_2rd_;
        vector<Eigen::Isometry3d> key_pose_T_2rd_;

        pcl::visualization::PCLVisualizer viewer_;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_last_;

        PointCloud::Ptr cloud_global_;
        
    #ifdef DISPLAY
        pcl::visualization::PCLVisualizer viewer;           //显示viewer对象
    #endif
    };
}

#endif //PLANES_DETECTION_PLANES_DETECTION_H
