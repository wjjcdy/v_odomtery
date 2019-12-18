//
// Created by jiajia on 19-3-5.
//

#include "speed_cal.h"
#include "pose_estimation_3d3d.h"

#define SIM

namespace planes_detection
{
    PlanesDetection::PlanesDetection(void)
    {
        ros::NodeHandle param_n("~");
        param_n.param<std::string>("sub_image_topic", sub_image_topic, "/camera_02/depth/image");
        param_n.param<std::string>("sub_rgb_topic", sub_rgb_topic, "/camera_02/rgb/image_raw");
        param_n.param<std::string>("sub_cloud_topic", sub_cloud_topic, "/camera/depth_registered/points");
        
        ros::NodeHandle n;
        //sub = n.subscribe(sub_cloud_topic, 1, &PlanesDetection::pointCloudCallback,this);
        image_transport::ImageTransport it(n);
        sub_depth_image_ = it.subscribeCamera(sub_image_topic, 1, &PlanesDetection::depthImageCb,this);
        rgb_sub_ = n.subscribe(sub_rgb_topic, 1, &PlanesDetection::rgbImageCallback,this);
        cloud_sub_ = n.subscribe(sub_cloud_topic, 1, &PlanesDetection::cloudCallback,this); 

        pose_curr_T_ = Isometry3d::Identity();
        key_pose_T_.clear();
        key_pose_T_.push_back(pose_curr_T_);

        pose_curr_T_2rd_ = Isometry3d::Identity();
        key_pose_T_2rd_.clear();
        key_pose_T_2rd_.push_back(pose_curr_T_2rd_);

        viewer_.setWindowName("viewer");
        viewer_.setBackgroundColor (0, 0, 0);       //设置背景颜色

        cloud_.clear();
        cloud_last_.clear();
        cloud_global_.reset(new pcl::PointCloud<PointT>()); 
    }

    PlanesDetection::~PlanesDetection(void){}

    bool PlanesDetection::runPlanesDetection(void)
    {
        static int flag1=0;
        static int flag2=0;
        static int i_num=0;
        static Mat image_last;
        static Mat image_depth_last;
        
       
        if(!depth_image_.empty()) {
            flag1 = 1;
        }
        else{
            ROS_ERROR("no depth imagee");
        }
        
        if(!rgb_image_.empty()) {
            flag2=1;
        } 
        else {
            ROS_ERROR("no rgb imagee");
        }

        if(!cloud_.size()){
            flag1=1;
            flag2=1;
        }

        // if(flag1 && flag2) {
        //     if (i_num==0) {
        //     // rgb_image_.copyTo( image_last);
        //     // depth_image_.copyTo(image_depth_last);
        //     // cout<<depth_image_<<endl;
        //         pcl::copyPointCloud(cloud_, cloud_last_);
        //     }
        //     else 
        //     {
        //         // Mat image, depth_image;
        //         // rgb_image_.copyTo(image);
        //         // depth_image_.copyTo(depth_image);
        //         // imshow("image",image);
        //         // waitKey(1);

        //         Eigen::Matrix3d R;
        //         Eigen::Vector3d t;
        //         // 计算两帧变换矩阵 R t

        //         //pose_estimation_get(image,image_last,depth_image,image_depth_last,R,t);

        //         pose_estimation_cloud_get(cloud_, cloud_last_, R, t);

        //         // 计算运动范围是否太大
        //         double norm = normofTransform(R, t);
        //         cout<<"norm:" << norm<<endl;
        //         if(norm < 0.3) 
        //         {
        //             Eigen::Isometry3d T = Isometry3d::Identity();  
        //             T.prerotate(R);
        //             T.pretranslate(t);

        //             pose_curr_T_ =  T* pose_curr_T_;
        //             key_pose_T_.push_back(pose_curr_T_);
                    
        //             pose_curr_T_2rd_.prerotate(R);
        //             pose_curr_T_2rd_.pretranslate(t);
        //             key_pose_T_2rd_.push_back(pose_curr_T_2rd_);
        //             // rgb_image_.copyTo( image_last);
        //             // depth_image_.copyTo(image_depth_last);
        //             pcl::copyPointCloud(cloud_, cloud_last_);
        //        }
        //     }

            i_num++;
            cout << "2" << endl;
            Mat path_image(800, 800, CV_8UC3, Scalar(255,255,255));
            for(int i=0; i< key_pose_T_.size(); ++i) {
                double x = key_pose_T_[i](0,3);
                double y = key_pose_T_[i](2,3);

                Point p2;
                p2.x = 400 + x*20;
                p2.y = 400 + y*20;
                //画实心点
                circle(path_image, p2, 3,Scalar(255,0,0),-1); //第五个参数我设为-1，表明这是个实点。

            }
            cout << "3" << endl;
            namedWindow("path",WINDOW_NORMAL);
            imshow("path",path_image);
            waitKey(1);
        return true;
    }

    double PlanesDetection::normofTransform( Eigen::Matrix3d R,  Eigen::Vector3d t )
    {
        cv::Mat rvec;
        cv::Mat tvec;
        // convert to cv::Mat
        rvec = (Mat_<double>(3, 3) <<
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2)
        );
        tvec = (Mat_<double>(3, 1) << t(0, 0), t(1, 0), t(2, 0));
        return fabs(cv::norm(tvec));
    }

    void PlanesDetection::depthImageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
         try {
            //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
            cv_bridge::CvImagePtr depth_imag_cv_ptr_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); 
            if (!depth_imag_cv_ptr_->image.empty()) {
                depth_imag_cv_ptr_->image.copyTo(depth_image_);
            }
         }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void PlanesDetection::rgbImageCallback(const sensor_msgs::Image::ConstPtr& rgb_image_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
            if (!cv_ptr->image.empty()) {
              cv_ptr->image.copyTo(rgb_image_);
            }
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }
    }

    void PlanesDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud_.clear();
        pcl::fromROSMsg(*cloud_msg,cloud_);
  //... populate cloud
        viewer_.removeAllPointClouds();
                viewer_.addPointCloud(cloud_global_, "test");
        viewer_.updatePointCloud(cloud_global_, "test");
        viewer_.spinOnce(0.0000000000001);

        if (!cloud_last_.size()) {
        // rgb_image_.copyTo( image_last);
        // depth_image_.copyTo(image_depth_last);
        // cout<<depth_image_<<endl;
            pcl::copyPointCloud(cloud_, cloud_last_);
            *cloud_global_ += cloud_;
        }
        else 
        {
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            // 计算两帧变换矩阵 R t

            //pose_estimation_get(image,image_last,depth_image,image_depth_last,R,t);

            pose_estimation_cloud_get(cloud_,cloud_last_, R, t);

            // 计算运动范围是否太大
            double norm = normofTransform(R, t);
            cout<<"norm:" << norm<<endl;
            if(norm < 0.3) 
            {
                Eigen::Isometry3d T = Isometry3d::Identity();  
                T.prerotate(R);
                T.pretranslate(t);

                pose_curr_T_ =  T* pose_curr_T_;
                key_pose_T_.push_back(pose_curr_T_);
                
                pose_curr_T_2rd_.prerotate(R);
                pose_curr_T_2rd_.pretranslate(t);
                key_pose_T_2rd_.push_back(pose_curr_T_2rd_);
                // rgb_image_.copyTo( image_last);
                // depth_image_.copyTo(image_depth_last);
                pcl::copyPointCloud(cloud_, cloud_last_);
                PointCloud::Ptr new_cloud = cloud_.makeShared();
                // *new_cloud += cloud_.makeShared();
                cloud_global_ = joinPointCloud( cloud_global_, new_cloud, T);
            }
        }
    }

    // joinPointCloud 
    // 输入：原始点云，新来的帧以及它的位姿
    // 输出：将新来帧加到原始帧后的图像
    PointCloud::Ptr PlanesDetection::joinPointCloud( PointCloud::Ptr original, 
                                                    PointCloud::Ptr newCloud, 
                                                    Eigen::Isometry3d T) 
    {
        // 合并点云
        PointCloud::Ptr output (new PointCloud());
        pcl::transformPointCloud( *original, *output, T.matrix() );
        *newCloud += *output;

        // Voxel grid 滤波降采样
        static pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize( 0.01, 0.01, 0.01 );
        voxel.setInputCloud( newCloud );
        PointCloud::Ptr tmp( new PointCloud() );
        voxel.filter( *tmp );
        return tmp;
    }

}
