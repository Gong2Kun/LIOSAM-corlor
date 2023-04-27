//写个接收点云和图像话题的ROS节点  用来给点云上色

//queue
#include <queue>
#include <iostream>  
#include <boost/thread/thread.hpp>  
#include <pcl/common/common_headers.h>  
#include <pcl/common/common_headers.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/console/parse.h>  
#include <pcl/point_types.h>
#include <ros/ros.h>
#include "utility.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include <ros/subscriber.h>
#include<string>
using namespace cv;
using namespace std;
ros::Publisher pub;
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
// viewer->setBackgroundColor(0, 0, 0);
//设置100的队列大小
queue<sensor_msgs::ImageConstPtr> img_vec;
// boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
// {
//     	viewer->removeAllPointClouds();
// 	viewer->removeAllShapes();


//     //创建3D窗口并添加点云   
//     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//     viewer->addCoordinateSystem(1.0);
//     viewer->initCameraParameters();
//     return (viewer);
// }



//image imcallback
void imageCallback( const sensor_msgs::CompressedImageConstPtr &img_msg){
     cv_bridge::CvImageConstPtr ptr;
  cv::Mat image = cv::imdecode(cv::Mat(img_msg->data), 1); 
    // cout<<image.size()<<endl;
    // cv::imshow("image_path",image);
    // cv::waitKey(1);
  sensor_msgs::ImagePtr img_msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  img_msg1->header.stamp=img_msg->header.stamp;
  // if(msg->header.stamp.toSec()!=1670371948.796)
  //imgLeftBuf.push(img_msg1);
    img_vec.push(img_msg1);
    //ROS_INFO("empty()");
    // string str=to_string(img_vec.size());
    // ROS_INFO(str);
    //cout<<img_vec.size()<<endl;
    while(img_vec.size()>1000){
        img_vec.pop();
    }
    
}

//输入点云和图像并将点云重投影到图像上
void pointcloud_color(const sensor_msgs::PointCloud2ConstPtr&  cloud_pos)
{
    //ROS_INFO("error");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_pos, *point_ptr);
    pcl::copyPointCloud(*point_ptr, *point_cloud_ptr);
    if(img_vec.empty()){
        //ROS_INFO("no match point and image!");
        return;
    }
    while(img_vec.front()->header.stamp<cloud_pos->header.stamp){
        img_vec.pop();
        if(img_vec.empty()){
            return;
        }
    }
    cout<<"time:"<<img_vec.front()->header.stamp<<"  "<<cloud_pos->header.stamp<<endl;
    const sensor_msgs::ImageConstPtr msg= img_vec.front();
    img_vec.pop();
    cv_bridge::CvImagePtr cv_ptr;
    //将ROS图像消息转化为opencv图像
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    int rows = img.rows;
	int cols = img.cols;
	unsigned char red,green,blue;
	float p_u,p_v,p_w;//pics_uv1;(u for cols, v for lines!!!)
	float c_x,c_y,c_z,c_i;//clouds_xyz、intensity;
	
	Mat P2 = (Mat_<float>(3,4) <<935.307436,0.000,960.0,0.000,0.000,935.307436,540.0,0.0000,0.000,0.000000,1.000,0.0000);
	//Mat R0_rect = (Mat_<float>(4,4) << 9.999239000000e-01,9.837760000000e-03,-7.445048000000e-03,0,-9.869795000000e-03,9.999421000000e-01,-4.278459000000e-03,0,7.402527000000e-03,4.351614000000e-03,9.999631000000e-01,0, 0,0,0,1);
	Mat Tr_lidar_imu = (Mat_<float>(4,4) << 1, 0, 0,1.5,
                  0, 1, 0,0.0,
                  0, 0, 1,-0.3,
                  0,0,0,1);
    Mat Tr_cam_imu = (Mat_<float>(4,4) << 0.0, 0.0, 1.0, -0.0,
         -1.0, 0.0, 0.0, -0.0,
         -0.0, -1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0);
	Mat Tr_lidar_cam = Tr_lidar_imu*(Tr_cam_imu.inv());
    cout<<P2<<endl;
	Mat trans = Mat(3,4,CV_32FC1);
	trans = P2 * Tr_lidar_cam;
	Mat c_tmp = Mat(4,1,CV_32FC1);
	Mat p_result = Mat(1,3,CV_32FC1);
for(int nIndex = 0; nIndex < point_cloud_ptr->points.size (); nIndex++)
	{	
		c_x = point_cloud_ptr->points[nIndex].x;
		c_y = point_cloud_ptr->points[nIndex].y;
		c_z = point_cloud_ptr->points[nIndex].z;
		
		c_tmp = (Mat_<float>(4, 1) << c_x, c_y, c_z, 1);		
		p_result = trans * c_tmp;
	//	cout<<"c_tmp:"<<c_tmp<<endl;
		p_w = p_result.at<float>(0,2);
      //  cout<<"p_result:"<<p_result<<endl;
		p_u = (int)((p_result.at<float>(0,0)) / p_w);
		p_v = (int)((p_result.at<float>(0,1)) / p_w);
		if( (p_u<0) || (p_u>cols)  || (p_v<0) || (p_v>rows) ||(p_w < 0)){
			point_cloud_ptr->points[nIndex].r = 128;
			point_cloud_ptr->points[nIndex].g = 2;
			point_cloud_ptr->points[nIndex].b = 64;
			continue;
		}
		point_cloud_ptr->points[nIndex].r = img.at<Vec3b>(p_v,p_u)[2];//not (p_u,p_v)!
		point_cloud_ptr->points[nIndex].g = img.at<Vec3b>(p_v,p_u)[1];
		point_cloud_ptr->points[nIndex].b = img.at<Vec3b>(p_v,p_u)[0];
	}	
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    // PointXYZIRPYT thisPose6D;
    // thisPose6D.x = cloud_pos->pose[3];
    // thisPose6D.y = cloud_pos->pose[4];
    // thisPose6D.z = cloud_pos->pose[5];
    // thisPose6D.roll  = cloud_pos->pose[0];
    // thisPose6D.pitch = cloud_pos->pose[1];
    // thisPose6D.yaw   = cloud_pos->pose[2];
    
   // PointXYZIRPYT thisPose6D = trans2PointTypePose(*clould_pos->pose);
   // cloud_out = transformPointCloud(point_cloud_ptr,    &thisPose6D);
    // rgbVis(point_ptr); 
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*point_cloud_ptr, output);
    output.header.frame_id = "odom";
    output.header.stamp = cloud_pos->header.stamp;
    pub.publish(output);
}
//subCloud = nh.subscribe<liorf::cloud_info>("liorf/deskew/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "liorf");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> Pointcloud_corlor Started.\033[0m");

    pub = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/color_pointcloud", 10);
    ros::Subscriber sub = nh.subscribe("/lidar", 10, pointcloud_color);
    ros::Subscriber   sub2 = nh.subscribe("/camera/main", 1000, imageCallback);

    // message_filters::Subscriber<liorf::cloud_pos> cloud_sub(nh, "liorf/mapping/cloud_registered", 1,ros::TransportHints().tcpNoDelay());
    // message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/main", 10000);
    // typedef message_filters::sync_policies::ApproximateTime<liorf::cloud_pos, sensor_msgs::Image> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    // sync.registerCallback(boost::bind(&pointcloud_color, _1, _2));

	// while (!viewer->wasStopped())
	// {
	// 	viewer->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }
    ros::spin();
    return 0;
}
