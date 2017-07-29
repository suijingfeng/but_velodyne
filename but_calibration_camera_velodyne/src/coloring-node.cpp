/*
 * coloring the pointcloud with the image 
 */

#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/tf.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>


// offline process the recorded data 
#define USE_RECORDED_FILES 1
#define USE_ONLY_XYZ	   1


using namespace std;
using namespace cv;
using namespace pcl;


string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;
string VELODYNE_COLOR_TOPIC;

ros::Publisher pub;
cv::Mat projection_matrix;
cv::Mat frame_rgb;

pcl::PointCloud<pcl::PointXYZ> pointxyz_pc;
pcl::PointCloud<Velodyne::Point> origin_pc;

#ifdef USE_ONLY_XYZ
pcl::PointCloud<pcl::PointXYZ> visible_points;
#else
pcl::PointCloud<Velodyne::Point> visible_points;
#endif
//float DoF[6] = {0.0503676, -0.14093, -0.0114802, -0.00142857, -0.00428571, 0.01};
std::vector<float> DoF;

/*  
//  Mat empty = Mat::zeros(frame.size(), CV_8UC1);

  Mat result_channel(frame.size(), CV_8UC3);
  Mat in[] = {image, plane};
  int from_to[] = {0, 0, 1, 1, 2, 2};
  mixChannels(in, 2, &result_channel, 1, from_to, 3);
  cv::imwrite("mixChannels.png", result_channel);
  return result_channel;
 */


static cv::Point2f projectfo(const PointXYZ &pt, const cv::Mat &projection_matrix)
{
	cv::Mat pt_3D(4, 1, CV_32FC1);

	pt_3D.at<float>(0) = pt.x;
	pt_3D.at<float>(1) = pt.y;
	pt_3D.at<float>(2) = pt.z;
	pt_3D.at<float>(3) = 1.0f;//is homogenious coords. the point's 4. coord is 1

	cv::Mat pt_2D = projection_matrix * pt_3D;

	float w = pt_2D.at<float>(2);
	float x = pt_2D.at<float>(0) / w;
	float y = pt_2D.at<float>(1) / w;

	return cv::Point2f(x, y);
}


PointCloud<PointXYZRGB> colouring(pcl::PointCloud<PointXYZ> &pc, 
		                          cv::Mat frame_rgb, cv::Mat P)
{
  PointCloud<PointXYZRGB> color_cloud;
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc.begin(); pt < pc.end(); pt++)
  {
    Point2f xy = projectfo(*pt, P);

    Vec3b rgb = Image::Image::atf(frame_rgb, xy);
    PointXYZRGB pt_rgb(rgb.val[2], rgb.val[1], rgb.val[0]);
    pt_rgb.x = pt->x;
    pt_rgb.y = pt->y;
    pt_rgb.z = pt->z;

    color_cloud.push_back(pt_rgb);
  }
  return color_cloud;
}



template<typename PointT>
void project( pcl::PointCloud<PointT> &pc, 
		       cv::Mat img, cv::Mat& proj_mat)
{

  cv::Rect frame(0, 0, img.cols, img.rows);

  for (typename pcl::PointCloud<PointT>::iterator pt = pc.points.begin();
                                   pt < pc.points.end(); pt++)
  {
    // behind the camera
    if (pt->z < 0)
    {
      continue; 
    }
#ifndef USE_ONLY_XYZ
    float intensity = pt->intensity;
#endif
	cv::Mat pt_3D(4, 1, CV_32FC1);
 
 	pt_3D.at<float>(0) = pt->x;
    pt_3D.at<float>(1) = pt->y;
    pt_3D.at<float>(2) = pt->z;
    pt_3D.at<float>(3) = 1.0f;//is homogenious coords. the point's 4. coord is 1

    cv::Mat pt_2D = proj_mat * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;

    cv::Point xy(x, y);

	if (xy.inside(frame))
    {
        visible_points.push_back(*pt);
		cv::circle(img, xy, 2, Scalar(0, 255, 0));
//		frame_rgb.at<float>(y, x) = intensity;
    }

  }
  cv::imwrite("mixChannels.png", img);
}



#ifdef USE_RECORDED_FILES
int LoadPointXYZData(std::string name)
{

  frame_rgb = cv::imread("frame_rgb.png");
  std::cout << "Loading camera image... " << std::endl;
  
  std::cout << "Loading projection matrix..." << std::endl;
  float p[12] = { 6.71134705e+02,             0., 6.61914856e+02, 0.,
	            	          0., 6.71134705e+02, 3.57206055e+02, 0.,
							  0.,             0.,             1., 0.};
  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
  std::cout << projection_matrix << std::endl;

//  PointCloud<Velodyne::Point> pc;

  std::cout << "Loading Velodyne point cloud..." << std::endl;

  //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (name, pointxyz_pc) == -1)
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << pointxyz_pc.width * pointxyz_pc.height
            << " data points from velodyne_pc.pcd with the following fields: "
            << std::endl;

//  pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI, -M_PI / 2, 0);
//	origin_pc = Velodyne::Velodyne(pc);

}


int LoadRecordedData(std::string name)
{

  frame_rgb = cv::imread("frame_rgb.png");
  std::cout << "Loading camera image... " << std::endl;
  
  std::cout << "Loading projection matrix..." << std::endl;
  float p[12] = { 6.71134705e+02,             0., 6.61914856e+02, 0.,
	            	          0., 6.71134705e+02, 3.57206055e+02, 0.,
							  0.,             0.,             1., 0.};
  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
  std::cout << projection_matrix << std::endl;

//  PointCloud<Velodyne::Point> pc;

  std::cout << "Loading Velodyne point cloud..." << std::endl;

  //* load the file
  if (pcl::io::loadPCDFile<Velodyne::Point> (name, origin_pc) == -1)
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << origin_pc.width * origin_pc.height
            << " data points from velodyne_pc.pcd with the following fields: "
            << std::endl;

//  pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI, -M_PI / 2, 0);
//	origin_pc = Velodyne::Velodyne(pc);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");

  ros::NodeHandle nh_;
  pub = nh_.advertise<sensor_msgs::PointCloud2>(VELODYNE_COLOR_TOPIC, 1);
  
  nh_.getParam("/but_calibration_camera_velodyne/6DoF", DoF);

  std::cout << "DoF: " << DoF[0] <<","<< DoF[1] <<","<< DoF[2] << "," 
					   << DoF[3] << ","<< DoF[4] << "," << DoF[5] 
					   << std::endl;
  
  Eigen::Affine3f transf2 = getTransformation(DoF[0], DoF[1], DoF[2],
		                                      DoF[3], DoF[4], DoF[5]);


  while(nh_.ok())
  {
	  	  
#ifdef USE_ONLY_XYZ
	  LoadPointXYZData("pointsProjectedtoPlane.pcd");
	  pcl::PointCloud<pcl::PointXYZ> calibrated_cloud;
	  transformPointCloud(pointxyz_pc, calibrated_cloud, transf2);
	  project( calibrated_cloud, frame_rgb, projection_matrix );

	  PointCloud<PointXYZRGB> color_cloud 
            = colouring(visible_points, frame_rgb, projection_matrix);


#else
	  LoadRecordedData("edges_scan.pcd");

	  pcl::PointCloud<Velodyne::Point> calibrated_cloud;
	  transformPointCloud(origin_pc, calibrated_cloud, transf2);
	  project( calibrated_cloud, frame_rgb, projection_matrix );
	  
	  Velodyne::Velodyne visible_scan(visible_points);
	  PointCloud<PointXYZRGB> color_cloud 
                           = visible_scan.colour(frame_rgb, projection_matrix);

#endif	  


	  sensor_msgs::PointCloud2 color_cloud2;
	  toROSMsg(color_cloud, color_cloud2);
	  color_cloud2.header.frame_id = "velodyne";
	  pub.publish(color_cloud2);
  }
  return 0;
}















#else

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

  // if no rgb frame for coloring:
  if (frame_rgb.data == NULL)
  {
    return;
  }

  pcl::PointCloud<Velodyne::Point> pc;
  pcl::PointCloud<Velodyne::Point> tmp;
  pcl::PointCloud<Velodyne::Point> new_cloud;


  fromROSMsg(*msg, pc);
  Eigen::Affine3f transf1 = getTransformation(0, 0, 0, M_PI / 2, -M_PI/2, 0);
  Eigen::Affine3f transf2 = getTransformation(DoF[0], DoF[1], DoF[2],
		                                      DoF[3], DoF[4], DoF[5]);
  transformPointCloud(pc, tmp, transf1);
  transformPointCloud(tmp, new_cloud, transf2);

  Image::Image img(frame_rgb);

  project( new_cloud, frame_rgb, projection_matrix );

  Velodyne::Velodyne visible_scan(visible_points);

  PointCloud<PointXYZRGB> color_cloud 
                           = visible_scan.colour(frame_rgb, projection_matrix);

  // reverse axix switching:
  Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI / 2, 0, 0);
  transformPointCloud(color_cloud, color_cloud, transf);

  sensor_msgs::PointCloud2 color_cloud2;
  toROSMsg(color_cloud, color_cloud2);
  color_cloud2.header = msg->header;


  pub.publish(color_cloud2);

//  std::cout <<" color cloud published!" << std::endl;
}


void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg->P.begin();
                                                  i != msg->P.end(); i++)
  {
    *pp = (float)(*i);
    pp++;

  }

  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr =
                 cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  frame_rgb = cv_ptr->image;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coloring_node");

  ros::NodeHandle n;
  n.getParam("/but_calibration_camera_velodyne/camera_frame_topic",
             CAMERA_FRAME_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/camera_info_topic",
             CAMERA_INFO_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_topic",
             VELODYNE_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_color_topic",
             VELODYNE_COLOR_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/6DoF", DoF);

  pub = n.advertise<sensor_msgs::PointCloud2>(VELODYNE_COLOR_TOPIC, 1);

  // Subscribe input camera image
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = 
                it.subscribe(CAMERA_FRAME_TOPIC, 10, imageCallback);

  ros::Subscriber info_sub = 
              n.subscribe(CAMERA_INFO_TOPIC, 10, cameraInfoCallback);

  ros::Subscriber pc_sub =
   n.subscribe<sensor_msgs::PointCloud2>(VELODYNE_TOPIC, 1, pointCloudCallback);

  ros::spin();

  return EXIT_SUCCESS;
}

#endif
