#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Calibration.h>
#include <but_calibration_camera_velodyne/Calibration3DMarker.h>
#include <but_calibration_camera_velodyne/Image.h>

// offline process the recorded data 
#define USE_RECORDED_FILES 1
// MAX laser and camera translation
#define T_C_L_MAX			0.5


using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;


////////////////// GLOBAL VARIABLES ///////////////////////// 

string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;

// marker properties:
float STRAIGHT_DISTANCE; // 23cm
float RADIUS; // 8.25cm
float EDGE_THRESHOLD;

cv::Mat projection_matrix;
cv::Mat frame_rgb;
Velodyne::Velodyne pointcloud;
bool doRefinement = false;


/**
* coarse calibration (only translation) from 2D - 3D correspondences.
*/

Calibration6DoF coarseCalibration( cv::Mat projection,
						std::vector<cv::Point2f> c_img, std::vector<float> r_img,
						std::vector<cv::Point3f> c_lsr, float r_truth )
{
	float Tx = 0;
	float Ty = 0;
	float Tz = 0;
	
	float f = projection.at<float>(0, 0);
	float o_x = projection.at<float>(0, 2);
	float o_y = projection.at<float>(1, 2);

	for (size_t i = 0; i < 4; i++)
	{
	  float t_z = r_truth / r_img[i] * f - c_lsr[i].z;
	  float t_x = (c_img[i].x - o_x) * (c_lsr[i].z + t_z)/f - c_lsr[i].x;
	  float t_y = (c_img[i].y - o_y) * (c_lsr[i].z + t_z)/f - c_lsr[i].y;

	  std::cout << "translation" << i << ": " <<
		    t_x << ", " << t_y << ", "<< t_z << std::endl;

	   Tx += t_x;
	   Ty += t_y;
	   Tz += t_z;
	}
	Tx /= 4;
	Ty /= 4;
	Tz /= 4;
	// 
	if( abs(Tx) > T_C_L_MAX || abs(Ty) > T_C_L_MAX || abs(Tz) > T_C_L_MAX )
	{	
		std::cout << "Calculated error again, check here" << std::endl; 
		exit(-1);
	}
	else
	{
		std::cout << "Average translation: " 
			      << Tx << ", " << Ty << ", "<< Tz << std::endl;
	}
	return Calibration6DoF(Tx, Ty, Tz, 0, 0, 0, 0);
}


void calibrationRefinement(
		  Image::Image img, pcl::PointCloud<pcl::PointXYZI> &edges, cv::Mat P, 
		  float x_rough, float y_rough, float z_rough,
		  float max_translation, float max_rotation, unsigned steps,
          Calibration6DoF &best_calibration, Calibration6DoF &average )
{

	img = Image::Image( img.computeIDTEdgeImage() );
	cv::imwrite("IDTE.png", img.img);
	pcl::PointCloud<pcl::PointXYZI> pc4fined;

	Eigen::Affine3f transf = 
		pcl::getTransformation(x_rough, y_rough, z_rough, 0.0, 0.0, 0.0);
	transformPointCloud(edges, pc4fined, transf);


    float x_min = x_rough - max_translation;
    float y_min = y_rough - max_translation;
    float z_min = z_rough - max_translation;
    float x_rot_min = -max_rotation;
    float y_rot_min = -max_rotation;
    float z_rot_min = -max_rotation;

    float step_transl = max_translation * 2 / (steps - 1);
    float step_rot = max_rotation * 2 / (steps - 1);


    float rough_val = Similarity::edgeSimilarity(img, pc4fined, P);
    best_calibration.set(x_rough, y_rough, z_rough, 0, 0, 0, rough_val);
	
    cout << "rough:\t";
    best_calibration.print();


    float x = x_min;
    for (size_t xi = 0; xi < steps; xi++)
	{
      float y = y_min;
      for (size_t yi = 0; yi < steps; yi++)
      {
        float z = z_min;
        for (size_t zi = 0; zi < steps; zi++)
        {
		  std::cout << "fine calibration: " << "xi = " << xi 
					<< ", yi = " << yi << ", zi = " << zi << std::endl;
          float x_r = x_rot_min;
          for (size_t x_ri = 0; x_ri < steps; x_ri++)
          {
            float y_r = y_rot_min;
            for (size_t y_ri = 0; y_ri < steps; y_ri++)
            {
              float z_r = z_rot_min;
              for (size_t z_ri = 0; z_ri < steps; z_ri++)
              {
				transf = getTransformation(x, y, z, x_r, y_r, z_r);
				transformPointCloud(edges, pc4fined, transf);

                float value = Similarity::edgeSimilarity(img, pc4fined, P);
                Calibration6DoF calibration(x, y, z, x_r, y_r, z_r, value);
                if (value > best_calibration.value)
                {
                  best_calibration.set(x, y, z, x_r, y_r, z_r, value);
                  calibration.print();
                }

                z_r += step_rot;
              }
              y_r += step_rot;
            }
            x_r += step_rot;
          }
          z += step_transl;
        }
        y += step_transl;
      }
      x += step_transl;
	}
}


 
///////////   CALIBRATION   ////////////
Calibration6DoF calibration(bool doRefinement = false)
{
	Mat frame_gray;
	cvtColor(frame_rgb, frame_gray, CV_BGR2GRAY);
	// write this gray image
	cv::imwrite("frame_gray.png", frame_gray);
  
	// Marker detection:
	Calibration3DMarker marker(frame_gray, projection_matrix, 
		  pointcloud.getPointCloud(), STRAIGHT_DISTANCE, RADIUS, EDGE_THRESHOLD);

	vector<float> radii2D;
	vector<Point2f> centers2D;
 
	if (!marker.detectCirclesInImage(centers2D, radii2D))
	{
		return Calibration6DoF::wrong();
	}
	float radius2D = accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();
	std::cout << "radius2d: " << radius2D << std::endl;
 
	vector<float> radii3D;
	vector<Point3f> centers3D;
	if (!marker.detectCirclesInPointCloud(centers3D, radii3D))
	{
		return Calibration6DoF::wrong();
	}
	std::cout << "centers3D: "<< std::endl
			  << centers3D[0] << std::endl
			  << centers3D[1] << std::endl
	          << centers3D[2] << std::endl
			  << centers3D[3] << std::endl;

	pcl::PointCloud< pcl::PointXYZ > edges;

	for(size_t n = 0; n < marker.line1_pc.points.size(); n++)
		edges.push_back(
				pcl::PointXYZ( marker.line1_pc.points[n].x,
							   marker.line1_pc.points[n].y,
							   marker.line1_pc.points[n].z));
	for(size_t n = 0; n < marker.line2_pc.points.size(); n++)
		edges.push_back(
				pcl::PointXYZ( marker.line2_pc.points[n].x,
							   marker.line2_pc.points[n].y,
							   marker.line2_pc.points[n].z)); 

	pcl::io::savePCDFile("twoLines.pcd", edges);	

	edges += marker.circle_cloud[0];
	edges += marker.circle_cloud[1];
	edges += marker.circle_cloud[2];
	edges += marker.circle_cloud[3];

	pcl::io::savePCDFile("edges.pcd", edges);


	Calibration6DoF translation =
		coarseCalibration( projection_matrix, centers2D, radii2D, centers3D, RADIUS);
	translation.print();

	if (doRefinement)
	{
		std::cout<<" Refinement process started, this take minutes. " << std::endl;
		Calibration6DoF best_calibration, avg_calibration;
		calibrationRefinement( Image::Image(frame_gray), marker.edgesProject2plane,
		projection_matrix, translation.DoF[0], translation.DoF[1], translation.DoF[2],
			0.1, 0.1, 14, best_calibration, avg_calibration );
		
		return best_calibration;
	}
	else
      return translation;
}


#ifdef USE_RECORDED_FILES

int LoadRecordedData()
{
  frame_rgb = cv::imread("frame_rgb.png");
  std::cout << "Loading camera image... " << std::endl;
  
  std::cout << "Loading projection matrix..." << std::endl;
  float p[9] = { 6.71134705e+02,             0., 6.61914856e+02, 
	            	          0., 6.71134705e+02, 3.57206055e+02, 
							  0.,             0.,             1.};
  cv::Mat(3, 3, CV_32FC1, &p).copyTo(projection_matrix);

//  std::cout << projection_matrix << std::endl;

  PointCloud<Velodyne::Point> pc;

  //* load the file
	if (pcl::io::loadPCDFile<Velodyne::Point> ("velodyne_pc.pcd", pc) == -1)
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << pc.width * pc.height
			  << " data points from velodyne_pc.pcd with the following fields: "
			  << std::endl;

	pointcloud = Velodyne::Velodyne(pc);

	std::cout << "------- loading data has complated! ------- " << std::endl;
//	Velodyne::Velodyne test_cloud = pointcloud.transform(0, 0, 0, M_PI/2, 0, 0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");

  ros::NodeHandle nh_;
  nh_.getParam("/but_calibration_camera_velodyne/marker/circles_distance",
		     STRAIGHT_DISTANCE);
  nh_.getParam("/but_calibration_camera_velodyne/marker/circles_radius",
		     RADIUS);
  nh_.getParam("EDGE_THRESHOLD", EDGE_THRESHOLD);
  std::cout << "STRAIGHT_DISTANCE: " << STRAIGHT_DISTANCE << std::endl;
  std::cout << "RADIUS: " << RADIUS << std::endl;
  
	int c;
	while ((c = getopt(argc, argv, "r")) != -1)
	{
		switch (c)
		{
			case 'r': doRefinement = true; break;
			default : return EXIT_FAILURE;
		}
	}

	while(nh_.ok())
	{
		LoadRecordedData();
		std::cout << " Start calibration... " << std::endl;

		Calibration6DoF calibrationParams = calibration(doRefinement);

		if( calibrationParams.isGood() )
		{
			ROS_INFO_STREAM("Calibration succeeded, found parameters:");
			shutdown();
		}
		else
		{
			ROS_WARN("Calibration failed - trying again after 3s ...");
			ros::Duration(3).sleep();
		}	  
	}
	return 0;
}

#else

void callback(const sensor_msgs::ImageConstPtr& msg_img,
		      const sensor_msgs::CameraInfoConstPtr& msg_info,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{

  ROS_INFO_STREAM("Image received at " << msg_img->header.stamp.toSec());
  ROS_INFO_STREAM("Camera info received at " << msg_info->header.stamp.toSec());
  ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());

  // Loading camera image:
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy( msg_img, 
		                       sensor_msgs::image_encodings::BGR8 );
  frame_rgb = cv_ptr->image;

  // Loading projection matrix:
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin();
		  i != msg_info->P.end(); i++)
  {
    *pp = (float)(*i);
	pp++;
  }
	cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);


	// Velodyne point cloud
	PointCloud<Velodyne::Point> pc;
	fromROSMsg(*msg_pc, pc);
	pointcloud = Velodyne::Velodyne(pc);

	// save pointcloud data, use later
	pointcloud.save("velodyne_ori.pcd");
	Eigen::Affine3f transf = getTransformation(x, y, z, rot_x, rot_y, rot_z);
	pcl::transformPointCloud(pointcloud, pointcloud, transf);

	//pointcloud = pointcloud.transform(0, 0, 0, M_PI/2, -M_PI/2, 0);
	// z y x must clear this??
	pointcloud.save("velodyne_pc.pcd");
	cv::imwrite("frame_rgb.png", frame_rgb);
	cv::FileStorage fs_P("projection.yml", cv::FileStorage::WRITE);
	fs_P << "P" << projection_matrix;
	fs_P.release();

	// calibration:
	Calibration6DoF calibrationParams = calibration(doRefinement);
	if( calibrationParams.isGood() )
	{
		ROS_INFO_STREAM("Calibration succeeded, found parameters:");
		calibrationParams.print();
		shutdown();
	}
	else
	{
		ROS_WARN("Calibration failed - trying again after 3s ...");
		ros::Duration(3).sleep();
	}
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "calibration_node");

  int c;
  while ((c = getopt(argc, argv, "r")) != -1)
  {
    switch (c)
    {
      case 'r':
        doRefinement = true;
        break;
      default:
        return EXIT_FAILURE;
    }
  }

  ros::NodeHandle n;
  n.getParam("/but_calibration_camera_velodyne/camera_frame_topic",
		     CAMERA_FRAME_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/camera_info_topic", 
		     CAMERA_INFO_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_topic",
		     VELODYNE_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/marker/circles_distance",
		     STRAIGHT_DISTANCE);
  n.getParam("/but_calibration_camera_velodyne/marker/circles_radius",
		     RADIUS);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, CAMERA_FRAME_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo>info_sub(n,CAMERA_INFO_TOPIC,1);
  message_filters::Subscriber<sensor_msgs::PointCloud2>cloud_sub(n,VELODYNE_TOPIC, 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image,
		  sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, cloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return EXIT_SUCCESS;
}

#endif
