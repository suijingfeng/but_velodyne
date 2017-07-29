/*
 * Calibration3DMarker.h
 *
 *  Created on: 2.4.2014
 *      Author: ivelas
 */

#ifndef CALIBRATION3DMARKER_H_
#define CALIBRATION3DMARKER_H_


#include <iostream>
#include <algorithm>
#include <cmath>

#include "opencv2/opencv.hpp"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>


#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Calibration.h>


class Calibration3DMarker
{

public:
	Calibration3DMarker( cv::Mat _frame_gray, cv::Mat _P, 
						pcl::PointCloud<Velodyne::Point> _pc,
						float _circ_distance, float _radius, float _edge_threshold);

	void getRings( pcl::PointCloud<Velodyne::Point>& inPC );
	void intensityByDiff( );

	bool detectCirclesInImage(std::vector<cv::Point2f> &centers, 
							  std::vector<float> &radiuses);

	bool detectCirclesInPointCloud(std::vector<cv::Point3f> &centers,
								   std::vector<float> &radiuses);

	bool detectPlane( pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr,
					Eigen::VectorXf &plane_coefficients, 
					  pcl::PointCloud<pcl::PointXYZ> &out_pc );

	void detectLine( pcl::PointCloud<pcl::PointXYZI>& in_pc,
					float threshold,
					Eigen::VectorXf &line_coefficients,
					pcl::PointCloud<pcl::PointXYZI> &inliers_pc,
					pcl::PointCloud<pcl::PointXYZI> &outliers_pc);


	void detectLine( pcl::PointCloud<pcl::PointXYZ>& in_pc, float threshold,
					 Eigen::VectorXf &line_coefficients,
					 pcl::PointCloud<pcl::PointXYZ> &inliers_pc,
					 pcl::PointCloud<pcl::PointXYZ> &outliers_pc );


	void perspectiveProject( pcl::PointCloud<pcl::PointXYZ> &in_pc,
							 Eigen::VectorXf &plane_coefficients, 
						     pcl::PointCloud<pcl::PointXYZ> &out_pc);

	void perspectiveProject( pcl::PointCloud<Velodyne::Point>& in_pc,
							 Eigen::VectorXf &plane_coefficients, 
						     pcl::PointCloud<pcl::PointXYZI>& out_pc);

	pcl::PointCloud<pcl::PointXYZ>* perspectiveProject( 
					pcl::PointCloud<Velodyne::Point> &in_pc,
				  Eigen::VectorXf &plane_coefficients );

	/* this is detect 4 circles in pointcloud */
	std::vector<pcl::PointXYZ> detect4CircleInPointcloud(
				pcl::PointCloud<pcl::PointXYZ>::Ptr plane, 
				std::vector<float> &radiuses);

	void removeNearbyPoints( pcl::PointCloud<pcl::PointXYZ> &cloud_in,
						   pcl::PointXYZ center,
						   float threshold,
						   pcl::PointCloud<pcl::PointXYZ> &cloud_out);

	bool detectPlane( pcl::PointCloud<Velodyne::Point>& in_pc,
					  float threshold,
				      Eigen::VectorXf &plane_coefficients,
				      pcl::PointCloud<Velodyne::Point> &inliers_pc,
				      pcl::PointCloud<Velodyne::Point> &outliers_pc );


    template<typename PointT>
    void remove_inliers(const pcl::PointCloud<PointT> &cloud_in, 
                              std::vector<int> inliers_indices,
                              pcl::PointCloud<PointT> &cloud_out )
    {
      std::vector<int> outliers_indicies;
      for (size_t i = 0; i < cloud_in.size(); i++)
      {
        if (find(inliers_indices.begin(), inliers_indices.end(), i)
            == inliers_indices.end() )
        {
          outliers_indicies.push_back(i);
        }
      }
      pcl::copyPointCloud< pcl::PointXYZ >(
                  cloud_in, outliers_indicies, cloud_out);
    }

  /*
   * Indexes of circles in marker:
   * 0 1
   * 2 3
   */
	void order4spheres(std::vector< ::pcl::PointXYZ > &spheres_centers);

	bool verify4spheres(std::vector< ::pcl::PointXYZ > &spheres_centers,
						float straight_distance, float delta);

public:
	cv::Mat frame_gray, P;
	pcl::PointCloud<Velodyne::Point> pc;
	std::vector<std::vector<Velodyne::Point> > rings;
	pcl::PointCloud< pcl::PointXYZ > four_circles;
	pcl::PointCloud<pcl::PointXYZ> circle_cloud[4];

	float circ_distance, radius;
	float radiuses3d[4];
	std::vector<pcl::PointXYZ> centers3d_;
	
	Eigen::VectorXf line1_coefficients;
	Eigen::VectorXf line2_coefficients;
	pcl::PointCloud<pcl::PointXYZI> line1_pc;
	pcl::PointCloud<pcl::PointXYZI> line2_pc;

	pcl::PointCloud<Velodyne::Point> front_plane;
	Eigen::VectorXf front_plane_coefficients;

	pcl::PointCloud<Velodyne::Point> back_plane;
	Eigen::VectorXf back_plane_coefficients;

	pcl::PointCloud<Velodyne::Point> edges_pc;
	pcl::PointCloud<pcl::PointXYZI> edgesProject2plane;

	float edge_threshold;
	static const int CANNY_THRESH = 150;
	static const int CENTER_THRESH_DISTANCE = 80;
};

#endif /* CALIBRATION3DMARKER_H_ */
