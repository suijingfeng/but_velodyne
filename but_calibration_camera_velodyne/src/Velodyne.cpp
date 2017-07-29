/*
 * Velodyne.cpp
 *
 *  Created on: 26.11.2013
 *      Author: ivelas
 */

#include <vector>
#include <cmath>

#include "but_calibration_camera_velodyne/Velodyne.h"
#include "but_calibration_camera_velodyne/Image.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/assert.h>

//#define DEBUG	1
using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;


Velodyne::Velodyne::Velodyne(PointCloud<Point> _point_cloud) :
    point_cloud(_point_cloud){}

  
void Velodyne::Velodyne::transform( float x, float y, float z, 
									float rot_x, float rot_y, float rot_z)
{
	Eigen::Affine3f transf = getTransformation(x, y, z, rot_x, rot_y, rot_z);
	//	PointCloud<Point> new_cloud;
	pcl::transformPointCloud(point_cloud, point_cloud, transf);
}


void Velodyne::Velodyne::transform(vector<float> DoF)
{
  ROS_ASSERT(DoF.size() == 6);
  return transform(DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);
}


///////////////////////////////////////////////////////////////////////////
void Velodyne::Velodyne::project( Mat projection_matrix,
								  cv::Rect frame,
								  pcl::PointCloud<Point> *visible_points)
{
//	cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);
  	cv::Mat pt_3D(3, 1, CV_32FC1);
	cv::Mat pt_2D;
//	cv::Point xy;
	visible_points->clear();
	for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
                                   pt < point_cloud.points.end(); pt++)
	{
		// behind the camera
		if (pt->z < 0)
          continue; 
		//float intensity = pt->intensity;
		//cv::Point2f xy = projectf(*pt, projection_matrix);
		pt_3D.at<float>(0) = pt->x;
		pt_3D.at<float>(1) = pt->y;
		pt_3D.at<float>(2) = pt->z;

		pt_2D = projection_matrix * pt_3D;

		cv::Point xy( pt_2D.at<float>(0) / pt_2D.at<float>(2),
			      	  pt_2D.at<float>(1) / pt_2D.at<float>(2) );

		if (xy.inside(frame))
		{
			if (visible_points != NULL)
			{
				visible_points->push_back(*pt);
			}
//		    cv::circle(plane, xy, 3, intensity, -1);
//			plane.at<float>(xy) = 255;
		}
	}
//	cv::imwrite("VproIMG.png", plane);
}


std::vector<std::vector<Velodyne::Point*> > Velodyne::Velodyne::getRings()
{
	vector<vector<Point* > > rings(Velodyne::Velodyne::RINGS_COUNT);
	for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); 
									 pt < point_cloud.points.end(); pt++)
	{
		ROS_ASSERT(pt->ring < RINGS_COUNT);
		pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
		rings[pt->ring].push_back( &(*pt) );
	}
	return rings;
}


void Velodyne::Velodyne::intensityByDiff()
{
//#define USE_Z_DIRECTION_DISCONTINUE	1
	std::vector<std::vector< Point* > > rings = this->getRings();
	for (vector<vector<Point*> >::iterator ring = rings.begin();
										   ring < rings.end(); ring++)
	{
		Point *prev;
		Point *prevprev;
	    Point *succ;
	    Point *succsucc;
		if( ring->empty() )
			continue;

		float last_intensity = (*ring->begin())->intensity;
		float new_intensity;
		( *ring->begin() )->intensity = 0;
		( *ring->begin()+1 )->intensity = 0;
		( *(ring->end()-1) )->intensity = 0;
		( *(ring->end()-2) )->intensity = 0;

		for (vector<Point*>::iterator pt = ring->begin() + 2;
									  pt < ring->end() - 2; pt++)
		{
			prevprev = *(pt - 2);
			prev = *(pt - 1);
			succ = *(pt + 1);
			succsucc = *(pt + 2);

			#ifdef USE_Z_DIRECTION_DISCONTINUE
			(*pt)->intensity = pow( abs( succ->z + succsucc->z  - prevprev->z-prev->z ), 1.5);
			#else 
			(*pt)->intensity = pow( abs( succ->range + succsucc->range  - prevprev->range-prev->range ), 1.2);
			#endif
		}
	}
}


pcl::PointCloud<pcl::PointXYZ> *Velodyne::Velodyne::toPointsXYZ()
{
  PointCloud<PointXYZ> *new_cloud = new PointCloud<PointXYZ>();
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
                                   pt < point_cloud.points.end(); pt++)
  {
    new_cloud->push_back( PointXYZ(pt->x, pt->y, pt->z) );
  }
  
  return new_cloud;
}


// all intensities to range min-max
void Velodyne::Velodyne::normalizeIntensity(float min, float max)
{
  float min_found = INFINITY;
  float max_found = -INFINITY;

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
                                   pt < point_cloud.points.end(); pt++)
  {
    max_found = std::max(max_found, pt->intensity);
    min_found = std::min(min_found, pt->intensity);
  }

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
                                   pt < point_cloud.points.end(); pt++)
  {
    pt->intensity = 
       (pt->intensity - min_found)/(max_found - min_found) * (max - min) + min;
  }
}


void Velodyne::Velodyne::threshold(float thresh, pcl::PointCloud<Point>& out_pc )
{
	out_pc.clear();
	for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
									 pt < point_cloud.points.end(); pt++)
	{
		if (pt->intensity > thresh)
		{
			out_pc.push_back(*pt);
		}
	}
}


void Velodyne::Velodyne::detectPlanes(cv::Mat projection)
{
  PointCloud<Point> visible_points;
  this->project(projection, Rect(0, 0, 1280, 720), &visible_points);
  // ...
}


vector<Velodyne::Velodyne> Velodyne::Velodyne::depthSegmentation(int segment_counts)
{
  vector<Velodyne> segments(segment_counts);

  Mat ranges(point_cloud.size(), 1, CV_32FC1);
//	Mat indicies(point_cloud.size(), 1, CV_32SC1);
  for (int i = 0; i < point_cloud.size(); i++)
  {
    ranges.at<float>(i) = point_cloud[i].range;
  }
  //kmeans(ranges, segment_counts, indicies, TermCriteria(TermCriteria::MAX_ITER, 3, 0), 3, KMEANS_PP_CENTERS);

  Mat ranges_uchar;
  normalize(ranges, ranges_uchar, 0, 255, NORM_MINMAX, CV_8UC1);
  Mat indicies(point_cloud.size(), 1, CV_8UC1);
  cv::threshold(ranges_uchar, indicies, 0, 1, THRESH_BINARY + THRESH_OTSU);

  for (int i = 0; i < point_cloud.size(); i++)
  {
    segments[indicies.at<uchar>(i)].push_back(point_cloud[i]);
  }

  return segments;
}


PointCloud<PointXYZRGB> Velodyne::Velodyne::colour(cv::Mat frame_rgb, cv::Mat P)
{
  PointCloud<PointXYZRGB> color_cloud;
  for (PointCloud<Point>::iterator pt = this->point_cloud.begin();
                                   pt < this->point_cloud.end(); pt++)
  {
    Point2f xy = Velodyne::Velodyne::projectf(*pt, P);

    Vec3b rgb = Image::Image::atf(frame_rgb, xy);
    PointXYZRGB pt_rgb(rgb.val[2], rgb.val[1], rgb.val[0]);
    pt_rgb.x = pt->x;
    pt_rgb.y = pt->y;
    pt_rgb.z = pt->z;

    color_cloud.push_back(pt_rgb);
  }
  return color_cloud;
}

