/*
 * Calibration3DMarker.cpp
 *
 *  Created on: 1.1.2017
 *      Author: suijingfeng
 */

#include "but_calibration_camera_velodyne/Calibration3DMarker.h"

#include <ros/assert.h>

#define SAVE_TEMPORARY_DATA		1
//#define DATA_TYPE				1


using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;



Calibration3DMarker::Calibration3DMarker(cv::Mat _frame_gray, cv::Mat _P, 
                                         pcl::PointCloud<Velodyne::Point> _pc,
                                         float _circ_distance, float _radius, float _edge_threshold) :
    frame_gray(_frame_gray), P(_P), pc(_pc),
    circ_distance(_circ_distance), radius(_radius), edge_threshold(_edge_threshold),
	rings(Calibration6DoF::N_RINGS)
{
	
	std::cout << "---------------- detect plane ----------------" << std::endl;

	pcl::PointCloud<Velodyne::Point> no_back_plane;
	pcl::PointCloud<Velodyne::Point> no_plane;
	pcl::PointCloud<Velodyne::Point> fine_front_plane;
	pcl::PointCloud<Velodyne::Point> fine_front_plane_outliers;
	pcl::PointCloud<Velodyne::Point> visible_cloud;

	#ifdef DATA_TYPE
	//////////////////////// RANSAC FRONT PLANE //////////////////////////
	detectPlane(pc, 0.04, front_plane_coefficients, front_plane, no_back_plane);

	//////////////////////// FINED FRONT PLANE //////////////////////////
	detectPlane(front_plane, 0.02, front_plane_coefficients,
		   	fine_front_plane, fine_front_plane_outliers);

	//////////////////////// RANSAC BACK PLANE //////////////////////////
	detectPlane(no_back_plane, 0.02, back_plane_coefficients, back_plane, no_plane);
	#else
	
	//////////////////////// RANSAC BACK PLANE //////////////////////////
	detectPlane(pc, 0.05, back_plane_coefficients, back_plane, no_back_plane );

	//////////////////////// RANSAC FRONT PLANE //////////////////////////
	detectPlane(no_back_plane, 0.05, front_plane_coefficients, front_plane, no_plane);

	//////////////////////// FINED FRONT PLANE //////////////////////////
	detectPlane(front_plane, 0.02, front_plane_coefficients,
		   	fine_front_plane, fine_front_plane_outliers);
	#endif
	
	#ifdef SAVE_TEMPORARY_DATA
	pcl::io::savePCDFile("back_plane.pcd", back_plane );
	pcl::io::savePCDFile("front_plane.pcd", front_plane );
	pcl::io::savePCDFile("fined_front_plane.pcd", fine_front_plane );
	pcl::io::savePCDFile("no_plane.pcd", no_plane );
	#endif
	
	// Get Rings -> Edge Detection -> Normalize -> findEdges -> project //
    // to do: examine the effectiveness edges detectors
	Velodyne::Velodyne scan(pc);
	//Velodyne::Velodyne scan(no_plane);
	scan.getRings();
	scan.intensityByDiff();
	scan.project(P, Rect(0, 0, 1280, 720), &visible_cloud); //
	scan.point_cloud = visible_cloud;

//	scan.normalizeIntensity(0.0, 1.0);
	scan.threshold(0.03, edges_pc);

	pcl::PointCloud<pcl::PointXYZI> OneLineLfeft;
	pcl::PointCloud<pcl::PointXYZI> NoLineLfeft;

	// when edges is extrated, rings and intensity information is not need  //  
	perspectiveProject(edges_pc, front_plane_coefficients, edgesProject2plane);
	// after the step, types of edgesProject2plane is pcl::PointXYZ

	#ifdef SAVE_TEMPORARY_DATA
		pcl::io::savePCDFile( "edges_scan.pcd", edges_pc);
		pcl::io::savePCDFile( "pointsProjectedtoPlane.pcd", edgesProject2plane);
	#endif

	std::cout << "---------------- detect lines ----------------" << std::endl;

	detectLine(edgesProject2plane, 0.02, line1_coefficients, line1_pc, OneLineLfeft);
	detectLine(OneLineLfeft, 0.02, line2_coefficients, line2_pc, NoLineLfeft);

	four_circles.clear();


	for (size_t i = 1; i < NoLineLfeft.size (); ++i)
	{
		four_circles.push_back( PointXYZ(	NoLineLfeft.points[i].x, 
											NoLineLfeft.points[i].y,
											NoLineLfeft.points[i].z ));
	}

} // END OF Calibration3DMarker Constructor


void Calibration3DMarker::getRings( pcl::PointCloud<Velodyne::Point>& inPC )
{
	rings.clear();
	for (pcl::PointCloud<Velodyne::Point>::iterator 
			pt = inPC.begin(); pt < inPC.end(); pt++)
	{
		pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
		rings[pt->ring].push_back( *pt );
	}
}

/*  
void Calibration3DMarker::intensityByDiff()
{
//	std::vector<std::vector< Velodyne::Point > > rings = getRings(inPC);
	for (std::vector<std::vector< Velodyne::Point> >::iterator 
			ring = this->rings.begin(); ring < this->rings.end(); ring++)
	{
		Velodyne::Point prev;
	    Velodyne::Point succ;
		if( ring->empty() )
			continue;

		float last_intensity = (*ring->begin()).intensity;
		float new_intensity;
		( *ring->begin() ).intensity = 0;
		( *(ring->end()-1)).intensity = 0;

		for (vector<Velodyne::Point>::iterator pt = ring->begin() + 1;
									  pt < ring->end() - 1; pt++)
		{
			prev = *(pt - 1);
			succ = *(pt + 1);
			// (*pt)->intensity = 10 * std::max( 
			// std::max( prev->range-(*pt)->range, succ->range-(*pt)->range), 0);
			pt->intensity = 10 * std::max( abs( prev.range - pt->range), 
											  abs( succ.range - pt->range ) );
		}
	}
}
*/

void Calibration3DMarker::detectLine( pcl::PointCloud<pcl::PointXYZI>& in_pc,
									   float threshold,
									   Eigen::VectorXf &line_coefficients,
									   pcl::PointCloud<pcl::PointXYZI> &inliers_pc,
									   pcl::PointCloud<pcl::PointXYZI> &outliers_pc )
{
	//// RANSAC ANAIN, IN ORDER TO PROJECT ALL POINTS TO FRONT PLANE ////
	std::vector<int> inliers_indicies;
	std::vector<int> outliers_indicies;

	pcl::PointCloud<pcl::PointXYZI>::Ptr 
		plane_ptr( new PointCloud<PointXYZI>( in_pc ) );
 
	SampleConsensusModelLine<PointXYZI>::Ptr model_l(
			new SampleConsensusModelLine<PointXYZI>(plane_ptr) );
	
	RandomSampleConsensus<PointXYZI> ransac_l(model_l);
	ransac_l.setDistanceThreshold(0.02);		
	ransac_l.computeModel();
	ransac_l.getInliers( inliers_indicies );
	ransac_l.getModelCoefficients( line_coefficients );
	
	std::cout << "------ line coefficients ------" << std::endl
			  << line_coefficients  << std::endl;
	std::cout << "plane inlier size: " << inliers_indicies.size() << std::endl;

	///////////    find outliers indices    ///////////  
	for (size_t i = 0; i < in_pc.size(); i++)
    {
        if( find(inliers_indicies.begin(), inliers_indicies.end(), i)
            == inliers_indicies.end() )
        {
			outliers_indicies.push_back(i);
        }
    }

	pcl::copyPointCloud<pcl::PointXYZI>(in_pc, inliers_indicies, inliers_pc);
	pcl::copyPointCloud<pcl::PointXYZI>(in_pc, outliers_indicies, outliers_pc);
}



void Calibration3DMarker::detectLine( pcl::PointCloud<pcl::PointXYZ>& in_pc,
									   float threshold,
									   Eigen::VectorXf &line_coefficients,
									   pcl::PointCloud<pcl::PointXYZ> &inliers_pc,
									   pcl::PointCloud<pcl::PointXYZ> &outliers_pc )
{
	//// RANSAC ANAIN, IN ORDER TO PROJECT ALL POINTS TO FRONT PLANE ////
	std::vector<int> inliers_indicies;
	std::vector<int> outliers_indicies;

	pcl::PointCloud<pcl::PointXYZ>::Ptr 
		plane_ptr( new PointCloud<PointXYZ>( in_pc ) );
 
	SampleConsensusModelLine<PointXYZ>::Ptr model_l(
			new SampleConsensusModelLine<PointXYZ>(plane_ptr) );
	
	RandomSampleConsensus<PointXYZ> ransac_l(model_l);
	ransac_l.setDistanceThreshold(0.02);		
	ransac_l.computeModel();
	ransac_l.getInliers( inliers_indicies );
	ransac_l.getModelCoefficients( line_coefficients );
	
	std::cout << "------ line coefficients ------" << std::endl
			  << line_coefficients  << std::endl;
	std::cout << "plane inlier size: " << inliers_indicies.size() << std::endl;

	///////////    find outliers indices    ///////////  
	for (size_t i = 0; i < in_pc.size(); i++)
    {
        if( find(inliers_indicies.begin(), inliers_indicies.end(), i)
            == inliers_indicies.end() )
        {
			outliers_indicies.push_back(i);
        }
    }

	pcl::copyPointCloud<pcl::PointXYZ>(in_pc, inliers_indicies, inliers_pc);
	pcl::copyPointCloud<pcl::PointXYZ>(in_pc, outliers_indicies, outliers_pc);
}




bool Calibration3DMarker::detectPlane( pcl::PointCloud<PointXYZ>::Ptr pc_ptr,
									   Eigen::VectorXf &plane_coefficients ,
									   pcl::PointCloud<pcl::PointXYZ> &out_pc )
{
	//// RANSAC ANAIN, IN ORDER TO PROJECT ALL POINTS TO FRONT PLANE ////
	std::vector<int> inliers_indicies;

	SampleConsensusModelPlane<PointXYZ>::Ptr model_plane(
		new SampleConsensusModelPlane<pcl::PointXYZ>( pc_ptr ) );

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_again( model_plane );

	ransac_again.setDistanceThreshold(0.03);
	ransac_again.computeModel();
	ransac_again.getInliers(inliers_indicies);
	ransac_again.getModelCoefficients(plane_coefficients);
	PointCloud<PointXYZ> ProjetedtoPlane;
	model_plane->projectPoints(inliers_indicies, plane_coefficients, out_pc);
	pcl::copyPointCloud<pcl::PointXYZ>(*pc_ptr, inliers_indicies, out_pc);
}


bool Calibration3DMarker::detectPlane( pcl::PointCloud<Velodyne::Point>& in_pc,
									   float threshold,
									   Eigen::VectorXf &plane_coefficients,
									   pcl::PointCloud<Velodyne::Point> &inliers_pc,
									   pcl::PointCloud<Velodyne::Point> &outliers_pc )
{
	//// RANSAC ANAIN, IN ORDER TO PROJECT ALL POINTS TO FRONT PLANE ////
	std::vector<int> inliers_indicies;
	std::vector<int> outliers_indicies;

	PointCloud<PointXYZ> new_cloud;
	for (PointCloud<Velodyne::Point>::iterator pt = in_pc.points.begin();
                                     pt < in_pc.points.end(); pt++)
	{
		new_cloud.push_back(PointXYZ(pt->x, pt->y, pt->z));
	}

	PointCloud<PointXYZ>::Ptr new_cloud_ptr(new PointCloud<PointXYZ>(new_cloud) );

	SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(
		new SampleConsensusModelPlane<pcl::PointXYZ>( new_cloud_ptr ));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_plane( model_plane );

	ransac_plane.setDistanceThreshold(threshold);
	ransac_plane.computeModel();
	ransac_plane.getInliers(inliers_indicies);
	ransac_plane.getModelCoefficients(plane_coefficients);
	///////////    find outliers indices    ///////////  
	for (size_t i = 0; i < in_pc.size(); i++)
    {
        if( find(inliers_indicies.begin(), inliers_indicies.end(), i)
            == inliers_indicies.end() )
        {
			outliers_indicies.push_back(i);
        }
    }
	std::cout << "plane inlier size: " << inliers_indicies.size() << std::endl;

	pcl::copyPointCloud<Velodyne::Point>(in_pc, inliers_indicies, inliers_pc);
	pcl::copyPointCloud<Velodyne::Point>(in_pc, outliers_indicies, outliers_pc);

}


void Calibration3DMarker::perspectiveProject( pcl::PointCloud<Velodyne::Point> &in_pc,
								            Eigen::VectorXf &plane_coefficients,
											pcl::PointCloud<pcl::PointXYZI> &out_pc)
{
	out_pc.clear();
	float a = plane_coefficients[0];
	float b = plane_coefficients[1];
	float c = plane_coefficients[2];
	float d = plane_coefficients[3];
	pcl::PointXYZI p;
	for ( pcl::PointCloud<Velodyne::Point>::iterator pt = in_pc.points.begin();
                                     pt < in_pc.points.end(); pt++ )
	{
		float k = -d /( a * pt->x + b * pt->y + c * pt->z );
		p.x = k * pt->x;
		p.y = k * pt->y;
		p.z = k * pt->z;
		p.intensity = pt->intensity;
		out_pc.push_back(p);
	}
}


void Calibration3DMarker::perspectiveProject( pcl::PointCloud<pcl::PointXYZ> &in_pc,
											  Eigen::VectorXf &plane_coefficients, 
											  pcl::PointCloud<pcl::PointXYZ> &out_pc)
{
	float a = plane_coefficients[0];
	float b = plane_coefficients[1];
	float c = plane_coefficients[2];
	float d = plane_coefficients[3];

	pcl::PointXYZ p;

	for (PointCloud<pcl::PointXYZ>::iterator pt = in_pc.points.begin();
                                     pt < in_pc.points.end();
									 pt++)
	{
		pcl::PointXYZ p = *pt;
		float k = -d /( a * p.x + b * p.y + c * p.z );
		out_pc.push_back( pcl::PointXYZ( k * p.x, k * p.y, k * p.z) );
	}
}


bool Calibration3DMarker::detectCirclesInImage(vector<Point2f> &centers, 
                                               vector<float> &radiuses)
{
//	Image::Image img(frame_gray);
	Image::Image img_edge(frame_gray);
	img_edge.computeEdgeImage();

	return img_edge.detect4Circles( Calibration3DMarker::CANNY_THRESH, 
                                    Calibration3DMarker::CENTER_THRESH_DISTANCE,
                                    centers, radiuses );
}



bool Calibration3DMarker::detectCirclesInPointCloud( vector<Point3f> &centers,
                                                     vector<float> &radiuses )
{
	std::cout << "Detect Circles In Point Cloud ......" << std::endl;
	PointCloud<PointXYZ>::Ptr detection_cloud(
			new PointCloud<PointXYZ>(four_circles));

	std::vector<pcl::PointXYZ> centres = 
		detect4CircleInPointcloud(detection_cloud, radiuses);

    if (centres.size() == 4)
    {
	  order4spheres(centres);

      if( verify4spheres(centres, this->circ_distance, 0.035))
	  {	   
        for (size_t i = 0; i < centres.size(); i++)
		{		
		  centers.push_back( Point3f( centres[i].x, centres[i].y, centres[i].z) );
		}
	    return true;
	  }
	  else
	  {
		std::cout << "Verify4spheres failed! " << std::endl;
	    return false;
	  }	
    }
	else
	{
		std::cout <<"Spheres_centers.size(): " << centres.size() <<std::endl;
		return false;
	}
    return true;
}


void Calibration3DMarker::removeNearbyPoints( pcl::PointCloud<PointXYZ> &cloud_in,
											  pcl::PointXYZ center,
											  float threshold,
											  pcl::PointCloud<PointXYZ> &cloud_out )
{
	threshold *= threshold;
	for(PointCloud<PointXYZ>::iterator pt=cloud_in.begin(); pt < cloud_in.end(); pt++)
	{
		float dis = (pt->x - center.x) * (pt->x - center.x) + 
					(pt->y - center.y) * (pt->y - center.y) +
				   	(pt->z - center.z) * (pt->z - center.z);
		if(dis > threshold)
			cloud_out.points.push_back(*pt); 
	}

//	std::cout << "Number of points removed: " 
//		      << cloud_in.size() - cloud_out.size() << std::endl;
}



///////////// Detecting 4 3D circles in pointcloud ///////////////////////////

std::vector<PointXYZ> Calibration3DMarker::detect4CircleInPointcloud(
		 PointCloud<PointXYZ>::Ptr plane_no_line, vector<float> &radiuses)
{
	for(int i = 0; i < 4; i++)
	{
		std::vector<int> circle_inliers;
		Eigen::VectorXf circle_coefficients;

		SampleConsensusModelCircle3D<PointXYZ>:: Ptr modle_3dCircle(	
			new SampleConsensusModelCircle3D<PointXYZ>( plane_no_line ) );
		
		RandomSampleConsensus<PointXYZ> ransac_3dCircle(modle_3dCircle);
		modle_3dCircle->setRadiusLimits(0.08, 0.085);

		ransac_3dCircle.setDistanceThreshold(0.03);
		ransac_3dCircle.computeModel();
		ransac_3dCircle.getInliers(circle_inliers);
		ransac_3dCircle.getModelCoefficients(circle_coefficients);

		std::cout << "------------ circle" << i << "------------" << std::endl
		          << "Inliers: " << circle_inliers.size() << std::endl;
		std::cout << " cofficients: "  << std::endl << circle_coefficients
		          << std::endl << std::endl;
			

		centers3d_.push_back( pcl::PointXYZ( circle_coefficients[0], 
											 circle_coefficients[1],
										 	 circle_coefficients[2]) );
		
		radiuses3d[i]  = circle_coefficients[3];
		radiuses.push_back(circle_coefficients[3]);

		copyPointCloud<PointXYZ>(*plane_no_line, circle_inliers, circle_cloud[i]);		
  		pcl::io::savePCDFile("circle" + to_string(i) + ".pcd", circle_cloud[i]);

		pcl::PointCloud<pcl::PointXYZ>::Ptr theRestofCircles(
							new pcl::PointCloud<pcl::PointXYZ>() );

		removeNearbyPoints( *plane_no_line, centers3d_[i], 0.115, *theRestofCircles);
		plane_no_line = theRestofCircles;

	}
//	pcl::io::savePCDFile("no_circle.pcd", *plane_no_line);

	return centers3d_; 
}	


/* Indexes of circles in marker:
 * 0 1
 * 2 3
 */
bool orderX(PointXYZ p1, PointXYZ p2)
{
	return p1.x < p2.x;
}
bool orderY(PointXYZ p1, PointXYZ p2)
{
	return p1.y < p2.y;
}


void Calibration3DMarker::order4spheres(vector<PointXYZ> &circles_centers)
{
	ROS_ASSERT(circles_centers.size() == 4);
	sort(circles_centers.begin(), circles_centers.end(), orderY);
	sort(circles_centers.begin(), circles_centers.begin() + 2, orderX);
	sort(circles_centers.begin() + 2, circles_centers.begin() + 4, orderX);

	std::cout << "after ordered, centers: " << endl;
	for (size_t i = 0; i < circles_centers.size(); i++)
	{
		std::cout << circles_centers[i] << std::endl;
	}
}


float euclid_dist(const PointXYZ p1, const PointXYZ p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}


bool Calibration3DMarker::verify4spheres( vector<PointXYZ> &circles_centers, 
										  float sqr_len, float delta )
{
	ROS_ASSERT(circles_centers.size() == 4);

	float dis0 = abs(euclid_dist(circles_centers[0],circles_centers[1]) - sqr_len);
	float dis1 = abs(euclid_dist(circles_centers[1],circles_centers[3]) - sqr_len);
	float dis2 = abs(euclid_dist(circles_centers[3],circles_centers[2]) - sqr_len);
	float dis3 = abs(euclid_dist(circles_centers[2],circles_centers[0]) - sqr_len);

	if( (dis0 < delta) && (dis1 < delta) && (dis2 < delta) && (dis3 < delta))
	{ 
		std::cout << "[*_*] veritfy 4 circles OK! " << std::endl;
		return true;
	}
	else
	{	  
		std::cout << " veritfy 4 circles failed "  << std::endl;
		std::cout << "dis0: " << dis0 << std::endl;
		std::cout << "dis1: " << dis1 << std::endl;
		std::cout << "dis2: " << dis2 << std::endl;
		std::cout << "dis3: " << dis3 << std::endl;
	}
	return false;
}
