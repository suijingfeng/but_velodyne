#ifndef __BUT_CALIBRATION_H__
#define __BUT_CALIBRATION_H__

#include <cstdlib>
#include <cstdio>
#include <math.h>       /* isnan, sqrt */
#include <iostream>

#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Similarity.h>


class Calibration6DoF
{
public:
	std::vector<float> DoF;
	float value; // NaN = wrong calibration
	static const unsigned int N_RINGS = 16;

public:
  Calibration6DoF(float x, float y, float z, 
                  float x_r, float y_r, float z_r, float val)
  {
    set(x, y, z, x_r, y_r, z_r, val);
  }
  
  Calibration6DoF()
  {
    value = 0;
    DoF.resize(6, 0);
  }

  
  static Calibration6DoF wrong()
  {
    return Calibration6DoF(0, 0, 0, 0, 0, 0, NAN);
  }

  bool isGood()
  {
    return !std::isnan(value);
  }

  void set(float x, float y, float z, float x_r, float y_r, float z_r, float val)
  {
    value = val;

    DoF.clear();
    DoF.push_back(x);
    DoF.push_back(y);
    DoF.push_back(z);
    DoF.push_back(x_r);
    DoF.push_back(y_r);
    DoF.push_back(z_r);
  }

  bool operator <=(Calibration6DoF &other)
  {
    return this->value <= other.value;
  }

  
  void operator +=(Calibration6DoF &other)
  {
    this->value += other.value;
    ROS_ASSERT(this->DoF.size() == other.DoF.size());
    for (size_t i = 0; i < DoF.size(); i++)
    {
      this->DoF[i] += other.DoF[i];
    }
  }

  
  void operator /=(float div)
  {
    this->value /= div;
    for (size_t i = 0; i < DoF.size(); i++)
    {
      this->DoF[i] /= div;
    }
  }

  void print( )
  {
    std::cout << "score: " << value << ";\t 6DoF: [" 
		<< DoF[0] << ", " << DoF[1] << ", " << DoF[2] << ", "
	   	<< DoF[3] << ", " << DoF[4] << ", " << DoF[5] << "]" << std::endl;
  }

};


#endif
