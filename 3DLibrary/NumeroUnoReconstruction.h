#pragma once

#ifndef __NUMEROUNORECONSTRUCTION__
#define __NUMEROUNORECONSTRUCTION__
#include "Method.h"
#include "ClosedSpace.h"


namespace pcl{
	namespace visualization{
		class PCLVisualizer;
	}
}

namespace r3d {
	namespace mtds {
		class NumeroUnoReconstruction :
			public Method
		{
		public:
			NumeroUnoReconstruction();
			~NumeroUnoReconstruction();

			virtual bool calculate();
			virtual void setConfiguration(TiXmlNode* param);
			virtual void visualise(pcl::visualization::PCLVisualizer* viewer);

		protected:
			int calculate_windows(float dist, float m_point);
			float calculate_mean_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cld);
			float calculate_total_distance_z(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, float* min, float* max);
			int meanshift(float* cntr, float b_width, std::vector<float> points, int* dir);
			void calculate_normals(pcl::PointCloud<pcl::Normal>::Ptr nrmls, pcl::PointCloud<pcl::PointXYZ>::Ptr cld, int k);
			void calculate_projected_points(std::vector<float>* proj_points, pcl::PointCloud<pcl::Normal>::Ptr nrmls, pcl::PointCloud<pcl::PointXYZ>::Ptr cld, pcl::PointCloud<pcl::PointXYZ>::Ptr z_cld, float err);
			float get_centroid(float* pos, float b_width, std::vector<float> proj_points);

		private:
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_outputCloud;
			space::ClosedSpace m_dataSet;
		};
	}
}

#endif __NUMEROUNORECONSTRUCTION__