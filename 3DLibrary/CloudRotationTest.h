#pragma once

#ifndef __CLOUDROTATIONTEST__
#define __CLOUDROTATIONTEST__
#include "Method.h"
#include "ClosedSpace.h"


namespace pcl{
	namespace visualization{
		class PCLVisualizer;
	}
}

namespace r3d {
	namespace mtds {
		class CloudRotationTest :
			public Method
		{
		public:
			CloudRotationTest();
			~CloudRotationTest();

			virtual bool calculate();
			virtual void setConfiguration(TiXmlNode* param);
			virtual void visualise(pcl::visualization::PCLVisualizer* viewer);

			pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud;
	
			
		private:
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr> > polygon;
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr> > segmentedClusters; // segmentedClusters is vector of PointClouds <- here are stored segmented "inside" clusters.
			pcl::PointCloud<pcl::PointXYZ>::Ptr klaud;

			space::ClosedSpace m_dataSet;

			int m_minPointsCount = 6000;
			float m_sphereSize = 0.4;
		};
	}
}

#endif __CLOUDROTATIONTEST__