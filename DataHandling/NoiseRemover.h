#ifndef __NOISEREMOVER__
#define __NOISEREMOVER__

#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "tinyxml.h"

namespace r3d {
	namespace remover {
		/*! \brief Removes noise from point cloudData parser, whatever data type from ASCII to pointcloud XYZ coordinate system
		*   \author Martin Jurik
		*/
		class NoiseRemover
		{
		public:
			NoiseRemover();
			~NoiseRemover();
			void setConfiguration(TiXmlNode* param);
			pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise(pcl::PointCloud<pcl::PointXYZRGBNormal>* cloud);
		private:
			unsigned short m_MeanK;
			float m_StddevMulThresh;
		};
	}
}

#endif __NOISEREMOVER__