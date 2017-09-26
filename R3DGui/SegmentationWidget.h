#pragma once
#include "MethodWidget.h"

namespace r3d {
	class SegmentationWidget :
		public MethodWidget
	{
	public:
		SegmentationWidget();
		~SegmentationWidget();

		bool calculate() override;
		void visualise(pcl::visualization::PCLVisualizer* viewer) override;

	private:

		/*!
		*	\brief Output XYZRGB point cloud needed for test visualisation
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmented_cloud;
		/*!
		*	\brief Set of dominant normals of the dataset, needed for visiulisation
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_dominantNormals;
	};
}