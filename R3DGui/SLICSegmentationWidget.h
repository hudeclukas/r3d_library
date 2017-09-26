#pragma once
#include "MethodWidget.h"

namespace r3d {
	class SLICSegmentationWidget :
		public MethodWidget
	{
	public:
		SLICSegmentationWidget();
		~SLICSegmentationWidget();

		void visualise(pcl::visualization::PCLVisualizer* viewer) override;
		bool calculate() override;

	private:
		/*!
		*	\brief Output XYZRGB point cloud needed for test visualisation
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmented_cloud;
	};

}