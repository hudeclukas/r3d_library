#pragma once
#include "MethodWidget.h"

namespace r3d {
	class OutliersWidget :
		public MethodWidget
	{
	public:
		OutliersWidget();
		~OutliersWidget();

		bool calculate() override;
		void visualise(pcl::visualization::PCLVisualizer* viewer) override;

	private:
		/*!
		* \brief Point Cloud of examined clustered planes outliers
		* Usefull for further segmentation wall objects like switches, paintings,
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeOutliers;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeInliers;
	};
}