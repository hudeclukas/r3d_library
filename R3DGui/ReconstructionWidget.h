#pragma once
#include "MethodWidget.h"
#include <PlaneSegment.h>

namespace r3d {
	class ReconstructionWidget :
		public MethodWidget
	{
	public:
		ReconstructionWidget();
		~ReconstructionWidget();

		void visualise(pcl::visualization::PCLVisualizer* viewer) override;
		bool calculate() override;

	private:
		/*!
		* \brief Contains extracted planes selected/segmented by "growing region"
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmentedPlanes;
		/*!
		* \brief Contains extracted borders(edges) between planes, that weren't connected to any plane by "growing region"
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_segmentedBorder;
		/*!
		* \brief Examined clusters around planes segmented by growing region
		*/
		std::vector<r3d::prims::PlaneSegment> m_planeSegments;
		/*!
		* \brief Point Cloud of examined clustered planes outliers
		* Usefull for further segmentation wall objects like switches, paintings,
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeOutliers;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeInliers;
	};
}
