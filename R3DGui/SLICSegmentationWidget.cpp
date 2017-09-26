#include "stdafx.h"
#include "SLICSegmentationWidget.h"
#include "SlicBasedSegmentation.h"
#include <boost/make_shared.hpp>


r3d::SLICSegmentationWidget::SLICSegmentationWidget()
	: m_segmented_cloud(nullptr)
{
	m_widgetName = "Segmentation SLIC";
}


r3d::SLICSegmentationWidget::~SLICSegmentationWidget()
{
}

void r3d::SLICSegmentationWidget::visualise(pcl::visualization::PCLVisualizer* viewer)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_segmented_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(m_segmented_cloud, rgb, "Segmented cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Segmented cloud");
}

bool r3d::SLICSegmentationWidget::calculate()
{
	if (m_segmented_cloud)
	{
		m_segmented_cloud->clear();
	}
	else
	{
		m_segmented_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	}
	mtds::SlicBasedSegmentation slic;
	slic.setConfiguration(m_configNode->FirstChildElement(slic._config_name));

	std::vector<pcl::PointIndices>* clusters = new std::vector<pcl::PointIndices>();
	slic.segmentClusters(m_dataSet, m_segmented_cloud, clusters);


	delete clusters;
	return true;
}
