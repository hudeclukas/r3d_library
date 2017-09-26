#include "stdafx.h"
#include "SegmentationWidget.h"
#include "BirkysNormalSegmentation.h"
//#include <pcl/segmentation/region_growing_rgb.h>


r3d::SegmentationWidget::SegmentationWidget()
	: m_segmented_cloud(nullptr)
{
	m_widgetName = "Segmentation (birkys)";
}


r3d::SegmentationWidget::~SegmentationWidget()
{
}

bool r3d::SegmentationWidget::calculate()
{
	/* for extended use 
	
	//!< Loading needed data and define needed variables
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m_dataSet.getXYZData().makeShared();
	pcl::PointCloud<pcl::Normal>::Ptr normals = m_dataSet.getNormalData().makeShared();
	pcl::PointCloud<pcl::PointXYZ>::Ptr normalHistogram;
	pcl::PointCloud<pcl::PointXYZ>::Ptr halfSphere;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
	std::vector <pcl::PointIndices> clusters;
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;

	//!< Calculate normals if they are not avaible
	if (normals->empty())
	{
		normals = estimateNormals(cloud, m_OMPNumberOfCores, m_NENumberOfNeighbours);
	}

	//!< Create "histogram" of normals from the calculated normals
	normalHistogram = createHistogramOfNormals(normals);

	//!< Create a half sphere of potencial dominant directions
	halfSphere = createHalfSphere(m_PointsPerHalfSphere);

	//!< searching for dominant normals
	m_dominantNormals = findDominantNormals(normalHistogram, m_NumberOfDominantRegions, m_DominantNormalsSearchRadius, halfSphere);

	//!< Labeling the points of the cloud according to the found dominant normals
	cloudRGB = labelPointsAccordingToDominantNormals(cloud, normals, m_dominantNormals);

	//!< Region growing 
	reg = regionGrowingRGB(cloudRGB, m_GRNumberOfNeighbours, m_PointColorThreshold, m_MinClusterSize, &clusters);
	m_segmented_cloud = reg.getColoredCloud();

	return true;
	*/

	std::vector <pcl::PointIndices> clusters;
	r3d::mtds::BirkysNormalSegmentation segmentation;

	segmentation.setConfiguration(m_configNode->FirstChildElement(segmentation._config_name));
	segmentation.setData(m_dataSet.getXYZData());
	//problem with vtk dataset... wrong normals
	if (!m_dataSet.getNormalData().empty())
	{
		segmentation.setData(m_dataSet.getNormalData());
	}
	m_segmented_cloud = segmentation.segmentClusters(&clusters);
	m_dominantNormals = segmentation.getDominantNormalsDirections();

	return true;
}

void r3d::SegmentationWidget::visualise(pcl::visualization::PCLVisualizer* viewer)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_segmented_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(m_segmented_cloud, rgb, "XYZRGB");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "XYZRGB");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(m_segmented_cloud, m_dataSet.getNormalData().makeShared(), 10, 0.05, "normals");

	viewer->addArrow<pcl::PointXYZ>(pcl::PointXYZ(0.0 + 3 * m_dominantNormals->points[0].x, 0.0 + 3 * m_dominantNormals->points[0].y, 0.0 + 3 * m_dominantNormals->points[0].z), pcl::PointXYZ(0.0, 0.0, 0.0), 1.0, 0.0, 0.0, false, "a1");
	viewer->addArrow<pcl::PointXYZ>(pcl::PointXYZ(0.0 + 3 * m_dominantNormals->points[1].x, 0.0 + 3 * m_dominantNormals->points[1].y, 0.0 + 3 * m_dominantNormals->points[1].z), pcl::PointXYZ(0.0, 0.0, 0.0), 0.0, 1.0, 0.0, false, "a2");
	viewer->addArrow<pcl::PointXYZ>(pcl::PointXYZ(0.0 + 3 * m_dominantNormals->points[2].x, 0.0 + 3 * m_dominantNormals->points[2].y, 0.0 + 3 * m_dominantNormals->points[2].z), pcl::PointXYZ(0.0, 0.0, 0.0), 0.0, 0.0, 1.0, false, "a3");
}
