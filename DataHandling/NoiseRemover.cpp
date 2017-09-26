#include "stdafx.h"
#include "ParameterGrabber.h"
#include "NoiseRemover.h"

r3d::remover::NoiseRemover::NoiseRemover() 
{
	m_MeanK = 10;
	m_StddevMulThresh = 1.5;
}

r3d::remover::NoiseRemover::~NoiseRemover()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::remover::NoiseRemover::removeNoise(pcl::PointCloud<pcl::PointXYZRGBNormal>* originalData)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*originalData, *original_cloud);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(original_cloud);
	sor.setMeanK(m_MeanK);
	sor.setStddevMulThresh(m_StddevMulThresh);
	sor.filter(*cloud_filtered);

	return cloud_filtered;
}

void r3d::remover::NoiseRemover::setConfiguration(TiXmlNode* param) {
	if (param != nullptr && param->FirstChild() != nullptr) {
		m_MeanK = std::atoi(PGrabber::getParameter(param, "MeanK"));
		m_StddevMulThresh = std::atof(PGrabber::getParameter(param, "StddevMulThresh"));
		
	}
}
