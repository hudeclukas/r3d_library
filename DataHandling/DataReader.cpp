#include "stdafx.h"
#include "DataReader.h"

template <typename PointT>
pcl::PointCloud<PointT>* r3d::io::loadCsvFile(const std::string fileName)
{
	return nullptr;
}

template<>
pcl::PointCloud<pcl::PointXYZ>* r3d::io::loadCsvFile(const std::string fileName)
{
	return r3d::parser::InputDataParser::parseXYZCsv(fileName);
}

template<>
pcl::PointCloud<pcl::PointXYZRGBNormal>* r3d::io::loadCsvFile(const std::string fileName)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud_p = new pcl::PointCloud<pcl::PointXYZRGBNormal>();
	pcl::PointCloud<pcl::PointXYZ> *cloud_x = r3d::parser::InputDataParser::parseXYZCsv(fileName);
	pcl::copyPointCloud(*cloud_x, *cloud_p);
	return cloud_p;
}