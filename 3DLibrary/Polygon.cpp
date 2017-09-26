#include "stdafx.h"
#include "Polygon.h"


r3d::prims::Polygon::Polygon()
{
}

r3d::prims::Polygon::Polygon(Eigen::Vector4f coefficients) : 
m_planeCoefficients(coefficients)
{
}

r3d::prims::Polygon::Polygon(pcl::PointCloud<pcl::PointXYZ> coordinates)
{
	m_coordinates = coordinates;
}

r3d::prims::Polygon::Polygon(r3d::prims::Primitives* prims)
{
	m_coordinates = prims->getCoordinates();
}

r3d::prims::Polygon::~Polygon()
{
}

void r3d::prims::Polygon::addCoordinate(const pcl::PointXYZ point)
{
	m_coordinates.push_back(point);
}

void r3d::prims::Polygon::addCoordinates(const pcl::PointCloud<pcl::PointXYZ> coordinateCloud)
{
	pcl::copyPointCloud(coordinateCloud, m_coordinates);
}

void r3d::prims::Polygon::setCoefficients(const Eigen::Vector4f coeff)
{
	m_planeCoefficients = coeff;
}

Eigen::Vector4f& r3d::prims::Polygon::getCoefficients()
{
	return m_planeCoefficients;
}
