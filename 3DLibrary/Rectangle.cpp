#include "stdafx.h"
#include "Rectangle.h"



r3d::prims::Rectangle::Rectangle()
{
	m_planeCoefficients.resize(4);
}

r3d::prims::Rectangle::Rectangle(Eigen::Vector4f coefficients)
{
	m_planeCoefficients.resize(4);
	setCoefficients(coefficients);
}

r3d::prims::Rectangle::Rectangle(r3d::prims::Primitives *prims)
{
	m_planeCoefficients.resize(4);
	this->m_coordinates = prims->getCoordinates();
}

r3d::prims::Rectangle::Rectangle(pcl::PointXYZ A, pcl::PointXYZ B, pcl::PointXYZ C, pcl::PointXYZ D)
{
	m_planeCoefficients.resize(4);
	setCoordinates(A, B, C, D);
}

r3d::prims::Rectangle::~Rectangle()
{
}

void r3d::prims::Rectangle::addCoordinate(const pcl::PointXYZ point)
{
	if (m_coordinates.size() == 4)
	{
		m_coordinates.erase(m_coordinates.begin());
	}
	m_coordinates.push_back(point);
}

void r3d::prims::Rectangle::setCoordinates(pcl::PointXYZ A, pcl::PointXYZ B, pcl::PointXYZ C, pcl::PointXYZ D)
{
	m_coordinates.push_back(A);
	m_coordinates.push_back(B);
	m_coordinates.push_back(C);
	m_coordinates.push_back(D);
}

void r3d::prims::Rectangle::setCoefficients(Eigen::Vector4f &coeff)
{
	for (size_t i = 0; i < coeff.size(); i++)
	{
		m_planeCoefficients[i] = coeff[i];
	}
}

Eigen::Vector4f &r3d::prims::Rectangle::getCoefficients()
{
	return m_planeCoefficients;
}