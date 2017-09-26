#include "stdafx.h"
#include "Line.h"


r3d::prims::Line::Line()
{
	m_linePoints = nullptr;
	m_lineCoefficients.resize(6);
}

r3d::prims::Line::Line(Eigen::VectorXf &coeff, pcl::PointCloud<pcl::PointXYZRGB> linePoints)
{
	m_lineCoefficients.resize(6);
	setCoefficients(coeff);
	setCloud(linePoints);
}

r3d::prims::Line::~Line()
{
}

void r3d::prims::Line::setCoefficients(Eigen::VectorXf &coeff)
{
	for (size_t i = 0; i < coeff.size(); i++)
	{
		m_lineCoefficients[i] = coeff[i];
	}
}

void r3d::prims::Line::setCoefficients(float pointX, float pointY, float pointZ, float directionX, float directionY, float directionZ)
{
	m_lineCoefficients[0] = pointX;
	m_lineCoefficients[1] = pointY;
	m_lineCoefficients[2] = pointZ;
	m_lineCoefficients[3] = directionX;
	m_lineCoefficients[4] = directionY;
	m_lineCoefficients[5] = directionZ;
}

void r3d::prims::Line::setCloud(pcl::PointCloud<pcl::PointXYZRGB> linePoints)
{
	m_linePoints = linePoints.makeShared();
}

Eigen::VectorXf r3d::prims::Line::getCoefficients()
{
	return m_lineCoefficients;
}


pcl::PointCloud<pcl::PointXYZRGB> r3d::prims::Line::getPoints()
{
	return *m_linePoints;
}
