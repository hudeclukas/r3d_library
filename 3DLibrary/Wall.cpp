#include "stdafx.h"
#include "Wall.h"


r3d::obj::Wall::Wall()
{
}

r3d::obj::Wall::Wall(pcl::PointCloud<pcl::PointXYZ> coordinates)
{
	setShape(prims::Polygon(coordinates));
}

r3d::obj::Wall::~Wall()
{
}

void r3d::obj::Wall::setShape(r3d::prims::Polygon shape)
{
	m_shape = shape;
}

void r3d::obj::Wall::addCornerPoint(const pcl::PointXYZ corner)
{
	m_shape.addCoordinate(corner);
}

void r3d::obj::Wall::addCornerPoint(const Eigen::Vector3f corner)
{
	m_shape.addCoordinate(pcl::PointXYZ(corner[0], corner[1], corner[2]));
}

pcl::PointCloud<pcl::PointXYZ> &r3d::obj::Wall::getCornerPoints()
{
	return m_shape.getCoordinates();
}

int r3d::obj::Wall::coordinatesCount()
{
	return m_shape.getCoordinates().size();
}

void r3d::obj::Wall::setIsValid(bool isValid)
{
	m_isValid = isValid;
}

bool r3d::obj::Wall::isValidWall() const
{
	return m_isValid;
}
