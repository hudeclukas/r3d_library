#include "stdafx.h"
#include "ColorGenerator.h"
#include "Object.h"


r3d::obj::Object::Object()
{	
	unsigned char* resultColors = r3d::ColorGenerator::getInstance().getNextColor();
	r = resultColors[0];
	g = resultColors[1];
	b = resultColors[2];
}

r3d::obj::Object::Object(pcl::PointCloud<pcl::PointXYZ> objectData, std::string name)
{
	setName(name);
	setObjectData(objectData);
	
	unsigned char* resultColors = r3d::ColorGenerator::getInstance().getNextColor();
	r = resultColors[0];
	g = resultColors[1];
	b = resultColors[2];
}

r3d::obj::Object::~Object()
{
}

void r3d::obj::Object::setObjectData(pcl::PointCloud<pcl::PointXYZ> objectData)
{
	pcl::copyPointCloud(objectData, m_originalData);
}

void r3d::obj::Object::setObjectData(pcl::PointCloud<pcl::PointXYZRGB> objectData)
{
	pcl::copyPointCloud(objectData, m_originalData);
}
