#include "stdafx.h"
#include "ClosedSpace.h"


r3d::space::ClosedSpace::ClosedSpace() //:
	//m_reducedDataXYZRGB(nullptr)
{
	//m_inputXYZ = new pcl::PointCloud<pcl::PointXYZ>();
	//m_inputRGB = new pcl::PointCloud<pcl::RGB>();
	//m_inputNormal = new pcl::PointCloud<pcl::Normal>();
}

r3d::space::ClosedSpace::ClosedSpace(std::vector<std::string> source, std::string name) : m_name(name)
{
	//m_inputXYZ = new pcl::PointCloud<pcl::PointXYZ>();
	//m_inputRGB= new pcl::PointCloud<pcl::RGB>();
	//m_inputNormal = new pcl::PointCloud<pcl::Normal>();
	m_source = source;
}

r3d::space::ClosedSpace::~ClosedSpace()
{
	//delete m_inputXYZ;
	//delete m_inputRGB;
	//delete m_inputNormal;
	//delete m_reducedDataXYZRGB;
}

void r3d::space::ClosedSpace::visualise(pcl::visualization::PCLVisualizer* viewer)
{
	for each (r3d::obj::Wall wall in m_walls)
	{
		double r = (rand() % 255) / 255;
		double g = (rand() % 255) / 255;
		double b = (rand() % 255) / 255;

		viewer->addPolygon<pcl::PointXYZ>(wall.getShape().getCoordinates().makeShared(), r, g, b, wall.getName());
	}

	// TODO !for each r3d::obj::Object!
}

void r3d::space::ClosedSpace::visualiseOriginalData(pcl::visualization::PCLVisualizer* viewer) const
{
	if (!m_inputXYZ.empty())
	{
		viewer->addPointCloud<pcl::PointXYZ>(m_inputXYZ.makeShared(), m_name + "XYZ");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_name + "XYZ");
	}
	if (!m_inputRGB.empty() && !m_inputXYZ.empty())
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(m_inputXYZ, *dataXYZRGB);
		pcl::copyPointCloud(m_inputRGB, *dataXYZRGB);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorVisualisation(dataXYZRGB);
		viewer->addPointCloud<pcl::PointXYZRGB>(dataXYZRGB, colorVisualisation, m_name + "XYZRGB");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, m_name + "XYZRGB");
	}
	//if (m_inputRGB != nullptr && m_inputXYZ != nullptr && m_inputNormal != nullptr)
	//{

	//}

	//if (m_inputRGB != nullptr && m_inputNormal != nullptr)
	//{

	//}
}

void r3d::space::ClosedSpace::addObject(r3d::obj::Object &object)
{
	m_objects.push_back(&object);
	// reduceCloudByData(m_data, object.getObjectData();
	return;
}

int r3d::space::ClosedSpace::addWall(r3d::obj::Wall &wall)
{
	assert(wall.getShape().empty() == false);
	m_walls.push_back(wall);
	return m_walls.size();
}
