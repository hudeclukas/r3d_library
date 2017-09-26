// Runner.cpp : Defines the entry point for the console application.
//
#include <pcl/common/common_headers.h>
#include "stdafx.h"

#include "Exporter.h"
#include "Polygon.h"
#include "Wall.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("cornerPoints.pcd", *cloud1) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr wall(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("strop2.pcd", *cloud2) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	
	// http://people.sc.fsu.edu/~jburkardt/data/ply/f16.ply
	pcl::PolygonMesh mesh;
	pcl::io::loadPLYFile("f16.ply", mesh);

	std::vector<r3d::obj::Wall> walls;
	r3d::obj::Wall wall1(*cloud1);
	r3d::obj::Wall wall2(*cloud2);
	walls.push_back(wall1);
	walls.push_back(wall2);
	
	r3d::exporter::Exporter e;
	e.addWalls(walls);
	e.addMesh(mesh);
	e.writeDXF("wall_and_mesh.dxf");

	return 0;
}
