#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include "Exporter.h"
#include "Polygon.h"
#include "sgCore.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string> 

r3d::exporter::Exporter::Exporter()
{
	sgInitKernel();
}

r3d::exporter::Exporter::~Exporter()
{
	sgFreeKernel();
}

bool comparePoint(pcl::PointXYZ p1, pcl::PointXYZ p2){
	if (p1.x != p2.x)
		return p1.x > p2.x;
	else if (p1.y != p2.y)
		return  p1.y > p2.y;
	else
		return p1.z > p2.z;
}

bool equalPoint(pcl::PointXYZ p1, pcl::PointXYZ p2){
	if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
		return true;
	return false;
}

bool r3d::exporter::Exporter::addWalls(std::vector<r3d::obj::Wall> walls)
{

	for (int i = 0; i < walls.size(); i++)
	{
		
		r3d::prims::Polygon polygon = walls[i].getShape();
		pcl::PointCloud<pcl::PointXYZ> coords = polygon.getCoordinates();
		pcl::io::savePCDFileASCII(std::to_string(i)+"_.pcd",coords);
		int cornerCount = coords.width;	// number of corner points = number of edge lines
	
		/* Remove duplicate points - but it only compares consecutive points*/
		for (int i = 0; i< coords.width; i++) {
			pcl::PointXYZ pointA = coords.points[i];
			pcl::PointXYZ pointB;
			if (i + 1 > coords.width - 1)
			{
				pointB = coords.points[0];
				//break;
			}
			else
			{
				pointB = coords.points[i + 1];

				
			}
			if ((pointA.x == pointB.x) &&
				(pointA.y == pointB.y) &&
				(pointA.z == pointB.z))
			{
				coords.erase(coords.begin() + (i + 1)%coords.width);
				i--;

			}
			
		}
		/* This should be better way for removing them:
		auto unique_end = std::unique(coords.begin(), coords.end(), equalPoint);
		coords.erase(unique_end, coords.end());
		*/
		cornerCount = coords.width;

		pcl::io::savePCDFileASCII(std::to_string(i) + "__.pcd", coords);
		if ((cornerCount >= 3) && (cornerCount <= 200))
		{

			sgCObject**  edges = (sgCObject**)malloc(cornerCount*sizeof(sgCObject*));
			int ommitedPointsCount = 0;
			
			for (int j = 0; j < cornerCount; j++)
			{
				
				if (j != cornerCount - 1)
				{
					edges[j] = sgCreateLine(
						coords.points[j].x, coords.points[j].y, coords.points[j].z,
						coords.points[j + 1].x, coords.points[j + 1].y, coords.points[j + 1].z
						);
				}
				else
				{
					edges[j] = sgCreateLine(
						coords.points[j].x, coords.points[j].y, coords.points[j].z,
						coords.points[0].x, coords.points[0].y, coords.points[0].z
						);
				}
			}

			sgCContour* contour;
			sgC3DObject* face;

			contour = sgCContour::CreateContour(&edges[0], cornerCount - ommitedPointsCount);
			face = (sgC3DObject*)sgSurfaces::Face((const sgC2DObject&)*contour, NULL, 0);
			sgGetScene()->AttachObject(face);

			free(edges);
			sgC3DObject::DeleteObject(face);
			sgC3DObject::DeleteObject(contour);
		}
	}

	return true;
}


bool r3d::exporter::Exporter::addMesh(pcl::PolygonMesh mesh)
{
	/*!
	*	Convert each point from mesh to SG_POINT
	*/

	pcl::PointCloud<pcl::PointXYZ> vertsPCL;
	pcl::fromPCLPointCloud2(mesh.cloud, vertsPCL);
	//SG_POINT *vert = (SG_POINT*)malloc(vertsPCL.size()*sizeof(SG_POINT));
	/*
	std::cout << "size:" << vertsPCL.size() << std::endl;
	std::cout << "widht:" << vertsPCL.width << std::endl;
	*/

	SG_POINT *vert = new SG_POINT[vertsPCL.size()];

	for (int i = 0; i < vertsPCL.size(); i++)
	{
		vertsPCL.points[i];
		vertsPCL.at(i);
		vert[i] = { vertsPCL.points[i].x, vertsPCL.points[i].y, vertsPCL.points[i].z };
	}
	
	
	/*!
	*	Take every triangle and convert it to SG_INDEX_TRIANGLE
	*/

	std::vector<::pcl::Vertices> indicesPCL = mesh.polygons;
	SG_INDEX_TRIANGLE *indexes = new SG_INDEX_TRIANGLE[indicesPCL.size()];

	for (int i = 0; i < indicesPCL.size(); i++)
	{
		indexes[i].ver_indexes[0] = indicesPCL[i].vertices[0];
		indexes[i].ver_indexes[1] = indicesPCL[i].vertices[1];
		indexes[i].ver_indexes[2] = indicesPCL[i].vertices[2];
	}
	
	sgGetScene()->AttachObject(sgFileManager::ObjectFromTriangles(vert, vertsPCL.size(), indexes, indicesPCL.size()));
	
	return true;
}


bool r3d::exporter::Exporter::writeDXF(const char * filename)
{
	return sgFileManager::ExportDXF(sgGetScene(), filename);
}

bool r3d::exporter::Exporter::writeOBJ(const char * filename)
{
	pcl::PolygonMesh mesh;
	
	//mesh.cloud
	//mesh.cloud = msg;
	//mesh.polygons = polygons;

	/*
	pcl::io::savePLYFile(const std::string &  	file_name,
		const pcl::PolygonMesh &  	mesh,
		unsigned  	precision = 5
		)

		pcl::io::savePLYFileBinary 	( 	const std::string &  	file_name,
		const pcl::PolygonMesh &  	mesh
		)

		Save a PolygonMesh object into a PLY file.
		pcl::io::savePolygonFilePLY 	( 	const std::string &  	file_name,
		const pcl::PolygonMesh &  	mesh,
		const bool  	binary_format = true
		)
	*/
	return true;
}