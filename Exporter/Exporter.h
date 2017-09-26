#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include "Wall.h"

//#include "Primitives.h"
//#include "Rectangle.h"
/*!
*  \brief     DXF exporter class
*  \details   This class exports data into DXF (a CAD format).
*  \author    Michal Löffler
*  \version   1.0
*  \date      29. 2. 2016
*  \warning   We are using "trial" version of sgCore. I have no idea how long will this code work.
*/

namespace r3d {
	namespace exporter {
		class Exporter
		{
		public:
			Exporter();
			virtual ~Exporter();
			
			bool addWalls(std::vector<r3d::obj::Wall> walls);
			bool addMesh(pcl::PolygonMesh mesh);
			/*
			bool addSphere(pcl::PointXYZ center, int radius);
			bool addCylinder(pcl::PointXYZ baseCenter, int height, int radius);
			*/
						
			bool writeDXF(const char* filename);
			bool writeOBJ(const char* filename);	// not implemented
		};
	}
}